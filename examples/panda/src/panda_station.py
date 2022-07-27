import numpy as np

import pydrake
from pydrake.all import (
        LoadModelDirectives, ProcessModelDirectives, GeometrySet, Parser,
        DiagramBuilder, Diagram, MultibodyPlant, 
        Demultiplexer, InverseDynamicsController, Adder, 
        PassThrough, StateInterpolatorWithDiscreteDerivative, RigidTransform, 
        RollPitchYaw, RotationMatrix, Integrator, SchunkWsgPositionController, 
        MakeMultibodyStateToWsgStateSystem, FirstOrderLowPassFilter,
        DrakeVisualizer, ConnectContactResultsToDrakeVisualizer, 
        FixedOffsetFrame, MultibodyPlantConfig, AddMultibodyPlant, FindResourceOrThrow)
from .differential_ik import PseudoInverseController
from .scenarios import AddPanda, AddHand
# from .utils_drake import AddMultibodyTriad, AddPackagePaths


def deg_to_rad(deg):
    return deg*np.pi/180.0


class PandaStation(Diagram):

    def __init__(self, 
                 dt=0.002, 
                 visualize_contact=False,
                 diff_ik_filter_hz=-1,
                 contact_solver='sap',
                 panda_joint_damping=200,
                 ):
        Diagram.__init__(self)
        self.dt = dt
        self.visualize_contact = visualize_contact
        self.diff_ik_filter_hz = diff_ik_filter_hz
        self.panda_joint_damping = panda_joint_damping

        # Initialie builder and plant
        self.builder = DiagramBuilder()
        plant_cfg = MultibodyPlantConfig(time_step=dt, 
                                         discrete_contact_solver=contact_solver)
        self.plant, self.sg = AddMultibodyPlant(plant_cfg, self.builder)
        self.plant.set_name("plant")

        self.set_name("panda_station")
        self.object_ids = []
        self.object_poses = []
        self.camera_info = {}   # dictionary in the form name: pose
        self.body_info = [] # path: (name, body_index)

        # Controller plant
        self.controller_plant = self.builder.AddSystem(
                                    MultibodyPlant(time_step=dt)
                                    )


    def set_table(self):
        directive = FindResourceOrThrow("drake/examples/panda/data/models/table/table_top.yaml")
        parser = Parser(self.plant)
        parser.package_map().Add(
            "table",
            pydrake.common.GetDrakePath() + "/examples/panda/data/models/table/")
        table_info = ProcessModelDirectives(LoadModelDirectives(directive), 
                                            self.plant, 
                                            parser)
        table_model_index = table_info[0].model_instance
        table_body_index = self.plant.GetBodyIndices(table_model_index)[0]
        return table_model_index, table_body_index


    def set_panda(self, hand_type='wsg'):
        self.panda = AddPanda(self.plant,
                              joint_damping=self.panda_joint_damping)
        self.hand = AddHand(self.plant, 
                            panda_model_instance=self.panda, 
                            roll=0, 
                            welded=False, 
                            type=hand_type) # offset between EE and hand, roll:-pi/4
        hand_body = self.plant.GetBodyByName("gripper_base", self.hand)

        # self.fix_collisions()
        ee_frame = self.plant.GetFrameByName("panda_link8", self.panda)
        hand_frame = self.plant.GetFrameByName("body_frame", self.hand)
        # AddMultibodyTriad(ee_frame, self.sg, length=.1, radius=0.005)
        # AddMultibodyTriad(hand_frame, self.sg, length=.1, radius=0.005)

        # Set joint acceleration limits - position and velocity limits already specified in SDF
        acc_limit = [15, 7.5, 10, 12.5, 15, 20, 20]
        for joint_ind in range(1, 8):
            joint_name = 'panda_joint' + str(joint_ind)
            joint = self.plant.GetJointByName(joint_name, self.panda)
            joint.set_acceleration_limits([-acc_limit[joint_ind-1]],
                                          [acc_limit[joint_ind-1]])
            # print(joint.acceleration_upper_limits())
        return hand_body

    def get_visualizer(self, use_meshcat=False):
        visualizer = DrakeVisualizer.AddToBuilder(
            builder=self.builder, 
            scene_graph=self.sg,
            )
        return visualizer


    def fix_collisions(self):
        panda_link7 = self.plant.GetFrameByName("panda_link7").body()
        panda_link7 = GeometrySet(
                self.plant.GetCollisionGeometriesForBody(panda_link7)
                )
        panda_hand = self.plant.GetFrameByName("panda_hand").body()
        panda_hand = GeometrySet(
                self.plant.GetCollisionGeometriesForBody(panda_hand)
                )
        self.collision_filter_manager = self.sg.collision_filter_manager()


    def Finalize(self):
        assert self.panda.is_valid(), "No panda model added"
        assert self.hand.is_valid(), "No panda hand model added"

        # Export state ports
        self.builder.ExportOutput(
                self.sg.get_query_output_port(), 
                "geometry_query"
                )
        self.builder.ExportOutput(
                self.plant.get_contact_results_output_port(), 
                "contact_results"
                )
        self.builder.ExportOutput(
                self.plant.get_state_output_port(),
                "plant_continuous_state"
                )

        # Send contact info to visualizer
        if self.visualize_contact:
            ConnectContactResultsToDrakeVisualizer(
                        builder=self.builder, 
                        plant=self.plant, 
                        scene_graph=self.sg,
                )

        # For visualization
        self.builder.ExportOutput(
                self.sg.get_query_output_port(),
                "query_object"
                )
        self.builder.BuildInto(self) 


    def set_arm_controller(self):
        # Export panda joint state outputs with demux
        num_panda_positions = self.plant.num_positions(self.panda)
        demux = self.builder.AddSystem(
                Demultiplexer(2 * num_panda_positions, num_panda_positions)
                )
        self.builder.Connect(
                self.plant.get_state_output_port(self.panda), 
                demux.get_input_port()
                )

        # Initialize plant for the controller
        self.panda_c = AddPanda(self.controller_plant,
                                joint_damping=self.panda_joint_damping)

        # Set joint acceleration limits - position and velocity limits already specified in SDF
        acc_limit = [15, 7.5, 10, 12.5, 15, 20, 20]
        for joint_ind in range(1, 8):
            joint_name = 'panda_joint' + str(joint_ind)
            joint = self.controller_plant.GetJointByName(joint_name, 
                                                         self.panda_c)
            joint.set_acceleration_limits([-acc_limit[joint_ind-1]],
                                          [acc_limit[joint_ind-1]])

        # Add fixed frame for the point between fingers (for ik), to arm (not hand!)
        fingertip_frame = self.controller_plant.AddFrame(
                FixedOffsetFrame(
                    "fingertip_frame", 
                    self.controller_plant.GetFrameByName(
                        "panda_link8", 
                        self.panda_c),
                RigidTransform([0, 0, 0.16]))
            )
        self.controller_plant.Finalize()

        # Add arm controller
        # kp = np.array([2000, 1500, 1500, 1500, 1500, 500, 500])*1.5
        kp = np.array([2000, 2000, 2000, 2000, 2000, 2000, 2000])*2
        kd = 2*np.sqrt(kp)
        ki = np.ones(7)
        self.panda_controller = self.builder.AddSystem(InverseDynamicsController(
                self.controller_plant,
                # kp=[5000]*num_panda_positions,  # 2000, 10000, 5000
                # kd=[250]*num_panda_positions,  # 200, 500, 250
                # ki=[0]*num_panda_positions,   # 0, 0, 0
                kp=kp,
                kd=kd,
                ki=ki,
                has_reference_acceleration=False))
        self.panda_controller.set_name("panda_controller")
        self.builder.Connect(
            self.plant.get_state_output_port(self.panda), 
            self.panda_controller.get_input_port_estimated_state()
            )

        # Add FF torque
        adder = self.builder.AddSystem(Adder(2, num_panda_positions))
        self.builder.Connect(
            self.panda_controller.get_output_port_control(),
            adder.get_input_port(0)
            )
        torque_passthrough = self.builder.AddSystem(
            PassThrough([0]*num_panda_positions)
            )
        self.builder.Connect(
            torque_passthrough.get_output_port(), 
            adder.get_input_port(1)
            )
        self.builder.ExportInput(
            torque_passthrough.get_input_port(), 
            "panda_feedforward_torque"
            )
        self.builder.Connect(
            adder.get_output_port(), 
            self.plant.get_actuation_input_port(self.panda)
            )
        self.builder.Connect(
            adder.get_output_port(), 
            self.controller_plant.get_actuation_input_port(self.panda_c)
            )

        # Add Differential IK
        diff_ik = self.builder.AddSystem(
            PseudoInverseController(self.controller_plant, self.dt)
            )
        diff_ik.set_name("PseudoInverseController")
        
        # Connnect joint measurements to diff ik from demux
        self.builder.Connect(
                        demux.get_output_port(0),
                        diff_ik.GetInputPort("q")
                        )
        self.builder.Connect(
                        demux.get_output_port(1),
                        diff_ik.GetInputPort("qdot")
                        )

        # Connect velocity command to diff ik
        V_WG_command = self.builder.AddSystem(PassThrough(6))
        self.builder.ExportInput(V_WG_command.get_input_port(), "V_WG_command")
        self.builder.Connect(
            V_WG_command.get_output_port(),
            diff_ik.GetInputPort("V_WG")
            )
        # self.builder.Connect(
        #     self.plant.get_body_spatial_velocities_output_port(),
        #     diff_ik.GetInputPort("V")
        #     )

        # Optional: pass diff ik output through low pass filter
        if self.diff_ik_filter_hz > 0:
            lp_filter = self.builder.AddSystem(
                FirstOrderLowPassFilter(1/self.diff_ik_filter_hz, 7)
            )
            self.builder.Connect(
                diff_ik.get_output_port(),
                lp_filter.get_input_port(0),
            )
            diff_ik_output_port = lp_filter.get_output_port(0)
        else:
            diff_ik_output_port = diff_ik.get_output_port()

        # Add diff ik output and v_joint input
        adder_diffik_vj = self.builder.AddSystem(Adder(2, num_panda_positions))
        self.builder.Connect(
            diff_ik_output_port,
            adder_diffik_vj.get_input_port(0)
            )
        v_j_input = self.builder.AddSystem(PassThrough(num_panda_positions))
        self.builder.Connect(
            v_j_input.get_output_port(), 
            adder_diffik_vj.get_input_port(1)
            )
        self.builder.ExportInput(
            v_j_input.get_input_port(), 
            "V_J_command"
            )

        # Feed to integrator
        self.state_integrator = self.builder.AddSystem(Integrator(7))
        self.state_integrator.set_name("state_integrator")
        self.builder.Connect(
            adder_diffik_vj.get_output_port(), 
            self.state_integrator.get_input_port()
            )

        # Add interpolatorto find velocity command based on positional commands, between DIK and IDC
        self.state_interpolator = self.builder.AddSystem(
            StateInterpolatorWithDiscreteDerivative(
                num_panda_positions, 
                self.dt, 
                suppress_initial_transient=False)
            )
        self.state_interpolator.set_name("state_interpolator")
        self.builder.Connect(
            self.state_integrator.get_output_port(), 
            self.state_interpolator.get_input_port()
            )
        self.builder.ExportOutput(
            self.state_interpolator.get_output_port(),
            "panda_desired_state"
            )

        # Add IK result port
        ik_input = self.builder.AddSystem(PassThrough(2*num_panda_positions))
        self.builder.ExportInput(
            ik_input.get_input_port(), 
            "ik_result"
            )

        # Add state_interpolator output and ik
        adder_interp_ik = self.builder.AddSystem(Adder(2, 2*num_panda_positions))
        self.builder.Connect(
            self.state_interpolator.get_output_port(),
            adder_interp_ik.get_input_port(0)
            )
        self.builder.Connect(
            ik_input.get_output_port(),
            adder_interp_ik.get_input_port(1)
            )
        
        # Feed to controller
        self.builder.Connect(
            adder_interp_ik.get_output_port(), 
            self.panda_controller.get_input_port_desired_state()
            )


    def set_hand_controller(self):
        self.hand_controller = self.builder.AddSystem(
                SchunkWsgPositionController(kp_command=1000, 
                                            kd_command=5, 
                                            default_force_limit=80)
            )
        self.hand_controller.set_name("hand_controller")
        self.builder.Connect(
            self.hand_controller.get_generalized_force_output_port(),
            self.plant.get_actuation_input_port(self.hand)
            )
        self.builder.Connect(
            self.plant.get_state_output_port(self.hand),
            self.hand_controller.get_state_input_port()
            )
        # self.builder.ExportInput(
        #                self.hand_controller.get_desired_position_input_port(),
        #                "hand_position")
        # self.builder.ExportInput(
        #                 self.hand_controller.get_force_limit_input_port(),
        #                 "hand_force_limit")
        hand_mbp_state_to_hand_state = self.builder.AddSystem(
                                MakeMultibodyStateToWsgStateSystem()
                                )
        self.builder.Connect(
            self.plant.get_state_output_port(self.hand), hand_mbp_state_to_hand_state.get_input_port()
            )
        self.builder.ExportOutput(
            hand_mbp_state_to_hand_state.get_output_port(), "hand_state_measured"
            )
        self.builder.ExportOutput(
            self.hand_controller.get_grip_force_output_port(), "hand_force_measured"
            )

        # add passthrough for hand position
        hand_source = self.builder.AddSystem(PassThrough(1))
        hand_source.set_name("hand_position_command_pt")
        self.builder.Connect(
            hand_source.get_output_port(), 
            self.hand_controller.GetInputPort("desired_position")
            )
        self.builder.ExportInput(
            hand_source.get_input_port(), 
            "hand_position_command"
            )


    ############################## Helper ##############################

    def AddModelFromFile(self, path, X_WO, name=None):
        parser = Parser(self.plant)
        if name is None:
            num = str(len(self.object_ids))
            name = "added_model_" + num
        model_index = parser.AddModelFromFile(path, name)
        indices = self.plant.GetBodyIndices(model_index)
        # assert len(indices) == 1, "Currently, we only support adding models with one body"
        # self.body_info.append((path, name, indices[0]))
        # self.object_ids.append(indices[0])
        # self.object_poses.append(X_WO)
        # return model_index, indices[0]
        return model_index, indices


    ############################## Setter ##############################

    def set_panda_position(self, station_context, q):
        num_panda_positions = self.plant.num_positions(self.panda)
        assert len(q) == num_panda_positions, "Incorrect size of q, needs to be 7"
        plant_context = self.GetSubsystemContext(self.plant, station_context)
        self.plant.SetPositions(plant_context, self.panda, q)


    def set_panda_velocity(self, station_context, state, v):
        num_panda_positions = self.plant.num_positions(self.panda)
        assert len(v) == num_panda_positions, "Incorrect size of v, needs to be 7"
        plant_context = self.GetSubsystemContext(self.plant, station_context)
        plant_state = self.GetMutableSubsystemState(self.plant, state)
        self.plant.SetVelocities(plant_context, plant_state, self.panda, v)


    def set_hand_position(self, station_context, q):
        plant_context = self.GetSubsystemContext(self.plant, station_context)
        self.plant.SetPositions(plant_context, self.hand, [q/2.0, q/2.0])


    def set_hand_velocity(self, station_context, state, v):
        plant_context = self.GetSubsystemContext(self.plant, station_context)
        plant_state = self.GetMutableSubsystemState(self.plant, state)
        self.plant.SetVelocities(plant_context, plant_state, self.hand, [v/2.0, v/2.0])


    ############################## Getter ##############################

    # def get_simulator(self, context):
    #     from pydrake.all import Simulator
    #     context = self.diagram.CreateDefaultContext()
    #     simulator = Simulator(self, context)
    #     # system = simulator.get_system()
    #     simulator.Initialize()
    #     return simulator

    def get_panda_position(self, station_context):
        plant_context = self.GetSubsystemContext(self.plant, station_context)
        return self.plant.GetPositions(plant_context, self.panda)


    def get_sg(self):
        return self.sg


    def get_multibody_plant(self):
        return self.plant


    def get_controller_plant(self):
        return self.controller_plant


    def get_renderer(self):
        return self.renderer


    def get_arm_controller(self):
        return self.panda_controller


    def get_hand_controller(self):
        if hasattr(self, 'hand_controller'):
            return self.hand_controller
        else:
            return None


    def get_camera_info(self):
        return self.camera_info


    def get_panda(self):
        return self.panda


    def get_panda_c(self):
        return self.panda_c


    def get_hand(self):
        return self.hand


    def get_state_integrator(self):
        return self.state_integrator


    def get_state_interpolator(self):
        return self.state_interpolator
