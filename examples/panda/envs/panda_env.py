# Modified from RobotLocomotion Group

from abc import ABC, abstractmethod
from pathlib import Path
home_path = str(Path.home())
from copy import deepcopy
import numpy as np

from pydrake.all import DiagramBuilder, RigidTransform, RotationMatrix, RollPitchYaw, ContactModel, Simulator, SpatialVelocity, InverseKinematics, Solve, Ellipsoid, Cylinder, Box, GeometryInstance, IllustrationProperties, PerceptionProperties, Rgba, MakePhongIllustrationProperties, Role, RenderLabel
from src.panda_station import PandaStation
from src.utils_drake import set_collision_properties


class PandaEnv(ABC):
    def __init__(self,
                 dt=0.002,
                 renders=False,
                 camera_params=None,
                 visualize_contact=False,
                 hand_type='wsg',
                 diff_ik_filter_hz=-1,
                 contact_solver='sap',
                 ):
        self.dt = dt
        self.renders = renders
        self.visualize_contact = visualize_contact
        self.camera_params = camera_params
        if self.camera_params is not None:
            self.flag_use_camera = True
        else:
            self.flag_use_camera = False
        self.hand_type = hand_type
        if self.hand_type != 'plate':
            self.flag_actuate_hand = True
        else:
            self.flag_actuate_hand = False
        self.diff_ik_filter_hz = diff_ik_filter_hz
        self.contact_solver = contact_solver

        # Flag for whether setup is built
        self.flag_setup = False


    @property
    def q0(self):
        """
        Default joint angles. Not really used right now.
        """
        return np.array([0., -0.255, 0, -2.437, 0, 2.181, 0.785])


    def _setup(self, task=None):
        """
        Set up Panda environment in Drake. Need to load all objects here due to design of Drake.
        """

        # Load panda and Drake basics
        builder = DiagramBuilder()
        system = PandaStation(dt=self.dt, 
                              visualize_contact=self.visualize_contact,
                              diff_ik_filter_hz=self.diff_ik_filter_hz,
                              contact_solver=self.contact_solver)
        self.station = builder.AddSystem(system)
        self.station.set_panda(self.hand_type)
        # self.station.set_camera(self.camera_params)
        self.plant = self.station.get_multibody_plant()
        self.sg = self.station.get_sg()
        if self.renders:
            self.visualizer = self.station.get_visualizer(use_meshcat=False)
        else:
            self.visualizer = None

        # Set contact model - kPoint, kHydroelastic, kHydroelasticWithFallback
        self.plant.set_contact_model(ContactModel.kHydroelasticWithFallback)

        # Load table
        table_model_index, table_body_index = self.station.set_table()
        self.table_body = self.plant.get_body(table_body_index)

        # Load objects - child - more like a template to be modified
        self.load_objects()

        # Finalize, get controllers
        self.plant.Finalize()
        self.station.set_arm_controller()
        if self.flag_actuate_hand:
            self.station.set_hand_controller()
        self.station.Finalize()

        # Get controllers
        self.panda = self.station.get_panda()
        self.hand = self.station.get_hand()
        self.state_integrator = self.station.get_state_integrator()
        self.state_interpolator = self.station.get_state_interpolator()
        self.panda_controller = self.station.get_arm_controller()
        self.hand_controller = self.station.get_hand_controller()

        # Get ports
        if self.flag_use_camera:
            self.color_image_port = self.station.GetOutputPort("color_image")
            self.depth_image_port = self.station.GetOutputPort("depth_image")
        self.contact_results_output_port = self.plant.  get_contact_results_output_port()
        self.reaction_forces_output_port = self.plant.get_reaction_forces_output_port()
        self.table_force_output_port = self.plant.get_generalized_contact_forces_output_port(table_model_index)
        self.panda_force_output_port = self.plant.get_generalized_contact_forces_output_port(self.panda)
        self.panda_desired_state_port = self.station.GetOutputPort("panda_desired_state")
        self.ik_result_port = self.station.GetInputPort("ik_result")
        self.V_WG_command_port = self.station.GetInputPort("V_WG_command")
        self.V_J_command_port = self.station.GetInputPort("V_J_command")
        if self.flag_actuate_hand:
            self.hand_force_measure_port = self.station.GetOutputPort("hand_force_measured")
            self.hand_position_command_port = self.station.GetInputPort("hand_position_command")

        # Set up simulator
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()  # need to change property before this - but now we have hacks...
        # context.DisableCaching()
        diagram.Publish(context)
        self.simulator = Simulator(diagram, context)

        # Use separate plant for arm controller
        self.panda_c = self.station.get_panda_c()
        self.controller_plant = self.station.get_controller_plant()


    def reset(self, task=None):
        """
        Set up simulator if first time. Reset the arm and gripper.
        """
        if task is None:
            task = self.task

        if not self.flag_setup:
            self._setup(task)
            self.flag_setup = True

        # Move arm away
        context = self.simulator.get_mutable_context()
        plant_context = self.plant.GetMyContextFromRoot(context)
        self.set_arm(plant_context, self.q0)
        if self.flag_actuate_hand:
            self.set_gripper(plant_context, self.finger_init_pos)   # from child
        return task


    @abstractmethod
    def reset_task(self, task=None):
        """
        Reset the task for the environment.
        """
        raise NotImplementedError


    @abstractmethod
    def step(self):
        """
        Gym style step function. Apply action, move robot, get observation,
        calculate reward, check if done.
        """
        raise NotImplementedError


    @abstractmethod
    def visualize(self):
        """
        Visualize trajectories and value functions.
        """
        raise NotImplementedError


    def close(self):
        pass


    def seed(self, seed=None):
        """
        Set the seed of the environment. Should be called after action_sapce is
        defined.    #* No torch rn # TODO: drake seed?

        Args:
            seed (int, optional): random seed value. Defaults to 0.
        """
        self.rng = np.random.default_rng(seed=seed)
        # torch.manual_seed(self.seed_val)
        # torch.cuda.manual_seed(self.seed_val)
        # torch.cuda.manual_seed_all(
        #     self.seed_val)  # if you are using multi-GPU.
        # torch.backends.cudnn.benchmark = False
        # torch.backends.cudnn.deterministic = True
        # self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]


    def ik(self, 
           p_e, 
           R_e, 
           plant_context, 
           controller_plant_context, 
           frame='arm',
           ):
        ik = InverseKinematics(self.controller_plant,
                               controller_plant_context,
                               with_joint_limits=False,  # much faster
                               )
        q_cur = self.plant.GetPositions(plant_context)[:7]  # use normal plant to get arm joint angles, since the control plant does not update them
        if frame == 'arm':
            ee_frame = self.controller_plant.GetFrameByName(
                "panda_link8", self.panda_c)
        elif frame == 'tip':
            ee_frame = self.controller_plant.GetFrameByName(
                "fingertip_frame", self.panda_c)
        else:
            raise 'Unknown frame error!'
        ik.AddPositionConstraint(
            ee_frame, [0, 0, 0], self.plant.world_frame(),
            p_e, p_e)
        ik.AddOrientationConstraint(
            ee_frame, RotationMatrix(), self.plant.world_frame(),
            R_e, 0.0)
        prog = ik.get_mutable_prog()
        # prog.SetSolverOption(IpoptSolver().solver_id(), 
        #                      "max_iter", 500)    # speed up
        q = ik.q()
        prog.AddQuadraticErrorCost(np.identity(len(q)), q_cur, q)
        prog.SetInitialGuess(q, q_cur)
        result = Solve(ik.prog())   # sim returns error if this line is called
        # if not result.is_success():
            # print("IK failed")
        # print('solver is: ', result.get_solver_id().name())
        qstar_raw = result.GetSolution(q)
        qstar = qstar_raw[:7]
        # print('IK solution: ', qstar)
        return qstar


    ########################## Helper ##########################

    def get_ee_pose(self, plant_context):
        return self.plant.EvalBodyPoseInWorld(
            plant_context, 
            self.plant.GetBodyByName("panda_link8")
            )


    def get_ee_vel(self, plant_context):
        return self.plant.EvalBodySpatialVelocityInWorld(
            plant_context, 
            self.plant.GetBodyByName("panda_link8")
            ).get_coeffs().reshape(6,1)


    def get_joint_angles(self, plant_context):
        return self.plant.GetPositions(plant_context)[:7]


    def get_joint_velocities(self, plant_context):
        return self.plant.GetVelocities(plant_context)[:7]


    def set_arm(self, plant_context, q, qdot=[0]*7):
        self.plant.SetPositions(plant_context, self.panda, q)
        self.plant.SetVelocities(plant_context, self.panda, qdot)


    def set_gripper(self, plant_context, g, gdot=[0]*2):
        self.plant.SetPositions(plant_context, self.hand, [-g, g])
        self.plant.SetVelocities(plant_context, self.hand, gdot)


    def set_body_pose(self, body, plant_context, p, rpy=[0,0,0], rm=None):
        if rm is None:
            rm = RotationMatrix(RollPitchYaw(rpy[0], rpy[1], rpy[2]))
        self.plant.SetFreeBodyPose(plant_context, body, RigidTransform(rm, p))


    def set_body_vel(self, body, plant_context, w=[0,0,0], v=[0,0,0]):
        self.plant.SetFreeBodySpatialVelocity(body,
                                            SpatialVelocity(w=w, v=v),
                                            plant_context)


    def reset_state(self, plant_context, context):
        state_integrator_context = self.state_integrator.GetMyMutableContextFromRoot(context)
        state_interpolator_context = self.state_interpolator.GetMyMutableContextFromRoot(context)
        q = self.plant.GetPositions(plant_context)[:7]
        self.state_interpolator.set_initial_position(state_interpolator_context, q)  # avoid large derivatives at the beginning
        self.state_integrator.set_integral_value(
            state_integrator_context, q)


    def set_obj_dynamics(self, 
                         context_inspector, 
                         context, 
                         body,
                         mu=None,
                         modulus=None,
                         dissipation=None,
                         resolution=None,
                         compliance_type='compliant'):
        '''
        Change friction - only dynamic friction is used in point contact (static ignored). Assign role with context
        '''
        if mu is None:
            mu = self.task['obj_mu']
        if modulus is None:
            modulus = self.task['obj_modulus']
        if dissipation is None:
            dissipation = self.veggie_dissipation   # TODO
        if resolution is None:
            resolution = self.veggie_resolution
        geometry_id = self.plant.GetCollisionGeometriesForBody(body)[0]
        set_collision_properties(self.sg,
                                context,
                                context_inspector,
                                self.plant, 
                                geometry_id, 
                                static_friction=mu, 
                                dynamic_friction=mu,
                                dissipation=dissipation,
                                resolution=resolution,
                                modulus=10**modulus,
                                compliance_type=compliance_type)


    def replace_body(self, 
                     context, 
                     context_inspector, 
                     body, 
                     frame_id, 
                     geom_type, 
                     x_dim, 
                     y_dim, 
                     z_dim,
                     x=0,
                     y=0,
                     z=0,
                     roll=0,
                     pitch=0,
                     yaw=0,
                     color=[0.659, 0.839, 0.514, 1],
                     render_label=100,
                     visual_name='visual',
                     collision_name='collision',
                    ):
        """Replace body at frame.
        """
        # Remove old geometries if exists - save a copy of old proximity properties - probably not needed since changing obj dynamics later # todo
        source_id = self.plant.get_source_id()
        old_visual_id = context_inspector.GetGeometries(frame_id, 
                                                        Role.kPerception)
        assert len(old_visual_id) <= 1
        illu_properties = None
        if len(old_visual_id) == 1:
            old_visual_id = old_visual_id[0]
            perc_properties = deepcopy(context_inspector.GetPerceptionProperties(old_visual_id))
            illu_properties = deepcopy(context_inspector.GetIllustrationProperties(old_visual_id))
            # print(context_inspector.GetIllustrationProperties(old_visual_id))
            # print(context_inspector.GetPerceptionProperties(old_visual_id))
            self.sg.RemoveGeometry(context=context, 
                                    source_id=source_id,
                                    geometry_id=old_visual_id)
        old_prox_id = context_inspector.GetGeometries(frame_id, 
                                                      Role.kProximity)
        assert len(old_prox_id) <= 1
        prox_properties = None
        if len(old_prox_id) == 1:
            old_prox_id = old_prox_id[0]
            prox_properties = deepcopy(context_inspector.GetProximityProperties(old_prox_id))   # no illustration or perception properties
            self.sg.RemoveGeometry(context=context, 
                                    source_id=source_id,
                                    geometry_id=old_prox_id)

        # Determine geometry/shape type - do not keep creating if no link speficied
        if geom_type == 'ellipsoid':
            visual_shape = Ellipsoid(x_dim, y_dim, z_dim)
        elif geom_type == 'cylinder':
            visual_shape = Cylinder(x_dim, z_dim)
        elif geom_type == 'box':
            visual_shape = Box(x_dim, y_dim, z_dim)
        else:
            return 0
        prox_shape = deepcopy(visual_shape)
        transform = RigidTransform(RollPitchYaw(roll, pitch, yaw),
                                   [x, y, z])

        # Create new visual geometry with illustration and perception properties
        new_visual = GeometryInstance(X_PG=transform, 
                                    shape=visual_shape, 
                                    name=visual_name)
        if illu_properties is None:
            illu_properties = MakePhongIllustrationProperties(color)
        new_visual.set_illustration_properties(illu_properties)
        if perc_properties is None:
            perc_properties = PerceptionProperties()
            perc_properties.AddProperty("phong", "diffuse", 
                                Rgba(color[0], color[1], color[2], color[3]))
            perc_properties.AddProperty("label", "id", RenderLabel(render_label))
        new_visual.set_perception_properties(perc_properties)

        # Create new proximity geometry
        new_prox = GeometryInstance(X_PG=transform, 
                                    shape=prox_shape, 
                                    name=collision_name)
        new_prox.set_proximity_properties(prox_properties)

        # Register new geometry on SG
        new_visual_id = self.sg.RegisterGeometry(context=context, 
                                                source_id=source_id,
                                                frame_id=frame_id, 
                                                geometry=new_visual)
        new_prox_id = self.sg.RegisterGeometry(context=context, 
                                                source_id=source_id,
                                                frame_id=frame_id, 
                                                geometry=new_prox)

        # Swap geometry on MbP
        self.plant.SwapCollisionGeometries(body, 
                                           old_prox_id, 
                                           new_prox_id)
        return 1

