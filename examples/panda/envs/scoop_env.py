from abc import ABC
import numpy as np

from pydrake.all import RotationMatrix, RollPitchYaw, PiecewisePolynomial, RigidTransform, FixedOffsetFrame, CollisionFilterDeclaration

from envs.panda_env import PandaEnv


class ScoopEnv(PandaEnv, ABC):
    def __init__(self, 
                dt=0.002,
                renders=False,
                camera_params=None,
                visualize_contact=False,
                diff_ik_filter_hz=500,
                contact_solver='sap',
                panda_joint_damping=200,
                ):
        """
        """
        super(ScoopEnv, self).__init__(
            dt=dt,
            renders=renders,
            camera_params=camera_params,
            visualize_contact=visualize_contact,
            diff_ik_filter_hz=diff_ik_filter_hz,
            contact_solver=contact_solver,
            panda_joint_damping=panda_joint_damping,
        )
        self.veggie_x = 0.68    # for scooping direction - assume veggies around this position in x
        self.finger_init_pos = 0.03
        self.spatula_init_z = 0.002

        # Fixed dynamics parameter
        self.veggie_hc_dissipation = 1.0
        self.veggie_hydro_resolution = 0.005

        # Set default task
        self.task = {}
        self.task['obj_num'] = 3
        self.task['sdf'] = '/data/drake/models/veggies/sample_ellipsoid.sdf'
        self.task['obj_mu'] = 0.4
        self.task['obj_modulus'] = 5
        self.task['obj_x'] = [0.70, 0.68, 0.68]
        self.task['obj_y'] = [0, 0.01, -0.01]
        self.task['obj_z'] = [0.011, 0.011, 0.011]


    def reset_task(self, task):
        return NotImplementedError


    def load_objects(self, ):
        # Load spatula
        spatula_model_index, spatula_body_index = \
            self.station.AddModelFromFile(
                        '/examples/panda/data/models/spatula/spatula_oxo.sdf',
                        RigidTransform([0.5,0.0,0.002]), 
                        name='spatula',
                        )  # if drop too high, spatula into table
        self.spatula_base = self.plant.get_body(spatula_body_index[0])  # only base

        # Load veggies - more like templates - save body and frame ids
        self.veggie_body_all = {}   # list of lists, in the order of links
        for ind in range(self.task['obj_num']):
            veggie_model_index, veggie_body_indice = \
                self.station.AddModelFromFile(
                    '/examples/panda/data/veggie_cylinder.sdf',
                    RigidTransform([self.task['obj_x'][ind],
                                    self.task['obj_y'][ind],
                                    self.task['obj_z'][ind]]),
                    name='veggie'+str(ind),
                )
            self.veggie_body_all[ind] = [self.plant.get_body(index) for index in veggie_body_indice]    # all links
        self.veggie_base_all = [b[0] for b in self.veggie_body_all.values()]

        # Add a generic frame for veggies - fixed to table
        self.veggie_fixed_frame = self.plant.AddFrame(
            FixedOffsetFrame("veggie_fixed_frame", 
                            self.plant.world_frame(), 
                            RigidTransform([self.veggie_x, 0, 0])
                            )
            )
        self.p_V = np.array([self.veggie_x, 0, 0])


    def reset(self, task=None):
        """
        Call parent to reset arm and gripper positions (build if first-time). Reset veggies and task. Do not initialize simulator.
        """
        task = super().reset(task)

        # Get new context
        context = self.simulator.get_mutable_context()
        plant_context = self.plant.GetMyContextFromRoot(context)
        sg_context = self.sg.GetMyMutableContextFromRoot(context)
        query_object = self.sg.get_query_output_port().Eval(sg_context)
        context_inspector = query_object.inspector()

        # Set veggie geometry - bodies is a list of bodies for one piece
        if not self.visualize_contact:
            sdf_config = task['sdf_config']
            sg_geometry_ids = context_inspector.GetAllGeometryIds()
            for _, bodies in self.veggie_body_all.items():
                for body_ind, body in enumerate(bodies):
        
                    # Get frame id  #? better way? - quit if not found in sg
                    old_geom_id = self.plant.GetCollisionGeometriesForBody(body)[0]
                    if old_geom_id not in sg_geometry_ids:
                        continue
                    frame_id = context_inspector.GetFrameId(old_geom_id)

                    # Replace body - this does not change the body mass/inertia
                    if 'link' + str(body_ind) not in sdf_config:
                        geom_type = None # flag for no link
                        x_dim, y_dim, z_dim, x, y, z, roll, pitch, yaw = 0, 0, 0, 0, 0, 0, 0, 0, 0
                    else:
                        body_str = str(body_ind)
                        geom_type = sdf_config['link'+body_str]
                        x_dim = sdf_config['x'+body_str]
                        y_dim = sdf_config['y'+body_str]
                        z_dim = sdf_config['z'+body_str]
                        if body_ind == 0:
                            x, y, z, roll, pitch, yaw = 0, 0, 0, 0, 0, 0
                        else:
                            x = sdf_config['x0'+body_str]
                            y = sdf_config['y0'+body_str]
                            z = sdf_config['z0'+body_str]
                            roll = sdf_config['roll0'+body_str]
                            pitch = sdf_config['pitch0'+body_str]
                            yaw = sdf_config['yaw0'+body_str]       
                    flag_replace = self.replace_body(context=sg_context,
                                    context_inspector=context_inspector,
                                    body=body,
                                    frame_id=frame_id,
                                    geom_type=geom_type,
                                    x_dim=x_dim*1.2,
                                    y_dim=y_dim*1.2,
                                    z_dim=z_dim*1.2,
                                    x=x,
                                    y=y,
                                    z=z,
                                    roll=roll,
                                    pitch=pitch,
                                    yaw=yaw,
                                    visual_name='link'+body_str+'_visual',
                                    collision_name='link'+body_str+'_collision',
                                    )

                    # Change body dynamics - if new geometry is added
                    if flag_replace:
                        self.set_obj_dynamics(context_inspector, 
                                sg_context,
                                body,
                                hc_dissipation=1.0,
                                sap_dissipation=0.1,
                                mu=self.task['obj_mu'],
                                hydro_modulus=self.task['obj_modulus'],
                                hydro_resolution=0.002,
                                compliance_type='compliant',
                                )

                        # Change mass - not using, specified in sdf
                        body.SetMass(plant_context, sdf_config['m'+body_str])

                # Exclude collision within body - # TODO: not working?
                body_geometry_set = self.plant.CollectRegisteredGeometries(bodies)
                self.sg.collision_filter_manager().Apply(
                    CollisionFilterDeclaration().ExcludeWithin(body_geometry_set)
                )

        # Set table properties
        self.set_obj_dynamics(context_inspector, 
                              sg_context, 
                              self.table_body,
                              hc_dissipation=1.0,
                              sap_dissipation=0.1,
                              mu=0.3,
                              hydro_modulus=5,
                              hydro_resolution=0.1, # does not matter
                              compliance_type='compliant')

        # Change global params - time step for both plant and controller_plant? seems impossible
        # point - plant.set_penetration_allowance (stiffness of normal penalty forces)
        # point/hydro - plant.set_stiction_tolerance (threshold for sliding)

        # Move spatula away from veggies
        self.set_body_pose(self.spatula_base, plant_context, 
                           p=[0.3, 0, 0.002],
                           rpy=[0, 0, 0])
        self.set_body_vel(self.spatula_base, plant_context)

        # Reset veggie
        for ind, veggie_base in enumerate(self.veggie_base_all):
            self.set_body_pose(veggie_base, plant_context, 
                                p=[task['obj_x'][ind],
                                   task['obj_y'][ind],
                                   task['obj_z'][ind]*1.2,
                                   ],
                                rpy=[0, 0, 0])
            self.set_body_vel(veggie_base, plant_context)
        station_context = self.station.GetMyContextFromRoot(context)
        return self._get_obs(station_context)


    def step(self, action):
        """
        Initialize simulator and then execute open-loop.
        Assume action in [yaw, pitch].
        """
        # Get new context
        context = self.simulator.get_mutable_context()
        station_context = self.station.GetMyContextFromRoot(context)
        plant_context = self.plant.GetMyContextFromRoot(context)
        controller_plant_context = self.controller_plant.GetMyContextFromRoot(context)
        sg_context = self.sg.GetMyMutableContextFromRoot(context)
        query_object = self.sg.get_query_output_port().Eval(sg_context)
        inspector = query_object.inspector()

        # Extract action
        s_yaw = action[0]
        s_init_pitch = action[1]
        # s_init_x = action[2]

        # Reset arm to initial location - first, veggie to tip
        R_V = RotationMatrix()
        R_VT = RotationMatrix(RollPitchYaw(0, 0, s_yaw))
        p_VT = [-0.07, 0., 0.]

        # then, tip to EE
        R_TE = RotationMatrix(RollPitchYaw(0, -np.pi, np.pi/2))
        p_TE = [-0.18, 0, 0.22]
        p_T = self.p_V + p_VT

        # finally, EE
        R_E = R_V.multiply(R_VT.multiply(R_TE))
        p_E = p_T + R_VT.multiply(p_TE)
        qstar = self.ik(p_E, R_E, plant_context, controller_plant_context)
        self.set_arm(plant_context, qstar)
        self.set_gripper(plant_context, self.finger_init_pos)

        # Reset spatula between gripper - in real, we can make a holder for spatula, thus ensures grasp always at the same pose on the spatula. Make sure the tip of spatula touches the table when the spatula is tilted
        # pad_half_height = 0.00015
        pad_half_height = 0.0005
        pad_half_length = 0.0575
        R_TI = RotationMatrix(RollPitchYaw(0, s_init_pitch, 0))
        p_TS = R_TI.multiply([-pad_half_length, 0, pad_half_height])
        p_S = p_T + R_VT.multiply(p_TS)
        R_S = R_VT.multiply(R_TI)
        self.set_body_pose(self.spatula_base, plant_context, 
                           p=p_S,
                           rm=R_S)
        self.set_body_vel(self.spatula_base, plant_context)

        # Initialize state interpolator/integrator
        self.reset_state(plant_context, context)

        # Reset simulation
        sim_context = self.simulator.get_mutable_context()
        sim_context.SetTime(0.)
        self.simulator.Initialize()
        self.simulator.AdvanceTo(0)

        ######################## Trajectory ########################
        q_all = []
        dq_all = []
        v_all = []
        v_d_all = []
        ddq_all = []
        dddq_all = []
        dq_prev = np.zeros((7))
        ddq_prev = np.zeros((7))
        hand_width_command = -0.2

        # Close gripper - keep arm fixed
        t = 0
        t1 = 0.2
        while t < t1:
            context = self.simulator.get_mutable_context()
            plant_context = self.plant.GetMyContextFromRoot(context)
            hand_force_measure = self.hand_force_measure_port.Eval(station_context)
            self.ik_result_port.FixValue(station_context, np.zeros((14)))
            self.V_WG_command_port.FixValue(station_context, np.zeros((6)))
            self.hand_position_command_port.FixValue(station_context, 
                                                    hand_width_command)

            # Keep spatula static
            self.set_body_pose(self.spatula_base, plant_context, 
                                p=p_S,
                                rm=R_S)
            self.set_body_vel(self.spatula_base, plant_context)

            # Simulate forward
            t += self.dt
            # try:
            status = self.simulator.AdvanceTo(t)
            # except RuntimeError as e:
            #     print(f'Sim error at time {t}!')
            #     return np.array([]), 0, True, {}

        # Push forward - cartesian velocity and diff IK
        # TODO: use transform to velocity
        context = self.simulator.get_mutable_context()
        plant_context = self.plant.GetMyContextFromRoot(context)
        t3 = 0.2 + t
        times = [0, 0.2]
        # vel_init = np.array([[0], [0], [0], 
                            # [0], [0], [0]])
        vel_init = self.get_ee_vel(plant_context)
        vel_0 = np.array([[0], [0], [0], 
                          [0.6*np.cos(s_yaw)], [0.6*np.sin(s_yaw)], [0.0]])
        traj_V_G = PiecewisePolynomial.FirstOrderHold(times, 
                                                np.hstack((vel_init, vel_0)))
        t_init = t
        while t < t3:
            context = self.simulator.get_mutable_context()
            plant_context = self.plant.GetMyContextFromRoot(context)
            self.ik_result_port.FixValue(station_context, np.zeros((14)))
            self.V_WG_command_port.FixValue(station_context, 
                                            traj_V_G.value(t - t_init))
            self.hand_position_command_port.FixValue(station_context, 
                                                    hand_width_command)

            # Simulate forward
            t += self.dt
            # print('time: ', t)
            # try:
            status = self.simulator.AdvanceTo(t)
            # except RuntimeError as e:
            #     print(f'Sim error at time {t}!')
            #     return np.array([]), 0, True, {}

        # Push forward - cartesian velocity and diff IK
        # TODO: use transform to velocity
        context = self.simulator.get_mutable_context()
        plant_context = self.plant.GetMyContextFromRoot(context)
        t3 = 0.1 + t
        times = [0, 0.1]
        vel_init = self.get_ee_vel(plant_context)
        vel_0 = np.array([[0.2*np.sin(s_yaw)], [-0.2*np.cos(s_yaw)], [0], 
                          [0.7*np.cos(s_yaw)], [0.7*np.sin(s_yaw)], [-0.02]])
        traj_V_G = PiecewisePolynomial.FirstOrderHold(times, 
                                                np.hstack((vel_init, vel_0)))
        t_init = t
        while t < t3:
            context = self.simulator.get_mutable_context()
            plant_context = self.plant.GetMyContextFromRoot(context)
            self.ik_result_port.FixValue(station_context, np.zeros((14)))
            self.V_WG_command_port.FixValue(station_context, 
                                            traj_V_G.value(t - t_init))
            self.hand_position_command_port.FixValue(station_context, 
                                                    hand_width_command)

            # Simulate forward
            t += self.dt
            # print('time: ', t)
            status = self.simulator.AdvanceTo(t)
            # except RuntimeError as e:
            #     print(f'Sim error at time {t}!')
            #     return np.array([]), 0, True, {}

        # Decelerate - cartesian velocity and diff IK
        context = self.simulator.get_mutable_context()
        plant_context = self.plant.GetMyContextFromRoot(context)
        t4 = 0.1 + t
        times = [0, 0.1]
        vel_init = self.get_ee_vel(plant_context)
        vel_1 = np.array([[0.6*np.sin(s_yaw)], [-0.6*np.cos(s_yaw)], [0], 
                          [0.2*np.cos(s_yaw)], [0.2*np.sin(s_yaw)], [-0.05]])
        traj_V_G = PiecewisePolynomial.FirstOrderHold(times, 
                                                np.hstack((vel_init, vel_1)))
        t_init = t
        while t < t4:
            context = self.simulator.get_mutable_context()
            plant_context = self.plant.GetMyContextFromRoot(context)
            self.ik_result_port.FixValue(station_context, np.zeros((14)))
            self.V_WG_command_port.FixValue(station_context, 
                                            traj_V_G.value(t - t_init))
            self.hand_position_command_port.FixValue(station_context, 
                                                    hand_width_command)

            # Simulate forward
            t += self.dt
            status = self.simulator.AdvanceTo(t)
            # except RuntimeError as e:
            #     print(f'Sim error at time {t}!')
                # return np.array([]), 0, True, {}
            # time.sleep(0.01)

        # Pitch back - cartesian velocity and diff IK
        context = self.simulator.get_mutable_context()
        plant_context = self.plant.GetMyContextFromRoot(context)
        t5 = 0.2 + t
        times = [0, 0.2]
        vel_init = self.get_ee_vel(plant_context)
        vel_2 = np.array([[0.8*np.sin(s_yaw)], [-0.8*np.cos(s_yaw)], [0], 
                          [0*np.cos(s_yaw)], [0*np.sin(s_yaw)], [-0.05]])
        traj_V_G = PiecewisePolynomial.FirstOrderHold(times, 
                                                np.hstack((vel_init, vel_2)))
        # self.reset_state(plant_context, context)
        t_init = t
        while t < t5:
            context = self.simulator.get_mutable_context()
            plant_context = self.plant.GetMyContextFromRoot(context)
            self.ik_result_port.FixValue(station_context, np.zeros((14)))
            self.V_WG_command_port.FixValue(station_context, 
                                            traj_V_G.value(t - t_init))
            self.hand_position_command_port.FixValue(station_context,       
                                                    hand_width_command)

            # Simulate forward
            t += self.dt
            # try:
            status = self.simulator.AdvanceTo(t)
            # except RuntimeError as e:
                # print(f'Sim error at time {t}!')
                # return np.array([]), 0, True, {}

        # Get reward
        context = self.simulator.get_mutable_context()
        plant_context = self.plant.GetMyContextFromRoot(context)
        reward = 0
        for veggie_body in self.veggie_base_all:
            veggie_pos = self.plant.GetFreeBodyPose(plant_context, veggie_body).translation()
            if veggie_pos[2] > 0.01:
                reward += 1/self.task['obj_num']

        # Always done: single step
        done = True
        return np.array([]), reward, done, {}


    def _get_obs(self, station_context):
        if not self.flag_use_camera:
            return None
        color_image = self.color_image_port.Eval(station_context).data[:,:,:3] # HxWx4
        color_image = np.transpose(color_image, [2,0,1])
        depth_image = np.squeeze(self.depth_image_port.Eval(station_context).data)

        # Normalize
        depth_image = ((self.camera_params['max_depth']-depth_image)/(self.camera_params['max_depth']-self.camera_params['min_depth']))*255
        depth_image = np.uint8(depth_image)[np.newaxis]
        image = np.vstack((color_image, depth_image))
        return image


    def visualize(self):
        raise NotImplementedError
