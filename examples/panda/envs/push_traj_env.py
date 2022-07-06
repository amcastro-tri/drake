from abc import ABC
import numpy as np
import pickle

from pydrake.all import RotationMatrix, RollPitchYaw, PiecewisePolynomial, FindResourceOrThrow
from envs.push_env import PushEnv


class PushTrajEnv(PushEnv, ABC):
    """
    Dynamic pushing environment in Drake
    """
    def __init__(self, 
                 dt=0.002,
                 renders=False,
                 visualize_contact=False,
                 camera_params=None,
                 hand_type='plate',
                 diff_ik_filter_hz=-1,
                 contact_solver='sap',
                 panda_joint_damping=200,
                 ):
        super(PushTrajEnv, self).__init__(
            dt=dt,
            renders=renders,
            visualize_contact=visualize_contact,
            camera_params=camera_params,
            hand_type=hand_type,
            diff_ik_filter_hz=diff_ik_filter_hz,
            contact_solver=contact_solver,
            panda_joint_damping=panda_joint_damping,
        )


    # only function overriding
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

        # Reset EE
        R_E = RotationMatrix(RollPitchYaw(0, 0, 0))
        R_E_fixed = RotationMatrix(RollPitchYaw(0, -np.pi, np.pi/2))
        R_E = R_E.multiply(R_E_fixed)
        p_E = [0.40, 0, 0.15]
        qstar = self.ik(p_E, R_E, plant_context, controller_plant_context)
        self.set_arm(plant_context, qstar)

        # Initialize state interpolator/integrator
        self.reset_state(plant_context, context)

        # Reset simulation
        sim_context = self.simulator.get_mutable_context()
        sim_context.SetTime(0.)
        self.simulator.Initialize()
        self.simulator.AdvanceTo(0)

        ######################## Run ########################
        t = 0

        # Warm up - wait for bottle to settle - very stochastic
        t1 = 0.01
        while t < t1:
            context = self.simulator.get_mutable_context()
            plant_context = self.plant.GetMyContextFromRoot(context)
            t += self.dt
            # try:
            status = self.simulator.AdvanceTo(t)
            # except RuntimeError as e:
            #     print('Sim error!')
            #     return np.array([]), 0, True, {}

        # Load trajectory
        traj_file = FindResourceOrThrow("drake/examples/panda/data/push_traj.pkl")
        with open(traj_file, 'rb') as f:
            data = pickle.load(f)
        dq_d = data['dq']
        num_step = dq_d.shape[0]
        self.plant.SetVelocities(plant_context, self.panda, np.zeros((7)))

        # Push forward - cartesian velocity and diff IK
        self.reset_state(plant_context, context)
        for step in range(num_step):
            context = self.simulator.get_mutable_context()
            plant_context = self.plant.GetMyContextFromRoot(context)
            self.ik_result_port.FixValue(station_context, np.zeros((14)))
            self.V_WG_command_port.FixValue(station_context, np.zeros((6)))
            self.V_J_command_port.FixValue(station_context, dq_d[step])

            # Simulate forward
            t += self.dt
            # try:
            status = self.simulator.AdvanceTo(t)
            # except RuntimeError as e:
                # print('Sim error!')
                # return np.array([]), 0, True, {}

        # Rest
        context = self.simulator.get_mutable_context()
        plant_context = self.plant.GetMyContextFromRoot(context)
        t4 = 0.5 + t
        times = [0, 0.5]
        vel_init = self.get_ee_vel(plant_context)
        vel_1 = np.array([[0], [0], [0], 
                          [0], [0], [0]])
        traj_V_G = PiecewisePolynomial.FirstOrderHold(times, 
                                                np.hstack((vel_init, vel_1)))
        t_init = t
        while t < t4:
            context = self.simulator.get_mutable_context()
            plant_context = self.plant.GetMyContextFromRoot(context)
            self.ik_result_port.FixValue(station_context, np.zeros((14)))
            self.V_WG_command_port.FixValue(station_context, 
                                            traj_V_G.value(t - t_init))
            self.V_J_command_port.FixValue(station_context, np.zeros((7)))

            # Simulate forward
            t += self.dt
            # try:
            status = self.simulator.AdvanceTo(t)
            # except RuntimeError as e:
                # print('Sim error!')
                # return np.array([]), 0, True, {}

        # Get reward
        context = self.simulator.get_mutable_context()
        plant_context = self.plant.GetMyContextFromRoot(context)
        reward = 0

        # Always done: single step
        done = True
        return np.array([]), reward, done, {}
