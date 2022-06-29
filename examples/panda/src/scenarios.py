"""
Helper utilities to build scenarios/experiments
"""
import numpy as np
import pydrake.all
from pydrake.all import (
        RigidTransform, RollPitchYaw, FindResourceOrThrow
        )


def AddPanda(plant, 
             q0=[0.0, 0.1, 0, -1.2, 0, 1.6, 0], 
             X_WB=RigidTransform(),
             ):
    """ Adds a franka panda arm without any hand to the mutlibody plant and welds it to the world frame

    plant: the multibody plant to add the panda to
    q0: the initial joint positions (optional)
    X_WB: the desired transformation between the world frame (W) and the base link of the panda (B)
    """
    urdf_file = FindResourceOrThrow('drake/examples/panda/data/models/franka_description/urdf/panda_arm.urdf')
    parser = pydrake.multibody.parsing.Parser(plant)
    panda_model_instance = parser.AddModelFromFile(urdf_file)

    # Weld panda to world
    plant.WeldFrames(plant.world_frame(), 
                     plant.GetFrameByName("panda_link0"), X_WB)

    # Set default joint
    index = 0
    for joint_index in plant.GetJointIndices(panda_model_instance):
        joint = plant.get_mutable_joint(joint_index)
        if isinstance(joint, pydrake.multibody.tree.RevoluteJoint):
            joint.set_default_angle(q0[index])
            index += 1
    return panda_model_instance


def AddHand(plant, 
            panda_model_instance=None, 
            roll=0, 
            welded=False, 
            type='wsg',
            ):
    """Adds a hand to the panda arm (panda_link8)

    plant: the multibody plant 
    panda_model_instance: the panda model instance to add the hand to
    roll: the rotation of the hand relative to panda_link8
    welded: if we want the version with welded fingers (for control)
    """
    parser = pydrake.multibody.parsing.Parser(plant)
    if type == 'wsg':
        file_path = FindResourceOrThrow('drake/examples/panda/data/models/wsg_50_description/sdf/schunk_wsg_50_box.sdf')
        gripper_base_frame_name = 'gripper_base'
        X_8G = RigidTransform(RollPitchYaw(np.pi / 2.0, 0, -roll), [0, 0, 0.03625+0.01])  # 0.03625: half dim of gripper base; 0.01: connector on real robot
    elif type == 'panda':
        file_path = FindResourceOrThrow('drake/examples/panda/data/models/franka_description/urdf/panda_hand.urdf')
        gripper_base_frame_name = 'panda_hand'
        X_8G = RigidTransform(RollPitchYaw(0, 0, roll), [0,0,0])
    elif type == 'plate':
        file_path = FindResourceOrThrow('drake/examples/panda/data/models/hand_plate/hand_plate.sdf')
        gripper_base_frame_name = 'plate_base'
        X_8G = RigidTransform(RollPitchYaw(0, 0, roll), [0,0,0.06])
    else:
        raise NotImplementedError
    gripper = parser.AddModelFromFile(file_path)
    
    # Weld gripper frame with arm
    if panda_model_instance is not None:
        plant.WeldFrames(
            plant.GetFrameByName("panda_link8", panda_model_instance), 
            plant.GetFrameByName(gripper_base_frame_name, gripper), X_8G)
    return gripper
