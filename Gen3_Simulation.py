#!/usr/bin/env/python3

'''!
@file Gen3_Simulation.py
'''

# import math
import numpy as np
from operator import add

# import time

''' PyBullet Libraries'''
import pybullet
import pybullet_data

''' Import VoAIge Proprietary Libraries '''

import voaige_mk1_sim



'''++++++++++++++++++++++++++'''
''' Simulation Configuration '''
'''++++++++++++++++++++++++++'''

# Configure the Physics Engine (TODO: Allow to Pass Arguments.)
voaige_mk1_sim.physics_config()

# Configure the Robot's Working Environment
voaige_mk1_sim.environment_config()

# Configure the Overhead Camera
overHeadCamWidth = 224
overHeadCamHeight = overHeadCamWidth
# viewMatrix, projectionMatrix = voaige_mk1_sim.camera_setup(camWidth, camHeight)

overHeadViewMatrix = pybullet.computeViewMatrix(
    cameraEyePosition=[0.75, 0, 0.75],
    cameraTargetPosition=[0.75, 0, 0],
    cameraUpVector=[-1, 0, 0]
)

overHeadProjectionMatrix = pybullet.computeProjectionMatrixFOV(
    fov=45.0, # Degrees
    aspect=1.0,
    nearVal=0.01,
    farVal=2.0
)

# Configure the Bracelet Camera
braceletCamWidth = 224
braceletCamHeight = braceletCamWidth

braceletViewMatrix = pybullet.computeViewMatrix(
    cameraEyePosition=[0.0, 0.0, 1.2],
    cameraTargetPosition=[1, 0, 0.5],
    cameraUpVector=[0, 0, 1]
)

braceletProjectionMatrix = pybullet.computeProjectionMatrixFOV(
    fov=45.0, # Degrees
    aspect=1.0,
    nearVal=0.01,
    farVal=2.0
)

# Spawn Objects Into Robot's Right Tray
# TODO: Implement This With Tom's New Object URDFs.

''' MK1 Robot Configuration '''
# TODO: Consolidate Into Module File.
# Configure the Robot Itself
MK1_URDF_PATH = "./voaige_description/arms/kinova_gen3/urdf/GEN3-6DOF_VISION_URDF_ARM_V01.urdf"
MK1_DEFAULT_START_POS = [0.0, 0.0, 0.0]
MK1_DEFAULT_START_ORIENTATION = pybullet.getQuaternionFromEuler(np.deg2rad([0.0, 0.0, 0.0]))
MK1_BRACELET_LINK_INDEX = 5
MK1_GRIPPER_BASE_LINK_INDEX = 6
MK1_NUM_DOFS = 6
MK1_MAX_JOINT_FORCE = 5E2
# TODO: Implement PyBullet Functionality to Compute This Dynamically.
MK1_END_EFFECTOR_ORIENTATION_OFFSET = [90.0, 0.0, -90.0]

''' Robot Arm Configuration '''

# Obtain the MK1 Robot ID
robotId = pybullet.loadURDF(
    fileName=MK1_URDF_PATH,
    basePosition=MK1_DEFAULT_START_POS,
    baseOrientation=MK1_DEFAULT_START_ORIENTATION,
    useFixedBase=True
)



''' Run the Simulation '''
while True:

    # Collect the Over-Head Camera Frame Image
    # overheadWidth, overheadHeight, ovrhdRGBImg, ovrhdDepthImg, ovrhdSegImg = \
    # pybullet.getCameraImage(
    #     width=overHeadCamWidth,
    #     height=overHeadCamHeight,
    #     viewMatrix=overHeadViewMatrix,
    #     projectionMatrix=overHeadProjectionMatrix
    # )

    # Collect the Robot Bracelet Camera Frame Image

    # Get the End-Effector Link State
    endEffectorLinkState = pybullet.getLinkState(
        bodyUniqueId=robotId,
        linkIndex=MK1_BRACELET_LINK_INDEX, # Check This Number
        computeForwardKinematics=True
    )

    print("\n\nendEffectorLinkState: {0}\n\n".format(endEffectorLinkState[0]))

    # braceletViewMatrix = pybullet.computeViewMatrixFromYawPitchRoll(
    #     cameraTargetPosition=endEffectorLinkState[0],
    #     distance=0.2,
    #     yaw=np.deg2rad(0.0), # Radians
    #     pitch=np.deg2rad(0.0), # Radians
    #     roll=np.deg2rad(0.0), # Radians
    #     upAxisIndex=2, # Z Up Axis
    # )

    braceletViewMatrix = pybullet.computeViewMatrix(
        cameraEyePosition=np.add(endEffectorLinkState[0], [0.0, 0.0, 0.2]),
        cameraTargetPosition=[0.75, 0.0, 0.0],
        cameraUpVector=[0.0, 0.0, 1.0]
    )

    # braceletProjectionMatrix = pybullet.computeProjectionMatrixFOV(
    #     fov=45.0,  # Degrees
    #     aspect=1.0,
    #     nearVal=0.01,
    #     farVal=2.0
    # )

    # brcltWidth, brcltHeight, brcltRGBImg, brcltDepthImg, brcltSegImg = \
    pybullet.getCameraImage(
         width=braceletCamWidth,
         height=braceletCamHeight,
         viewMatrix=braceletViewMatrix,
         projectionMatrix=braceletProjectionMatrix
    )

    # TODO: Dipan, Please Merge End-Effector Code Here.
    # string_series = input("\n\nPlease input a target End Effector Location.\n")
    # targetEndEffectorLocation = [float(item) for item in string_series.split()]
    targetEndEffectorPosition = [0.7, 0.0, 0.4]
    targetEndEffectorOrientation = pybullet.getQuaternionFromEuler(
        np.deg2rad(list(map(add, MK1_END_EFFECTOR_ORIENTATION_OFFSET, [0.0, 0.0, 0.0])))
    )

    # Obtain Necessary Joint Values for Target End-Effector Position
    targetJointValues = pybullet.calculateInverseKinematics(
        bodyUniqueId=robotId,
        endEffectorLinkIndex=MK1_BRACELET_LINK_INDEX,
        targetPosition=targetEndEffectorPosition,
        targetOrientation=targetEndEffectorOrientation
    )

    # Set the Joint Motor Positions for Desired End-Effector Position
    pybullet.setJointMotorControlArray(
        bodyUniqueId=robotId,
        jointIndices=range(MK1_BRACELET_LINK_INDEX + 1),
        controlMode=pybullet.POSITION_CONTROL,
        targetPositions=targetJointValues[:MK1_BRACELET_LINK_INDEX + 1],
        forces=[MK1_MAX_JOINT_FORCE] * MK1_NUM_DOFS  # Newtons
    )

    pybullet.stepSimulation()

pybullet.disconnect()
