#!/usr/bin/env/python3

import math
import numpy as np
import time

''' PyBullet Libraries'''
import pybullet
import pybullet_data

'''!
@file Gen3_Simulation.py
'''


def spawn_random_objects(numObjects, objSpawnLocation, objPath, objScale=0.1):
    '''!
    Spawns a random object from a .URDF file.
    @name spawn_random_objects()

    @brief Spawns a specified dnumber of random objects at a specified
           location with added Gaussian Noise to prevent object spawn
           collisions.

    @param numObjects The number of objects to be spawned.
    @param objSpawnLocation A 3-Vector (list type) specifying the spawn
           location.
    @param objPath The path string to the object files.
    @param objScale The value by which to scale the objects volumetrically.

    @return objIds Integer IDs of Spawned Objects.
    '''

    # Add Gaussian Noise to Spawn Location to Avoid Object Collisions
    objStartPos = npybullet.random.normal(
        loc=objSpawnLocation,
        scale=[objScale]*2 + [0.0],
        size=(numObjects, 3)
    )

    objIds = []
    for objIndex in range(numObjects):

        # Fetch an Object from the Repository at Random
        objFileNumStr = str("%03d" % npybullet.random.randint(0, 1000))

        # Load the Corresponding URDF File of the Retreived Object
        objIds.append(pybullet.loadURDF(
            fileName=objPath+"/"+objFileNumStr+"/"+objFileNumStr+".urdf",
            basePosition=objStartPos[objIndex]
        ))

        print("Spawned object: " + objFileNumStr)

    return objIds

ROBOT_URDF_PATH = "./voaige_description/robots/voaige_mk1/urdf/mk1.urdf"

''' Simulation Configuration '''

fps = 240.0
timeStep = 1.0/fps

physicsClient = pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setRealTimeSimulation(
    enableRealTimeSimulation = 1,
    physicsClientId = physicsClient
    )
pybullet.setGravity(0, 0, -9.81)

# pybullet.setTimeStep(
#     timeStep = timeStep,
#     physicsClientId = physicsClient
#     )

''' Robot Arm Confi# pybullet.setTimeStep(
#     timeStep = timeStep,
#     physicsClientId = physicsClient
#     )guration '''
# JointStates = pybullet.calculateInverseKinematics(robotId, 7, [0.1,0.1,0.1])

numDofs = 6
robotStartPos = [0, 0, 0]
robotStartOrn = pybullet.getQuaternionFromEuler(np.deg2rad([0, 0, 0]))
robotId = pybullet.loadURDF(
    fileName=ROBOT_URDF_PATH,
    basePosition=robotStartPos,
    baseOrientation=robotStartOrn,
    useFixedBase=True)

numJoints = pybullet.getNumJoints(robotId)
maxForce = [500]*numDofs


''' Environment Configuration '''
# Load Solid Floor
planeId = pybullet.loadURDF("plane.urdf")

# Load Tray
numTrays = 2
trayPosition = [[0.75, -0.295, 0], [0.75, 0.295, 0.0]]
trayOrientation = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

trayId = np.zeros(numTrays)
for trayIdInd in range(numTrays):
    trayId[trayIdInd] = pybullet.loadURDF(
        fileName = "tray/traybox.urdf",
        basePosition = trayPosition[trayIdInd],
        baseOrientation = pybullet.getQuaternionFromEuler(trayOrientation[trayIdInd]),
        useFixedBase = True
        )

''' Camera Setup '''
cameraFPS = 12.
camWidth = 224.
camHeight = camWidth

viewMatrix = pybullet.computeViewMatrix(
    cameraEyePosition = [0.75, 0, 0.75],
    cameraTargetPosition = [0.75, 0, 0],
    cameraUpVector = [-1, 0, 0])

projectionMatrix = pybullet.computeProjectionMatrixFOV(
    fov = 45.0,
    aspect = 1.0,
    nearVal = 0.01,
    farVal = 2.0)

maxForce = [1e8]*7
# pybullet.setJointMotorControlArray(
#     bodyUniqueId = robotId,
#     jointIndices = [0, 1, 2, 3, 4, 5, 6],
#     controlMode = pybullet.POSITION_CONTROL,
#     targetPositions = np.deg2rad([0.0, 45.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
#     forces=maxForce)

targetEndEffectorLocation = [0.0, 0.0, 0.8]

targetThetas = np.deg2rad([-45.0, 45.0, -45.0, 45.0, 45.0, 45.0, 0.0])
pybullet.setJointMotorControlArray(
        bodyUniqueId=robotId,
        jointIndices=range(1,8),
        controlMode=pybullet.POSITION_CONTROL,
        targetPositions=targetThetas,
        forces=maxForce
        )

pybullet.stepSimulation()

linkState = pybullet.getLinkState(
    bodyUniqueId=robotId,
    linkIndex=7,
    computeForwardKinematics=True
    )

# print("\n\nlinkState: {0}\n\n".format(linkState[0]))

# targetEndEffectorLocation = [0.450936795079764, 0.5710833124142867, 0.658157091080004]


''' Run the Simulation '''
while True:

    string_series = input("\n\nPlease input a target End Effector Location.\n")
    targetEndEffectorLocation = [float(item) for item in string_series.split()]

    targetJointValues = pybullet.calculateInverseKinematics(
            bodyUniqueId=robotId,
            endEffectorLinkIndex=5,
            targetPosition=targetEndEffectorLocation
        )

    np.set_printoptions(suppress=True)
    print("targetJointValues: {0}\n".format(np.rad2deg(targetJointValues)))

    pybullet.setJointMotorControlArray(
        bodyUniqueId=robotId,
        jointIndices=range(1,8),
        controlMode=pybullet.POSITION_CONTROL,
        targetPositions=targetJointValues[1:8],
        forces=maxForce
        )

    # Collect the Camera Frame Image
    # width, height, rgbImg, depthImg, segImg = pybullet.getCameraImage(
    #     width=camWidth,
    #     height=camHeight,
    #     viewMatrix=viewMatrix,
    #     projectionMatrix=projectionMatrix)

    pybullet.stepSimulation()

pybullet.disconnect()
