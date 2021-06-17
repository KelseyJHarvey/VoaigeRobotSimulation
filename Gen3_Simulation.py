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
        scale=[objScale, objScale, 0.0],
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


def accurateCalculateInverseKinematics(robotId, targetPos, threshold, maxIter):
    closeEnough = False
    iter = 0
    dist2 = 1e30
    while (not closeEnough and iter < maxIter):

        jointPoses = pybullet.calculateInverseKinematics(
            robotId,
            7,
            targetPosition=targetPos)

        for i in range(6):
            pybullet.resetJointState(robotId, i, jointPoses[i])

        ls = pybullet.getLinkState(robotId, 7)
        newPos = ls[4]
        diff = [targetPos[0] - newPos[0], targetPos[1] -
                newPos[1], targetPos[2] - newPos[2]]
        dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
        closeEnough = (dist2 < threshold)
        iter += 1

    # print ("Num iter: "+str(iter) + "threshold: "+str(dist2))
    return jointPoses


ROBOT_URDF_PATH = "./voaige_description/robots/gen3_robotiq_2f_140.urdf"

''' Configure the Simulation '''

fps = 240.0
timeStep = 1.0/fps

physicsClient = pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setRealTimeSimulation(
    enableRealTimeSimulation=1, physicsClientId=physicsClient)
pybullet.setGravity(0, 0, -9.81)
pybullet.setTimeStep(timeStep=timeStep, physicsClientId=physicsClient)

''' Robot Arm Configuration '''
# JointStates = pybullet.calculateInverseKinematics(robotId, 7, [0.1,0.1,0.1])

numDofs = 6
robotStartPos = [0, 0, 0]
robotStartOrn = pybullet.getQuaternionFromEuler(np.deg2rad([0, 0, 0]))
robotId = pybullet.loadURDF(
    ROBOT_URDF_PATH, 
    basePosition = robotStartPos,
    baseOrientation = robotStartOrn, 
    useFixedBase=True)

numJoints = pybullet.getNumJoints(robotId)
maxForce = [500]*numDofs


''' Environment Configuration '''
# Load Solid Floor
planeId = pybullet.loadURDF("plane.urdf")

# Load Tray
tray_position = [0.75, 0, 0]
tray_orientation = pybullet.getQuaternionFromEuler([0, 0, 0])
tray_id = pybullet.loadURDF(
    "tray/traybox.urdf",
    tray_position,
    tray_orientation,
    useFixedBase=True)


''' Camera Setup '''
cameraFPS = 12.
camWidth = 224
camHeight = camWidth

viewMatrix = pybullet.computeViewMatrix(
    cameraEyePosition=[0.75, 0, 0.75],
    cameraTargetPosition=[0.75, 0, 0],
    cameraUpVector=[-1, 0, 0])

projectionMatrix = pybullet.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.01,
    farVal=2.0)

# Spawn objects
numObjects = 15

'''
objId = spawn_random_objects(
    numObjects,
    objSpawnLocation=[1, 1, 1],
    objPath="./objects/random_urdfs")
'''


maxForce = [1e5]*6
pybullet.setJointMotorControlArray(
    bodyUniqueId=robotId,
    jointIndices=list(range(numDofs)),
    controlMode=pybullet.POSITION_CONTROL,
    targetPositions=np.deg2rad([30.0, 45.0, 15.0, 20.0, 10.0, 5.0]),
    forces=maxForce)
    
    
''' Run the Simulation '''
while True:

    width, height, rgbImg, depthImg, segImg = pybullet.getCameraImage(
        width=camWidth,
        height=camHeight,
        viewMatrix=viewMatrix,
        projectionMatrix=projectionMatrix)
    
    linkWorldPosition = pybullet.getLinkState(
    bodyUniqueId=robotId,
    linkIndex=7,
    computeForwardKinematics = 1)

    targetEndEffectorLocation = linkWorldPosition[0]

    # The Theoretical End Effector Position
    print("\n\n\n")
    print(targetEndEffectorLocation)
    print("\n\n\n")

    thetaInvKin = pybullet.calculateInverseKinematics(
        bodyUniqueId=robotId,
        endEffectorLinkIndex=7,
        targetPosition=targetEndEffectorLocation)

    print(np.rad2deg(thetaInvKin))
    print("\n\n\n")    

    pybullet.stepSimulation()

pybullet.disconnect()
