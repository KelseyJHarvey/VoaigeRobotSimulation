'''!
@file voaige_mk1_setup.py

@brief Python Library for the VoAIge MK1 Robot Simulation
'''

''' Import Canonical Libraries '''
import math
import time
import subprocess
import numpy as np

''' Import PyBullet Libraries '''
import pybullet
import pybullet_data

# import pybullet_envs
# import pybullet_utils
# import pybullet_robots
# import pybullet_examples
# import pybullet_rendering

''' Physics Configuration '''


def physics_config():
    '''!
    TODO: Fill This In.
    '''

    physicsClientId = pybullet.connect(
        pybullet.GUI,
        # TODO: Automate Obtaining the Screen Resolution
        options="--width=1920 --height=2160"
    )

    if (physicsClientId == -1):
        print("\n\nPROBLEM\n\n")

    pybullet.setAdditionalSearchPath(
        pybullet_data.getDataPath()
    )

    # pybullet.setTimeStep(
    #     timeStep = timeStep,
    #     physicsClientId = physicsClientId
    #     )

    pybullet.setRealTimeSimulation(
        enableRealTimeSimulation=True,
        physicsClientId=physicsClientId
    )

    # Simulate Standard Earth Gravity
    pybullet.setGravity(
        gravX=0.0,
        gravY=0.0,
        gravZ=-9.81,
        physicsClientId=physicsClientId
    )


def environment_config():
    '''!
    TODO: Environment Configuration
    '''

    # Load a Simple Plane to Model a Rigid Floor
    planeId = pybullet.loadURDF("plane.urdf")

    # Load Tray
    numTrays = 2
    trayPosition = [[0.75, -0.295, 0], [0.75, 0.295, 0.0]]
    trayOrientation = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

    trayId = np.zeros(numTrays)
    for trayIdInd in range(numTrays):
        trayId[trayIdInd] = pybullet.loadURDF(
            fileName="tray/traybox.urdf",
            basePosition=trayPosition[trayIdInd],
            baseOrientation=pybullet.getQuaternionFromEuler(trayOrientation[trayIdInd]),
            useFixedBase=True
        )


def camera_setup(camWidth, camHeight):
    ''' Camera Setup '''

    viewMatrix = pybullet.computeViewMatrix(
        cameraEyePosition=[0.75, 0, 0.75],
        cameraTargetPosition=[0.75, 0, 0],
        cameraUpVector=[-1, 0, 0]
    )

    projectionMatrix = pybullet.computeProjectionMatrixFOV(
        fov=45.0,
        aspect=1.0,
        nearVal=0.01,
        farVal=2.0
    )

    return viewMatrix, projectionMatrix


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
        scale=[objScale] * 2 + [0.0],
        size=(numObjects, 3)
    )

    objIds = []
    for objIndex in range(numObjects):
        # Fetch an Object from the Repository at Random
        objFileNumStr = str("%03d" % npybullet.random.randint(0, 1000))

        # Load the Corresponding URDF File of the Retreived Object
        objIds.append(pybullet.loadURDF(
            fileName=objPath + "/" + objFileNumStr + "/" + objFileNumStr + ".urdf",
            basePosition=objStartPos[objIndex]
        ))

        print("Spawned object: " + objFileNumStr)

    return objIds
