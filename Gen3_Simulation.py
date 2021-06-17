import pybullet
import time
import pybullet_data
import math
import numpy as np

def spawnObj():
    """
    Spawns a random object from a .URDF file.
    Returns ID of spawned object.
    """

    # general parameters
    scale = 0.15 # max magnitude of position randomness
    height = 0.3

    # randomize position
    objStartPos = [tray_position[0] + scale*2*(np.random.rand()-0.5),
                   tray_position[1] + scale*2*(np.random.rand()-0.5),
                   tray_position[2] + height]
    objStartOrientation = pybullet.getQuaternionFromEuler([0,0,0])

    # get random object from repository
    path = "./objects/random_urdfs/"
    objNum = np.random.randint(0, 1000)
    objIndex = str("%03d" % objNum)
    objId = pybullet.loadURDF(path + objIndex + "/" + objIndex + ".urdf", objStartPos, objStartOrientation)

    print("Spawned object: " + objIndex)
    return objId

ROBOT_URDF_PATH = "./voaige_description/robots/gen3_robotiq_2f_140.urdf"

''' Configure the Simulation '''
physicsClient = pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setRealTimeSimulation(enableRealTimeSimulation=1, physicsClientId = physicsClient)
pybullet.setGravity(0,0,-9.81)

fps = 240.
timeStep = 1./fps


''' Robot Arm Configuration '''
# JointStates = pybullet.calculateInverseKinematics(robotId, 7, [0.1,0.1,0.1])
armStartPos = [0,0.75,0]
armStartOrn = pybullet.getQuaternionFromEuler([0,0,0])
robotId = pybullet.loadURDF(ROBOT_URDF_PATH, useFixedBase=True)
numDofs = 7
numJoints = pybullet.getNumJoints(robotId)
maxForce = [500]*numDofs


''' Environment Configuration '''
# Load Solid Floor
planeId = pybullet.loadURDF("plane.urdf")

# Load Tray
tray_position = [0.75, 0, 0]
tray_orientation = pybullet.getQuaternionFromEuler([0,0,0])
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
    cameraEyePosition = [0.75, 0, 0.75],
    cameraTargetPosition=[0.75, 0, 0],
    cameraUpVector = [0, 1, 0])
    
projectionMatrix = pybullet.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.01,
    farVal=2.0)

# Spawn objects
numObjects = 5
for _ in range(numObjects):
    objId = spawnObj()

''' Run the Simulation '''      
while True:

    width, height, rgbImg, depthImg, segImg = pybullet.getCameraImage(
        width=camWidth,
        height=camHeight,
        viewMatrix=viewMatrix,
        projectionMatrix=projectionMatrix)
    
    pybullet.stepSimulation()
    time.sleep(timeStep)

pybullet.disconnect()

