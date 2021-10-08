# Import modules
import sys  
import time 

# Interface between python and Coppelia
sys.path.append('/home/rafael-barbosa/ptr_project/PyBinding')
import sim  

# Just in case, close all opened connections
sim.simxFinish(-1)

# Connect to CoppeliaSim
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

# End program if connection failed
if clientID != -1:
    print('Connected to remote API server')
else:
    print("Failed to connect to remote API server")
    print("Leaving program")
    sys.exit()

# Show message in coppelia
sim.simxAddStatusbarMessage(
    clientID, "Python running!", sim.simx_opmode_oneshot)


# Get motor handlers
errorLeftMotor, leftMotor = sim.simxGetObjectHandle(
    clientID, "Pioneer_p3dx_leftMotor", sim.simx_opmode_oneshot_wait)
errorRightMotor, rightMotor = sim.simxGetObjectHandle(
    clientID, "Pioneer_p3dx_rightMotor", sim.simx_opmode_oneshot_wait)

# Get sensor handlers
errorLeftSensor, leftSensor = sim.simxGetObjectHandle(
    clientID, "Pioneer_p3dx_ultrasonicSensor9", sim.simx_opmode_oneshot_wait)

# Print possible errors when gettint handlers
print(errorLeftMotor, errorRightMotor, errorLeftSensor)


def setVelocity(leftV, rightV, clientID, rightMotor, leftMotor):
    sim.simxSetJointTargetVelocity(
        clientID, leftMotor, leftV, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(
        clientID, rightMotor, rightV, sim.simx_opmode_streaming)

# Turn options right, left, 180
def turn(direction, clientID, rightMotor, leftMotor):

    time_multiplier = 5;

    if(direction == 'left'):
        setVelocity(-1*time_multiplier, time_multiplier, clientID, rightMotor, leftMotor)
        time.sleep(1/time_multiplier)
        setVelocity(0, 0, clientID, rightMotor, leftMotor)
    elif(direction == 'right'):
        setVelocity(time_multiplier, -1*time_multiplier, clientID, rightMotor, leftMotor)
        time.sleep(1/time_multiplier)
        setVelocity(0, 0, clientID, rightMotor, leftMotor)
    elif(direction == '180'):
        setVelocity(-1.15*time_multiplier, 1.15*time_multiplier, clientID, rightMotor, leftMotor)
        time.sleep(1/time_multiplier)
        setVelocity(0, 0, clientID, rightMotor, leftMotor)
    

# 	number returnCode,boolean detectionState,array detectedPoint,number detectedObjectHandle,array detectedSurfaceNormalVector=simxReadProximitySensor(number clientID,number sensorHandle,number operationMode)


def initializeLeftSensor(clientID, leftSensor):
    # Roda a leitura do sensor de proximida pela primeira vez
    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVecto = sim.simxReadProximitySensor(
        clientID, leftSensor, sim.simx_opmode_streaming)
    print(returnCode)
    print(detectionState)


# Main
turn('180', clientID, rightMotor, leftMotor)

sim.simxAddStatusbarMessage(clientID, "Success!", sim.simx_opmode_oneshot_wait)

initializeLeftSensor(clientID, leftSensor)

count = 0


# Uso de sensores

# while True:
#     returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVecto = sim.simxReadProximitySensor(
#         clientID, leftSensor, sim.simx_opmode_buffer)

#     if(detectionState):
#         count += 1
#         sim.simxAddStatusbarMessage(
#             clientID, "Detectei algo!"+str(count), sim.simx_opmode_oneshot_wait)
