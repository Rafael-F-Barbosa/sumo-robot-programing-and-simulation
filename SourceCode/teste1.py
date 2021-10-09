# Import modules
import sys  
import time 
import numpy as np

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

errorFrontalSensor, frontalSensor = sim.simxGetObjectHandle(
    clientID, "Pioneer_p3dx_ultrasonicSensor13", sim.simx_opmode_oneshot_wait)

# Infrared sensor
errorInfraredSensor, infraredSensor = sim.simxGetObjectHandle(
    clientID, "Vision_sensor", sim.simx_opmode_oneshot_wait)

# Print possible errors when gettint handlers
print(errorLeftMotor, errorRightMotor, errorLeftSensor, errorInfraredSensor, errorFrontalSensor)


def setVelocity(leftV, rightV, clientID, rightMotor, leftMotor):
    sim.simxSetJointTargetVelocity(
        clientID, leftMotor, leftV, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(
        clientID, rightMotor, rightV, sim.simx_opmode_oneshot_wait)

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

# Sensor initialization
def initializeSensors(clientID, leftSensor, infraredSensor, frontalSensor):

    # Roda a leitura do sensor de proximidade esquerdo pela primeira vez
    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, leftSensor, sim.simx_opmode_streaming)
    
    # Roda a leitura do sensor de proximidade esquerdo pela primeira vez
    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, frontalSensor, sim.simx_opmode_streaming)

    # Roda a leitura do sensor infravermelho pela primeira vez
    # returnCode, resolution, image = sim.simxGetVisionSensorImage(clientID,infraredSensor, 1, sim.simx_opmode_streaming)

    # print(returnCode, resolution, image)



# Main ----------------------------------------------------------------------------------

# turn('180', clientID, rightMotor, leftMotor)

sim.simxAddStatusbarMessage(clientID, "Success!", sim.simx_opmode_oneshot_wait)

initializeSensors(clientID, leftSensor, infraredSensor, frontalSensor)




sim.simxAddStatusbarMessage(clientID, "Done!", sim.simx_opmode_oneshot_wait)

finalTime = time.time() + 10

while(time.time() < finalTime):
    
    # Anda de boas
    # setVelocity(-2, -2, clientID, rightMotor, leftMotor)

    # Lê sensor de infravermelho (imagem na verdade)
    # returnCode, resolution, image = sim.simxGetVisionSensorImage(clientID,infraredSensor, 1, sim.simx_opmode_buffer)
    # print(abs(np.mean(image)))

    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, frontalSensor, sim.simx_opmode_streaming)
    
    print(detectedObjectHandle)

    if(detectedObjectHandle != 14):
        sim.simxAddStatusbarMessage(clientID, "Detectei linha!"+"-"+str(np.mean(image)), sim.simx_opmode_oneshot_wait)

    
    # if(abs(np.mean(image)) < 15):

        # print('Linha detectada!')

        # setVelocity(0, 0, clientID, rightMotor, leftMotor)

        # turn('180', clientID, leftMotor, rightMotor)

        # sim.simxAddStatusbarMessage(clientID, "Detectei linha!"+"-"+str(np.mean(image)), sim.simx_opmode_oneshot_wait)

        # time.sleep(0.5)
            


print("Cabou")
# Para
setVelocity(0, 0, clientID, rightMotor, leftMotor) 




# Uso de sensores
# count = 0
# while True:
#     returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVecto = sim.simxReadProximitySensor(
#         clientID, infraredSensor, sim.simx_opmode_buffer)

#     print(detectedObjectHandle)
#     if(detectionState):
#         count += 1
#         sim.simxAddStatusbarMessage(
#             clientID, "Detectei algo!"+str(count), sim.simx_opmode_oneshot_wait)
