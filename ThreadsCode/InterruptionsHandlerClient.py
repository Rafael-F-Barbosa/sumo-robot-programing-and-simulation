# Import modules
import sys  
import time 
import numpy as np
import socket


 
# Adding directories for linux and mac
if(sys.platform == "linux" or sys.platform == "linux2"):
    sys.path.append('/home/rafael-barbosa/ptr_alternatives/ptr_project/PyBinding')
elif(sys.platform == 'darwin'):
    sys.path.append('/Users/admin/Documents/GitHub/sumo-robot-programing-and-simulation/PyBindingMac')

# Connects python to Coppelia
import sim  


# Connect to server controler
HOST = 'localhost'
PORT = 50010
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    s.connect((HOST, PORT))
except:
    print("Failed to connect to server controller")
    sys.exit()


# Connect to CoppeliaSim
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


# Get sensor handlers
errorLeftSensor, leftSensor = sim.simxGetObjectHandle(
    clientID, "Pioneer_p3dx_ultrasonicSensor9", sim.simx_opmode_oneshot_wait)
errorRightSensor, rightSensor = sim.simxGetObjectHandle(
    clientID, "Pioneer_p3dx_ultrasonicSensor16", sim.simx_opmode_oneshot_wait)
errorFrontSensor, frontSensor = sim.simxGetObjectHandle(
    clientID, "Pioneer_p3dx_ultrasonicSensor13", sim.simx_opmode_oneshot_wait)

# Infrared sensor
errorInfraredSensor, infraredSensor = sim.simxGetObjectHandle(
    clientID, "Vision_sensor", sim.simx_opmode_oneshot_wait)

# Emergency Button
errorEmergencySensor, emergencySensor = sim.simxGetObjectHandle(
    clientID, "EmergencySensor", sim.simx_opmode_oneshot_wait)

# Temperature sensor - Esse aqui acho que vai ficar no controller mesmo
errorTemperatureSensor, temperatureSensor = sim.simxGetObjectHandle(
    clientID, "TemperatureSensor", sim.simx_opmode_oneshot_wait)

# Print in handlers connections
print("Handlers: (0 == alright)")
print(errorLeftSensor, errorRightSensor, errorFrontSensor, errorInfraredSensor, errorEmergencySensor, errorTemperatureSensor)


# Read sensors first time
def initializeSensors(clientID, leftSensor, rightSensor, frontSensor, infraredSensor, emergencySensor, temperatureSensor):
    # Car sensors
    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, leftSensor, sim.simx_opmode_streaming)
    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, rightSensor, sim.simx_opmode_streaming)
    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, frontSensor, sim.simx_opmode_streaming)
    returnCode, resolution, image = sim.simxGetVisionSensorImage(clientID,infraredSensor, 1, sim.simx_opmode_streaming)

    # Emergency Sensor - Remote control
    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, emergencySensor, sim.simx_opmode_streaming)

    # Temperature Sensor - Remote control
    returnCode, detectionStateTemp, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, temperatureSensor, sim.simx_opmode_streaming)


# Main ----------------------------------------------------------------------------------------------------
initializeSensors(clientID, leftSensor, rightSensor, frontSensor, infraredSensor, emergencySensor, temperatureSensor)
sim.simxAddStatusbarMessage(clientID, "SensorsInitialized!", sim.simx_opmode_oneshot_wait)



# Simulate for some seconds
finalTime = time.time() + 30
while(time.time() < finalTime):
    # Read sensors
    returnCode, resolution, image = sim.simxGetVisionSensorImage(clientID,infraredSensor, 1, sim.simx_opmode_buffer)
    returnCodeLeft, detectionStateLeft, detectedPointLeft, detectedObjectHandLeft, detectedSurfaceNormalVectorLeft = sim.simxReadProximitySensor(
        clientID, leftSensor, sim.simx_opmode_buffer)
    returnCodeRight, detectionStateRight, detectedPointRight, detectedObjectHandRight, detectedSurfaceNormalVectorRight = sim.simxReadProximitySensor(
        clientID, rightSensor, sim.simx_opmode_buffer)
    returnCodeFront, detectionStateFront, detectedPointFront, detectedObjectHandFront, detectedSurfaceNormalVectorFront = sim.simxReadProximitySensor(
        clientID, frontSensor, sim.simx_opmode_buffer)
    # Emergency Sensor 
    returnCodeEmergency, detectionStateEmergency, detectedPointEmergency, detectedObjectHandEmergency, detectedSurfaceNormalVectorEmergency = sim.simxReadProximitySensor(
        clientID, emergencySensor, sim.simx_opmode_buffer)

    
  


    # Sending information
    if(detectionStateEmergency):
        s.send(str.encode("Emergency."))
        break
    elif(abs(np.mean(image)) < 30):
        s.send(str.encode("LineDetected."))
        print("Linha!")
    elif(detectionStateLeft):
        s.send(str.encode("EnemyOnLeft."))
        print("Inimigo a esquerda!")
    elif(detectionStateRight):
        s.send(str.encode("EnemyOnRight."))
        print("Inimigo a direita!")
    elif(detectionStateFront):
        s.send(str.encode("EnemyOnFront."))
        print("Inimigo à frente!")
    else:
        print("Nada detectado")
        s.send(str.encode("Nothing."))


    time.sleep(0.02)

    # Debugging
    # action = int(input("\nDigite: \n1-direita\n2-esq\n3-linha\n4-nda\n0-break"))
    # if(action == 0):
    #     s.send(str.encode("Break."))
    #     break
    # if(action == 1):
    #     s.send(str.encode("EnemyOnRight."))  
    # if(action == 2):
    #     s.send(str.encode("EnemyOnLeft."))
    # if(action == 3):
    #     s.send(str.encode("LineDetected."))
    
    # if(action == 4):
    #     s.send(str.encode("Nothing."))
    
    
    


# Ending program client and server
print("End program")
sim.simxAddStatusbarMessage(clientID, "SensorsFinished!", sim.simx_opmode_oneshot_wait)
s.send(str.encode("Break."))