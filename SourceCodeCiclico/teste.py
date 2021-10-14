# Import modules
import sys  
import time 
import numpy as np
 

# Adding directories for linux and mac
if(sys.platform == "linux" or sys.platform == "linux2"):
    sys.path.append('/home/rafael-barbosa/ptr_project/PyBinding')
elif(sys.sys.platform == 'darwin'):
    sys.path.append('/Users/admin/Documents/Mecatronica/8o semestre/PTR/Projeto 2/codigo')

# Interface between python and Coppelia
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

# Temperature sensor
errorTemperatureSensor, temperatureSensor = sim.simxGetObjectHandle(
    clientID, "TemperatureSensor", sim.simx_opmode_oneshot_wait)


# Print in handlers connections
print("Handlers: (0 == alright)")
print(errorLeftMotor, errorRightMotor, errorLeftSensor, errorRightSensor, errorFrontSensor, errorInfraredSensor, errorEmergencySensor, errorTemperatureSensor)


# Function to set a velocity
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
    elif(direction == 'back'):
        setVelocity(1.15*time_multiplier, 1.15*time_multiplier, clientID, rightMotor, leftMotor)
        time.sleep(1/time_multiplier)
        setVelocity(0, 0, clientID, rightMotor, leftMotor)

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


# Function to read temperature
def readTemperature(clientID, temperatureSensor):
    returnCodeTemp, detectionStateTemp, detectedPointTemp, detectedObjectHandTemp, detectedSurfaceNormalVectorTemp = sim.simxReadProximitySensor(
        clientID, temperatureSensor, sim.simx_opmode_buffer)
    if(detectionStateTemp): 
        sim.simxAddStatusbarMessage(clientID, "TEMPERATURA ALTA, irmão, para tudo!!!!", sim.simx_opmode_oneshot_wait)
        print("Temperatura alta - Finalizando programa")
        return 1
    print('LI TEMPERATURA')
    sim.simxAddStatusbarMessage(clientID, "LI TEMPERATURA", sim.simx_opmode_oneshot_wait)

    return 0


# Main ----------------------------------------------------------------------------------------------------
sim.simxAddStatusbarMessage(clientID, "Main program started!", sim.simx_opmode_oneshot_wait)

initializeSensors(clientID, leftSensor, rightSensor, frontSensor, infraredSensor, emergencySensor, temperatureSensor)

# Period - Read Temperature
pReadTemperature = 3
t1 = 0
t2 = 0

# Simulate for 60 seconds
finalTime = time.time() + 30
while(time.time() < finalTime):
    
    # Go ahead
    setVelocity(-4, -4, clientID, rightMotor, leftMotor)

    # Read Temperature  - IMPOSSIVEL FAZER POR THREAD
    t1 = time.time()
    if (t1 - t2) > 3:
        high = readTemperature(clientID, temperatureSensor)
        t2 = time.time()
        if high:
            break



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



    # Basic funcionality
    if(abs(np.mean(image)) < 30):
        setVelocity(0, 0, clientID, rightMotor, leftMotor)
        turn('back', clientID, leftMotor, rightMotor)
        turn('180', clientID, leftMotor, rightMotor)
        sim.simxAddStatusbarMessage(clientID, "Detectei linha!!!!"+"-"+str(np.mean(image)), sim.simx_opmode_oneshot_wait)
    elif(detectionStateEmergency): # Emergência tá com prioridade menor que linha aqui, mas era só pra testar
        sim.simxAddStatusbarMessage(clientID, "Emergência, irmão, para tudo!!!!", sim.simx_opmode_oneshot_wait)
        print("Emergência - Finalizando programa")
        break
    elif(detectionStateLeft):
        turn('left', clientID, rightMotor,leftMotor)
        sim.simxAddStatusbarMessage(clientID, "Adversário à esquerda!!!!"+"-"+str(np.mean(image)), sim.simx_opmode_oneshot_wait)
    elif(detectionStateRight):
        turn('right', clientID, rightMotor,leftMotor)
        sim.simxAddStatusbarMessage(clientID, "Adversário à direita!!!!"+"-"+str(np.mean(image)), sim.simx_opmode_oneshot_wait)
    elif(detectionStateFront):
        setVelocity(-7, -7, clientID, rightMotor, leftMotor)
        sim.simxAddStatusbarMessage(clientID, "Adversário à Frente!!!!"+"-"+str(np.mean(image)), sim.simx_opmode_oneshot_wait)
    

# Stop robot
setVelocity(0, 0, clientID, rightMotor, leftMotor) 

# Ends communication with server
sim.simxFinish(clientID)