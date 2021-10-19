# Import modules 
from datetime import datetime
import sys  
import time 
import numpy as np
import pandas as pd
import threading
import datetime


from pyextremes import __version__, get_extremes, EVA
from pyextremes.plotting import plot_extremes
from pyextremes import EVA


from scipy.stats import genextreme
import matplotlib.pyplot as plt
fig, ax = plt.subplots(1, 1)



# Constants
TIME_CONST = 5

# Colors
RED = "\033[1;31m"
BLUE = "\033[1;34m"
CYAN = "\033[1;36m"
GREEN = "\033[0;32m"
RESET = "\033[0;0m"
BOLD = "\033[;1m"
REVERSE = "\033[;7m"

# Adding directories for linux and mac
if(sys.platform == "linux" or sys.platform == "linux2"):
    sys.path.append('/home/rafael-barbosa/ptr_alternatives/ptr_project/PyBinding')
elif(sys.platform == 'darwin'):
    sys.path.append('/Users/admin/Documents/GitHub/sumo-robot-programing-and-simulation/PyBindingMac')

# Module to connect python to Coppelia
import sim


# Connect to CoppeliaSim
# Just in case, close all opened connections
sim.simxFinish(-1)

# Connect to CoppeliaSim
clientID = sim.simxStart('127.0.0.1', 19998, True, True, 5000, 5)

# End program if connection failed
if clientID != -1:
    print('Connected to remote API server')
else:
    print("Failed to connect to remote API server")
    print("Leaving program")
    sys.exit()


# Get motor handlers
errorLeftMotor, leftMotor = sim.simxGetObjectHandle(
    clientID, "Pioneer_p3dx_leftMotor", sim.simx_opmode_oneshot_wait)
errorRightMotor, rightMotor = sim.simxGetObjectHandle(
    clientID, "Pioneer_p3dx_rightMotor", sim.simx_opmode_oneshot_wait)

# Temperature sensor handler
errorTemperatureSensor, temperatureSensor = sim.simxGetObjectHandle(
    clientID, "TemperatureSensor", sim.simx_opmode_oneshot_wait)

# Print in handlers connections
print("Handlers: (0 == alright)")
print(errorLeftMotor, errorRightMotor)


# Threads controller ---------------------------------------------------
def initiateThread(direction, clientID, rightMotor, leftMotor):

    global clientRequests
    global currentThreadName
    global stopCurrentThread

    
    if(currentThreadName != ""):

        actionPriority  = clientRequests.index(direction)
        currentPriority = clientRequests.index(currentThreadName)

        # Debugging purpose
        print("CURRENT PRIORITY:", currentPriority)
        print("ACTION PRIORITY:", actionPriority)

        # If new task isn't highest priority it's not executed 
        if(currentPriority <= actionPriority):
            print("Action {} is less important or equal.".format(direction)) 
            return

        # If new taks is highest priority interrupts the other and execute
        else:
            print("Interrupts {} e runs {}!".format(currentThreadName, direction))
            stopCurrentThread = True
            currentThreadName = direction
    else:
        print("Run {}!".format(direction))
        currentThreadName = direction

    
    if(direction == 'EnemyOnLeft.'):
        # Create thread turn left
        t = threading.Thread(target=turnLeft, name=direction, args=(clientID, rightMotor, leftMotor))

        # Inicialize thread
        t.start()

    elif(direction == 'EnemyOnRight.'):
        # Create thread turn right
        t = threading.Thread(target=turnRight, name=direction, args=(clientID, rightMotor, leftMotor))

        # Inicialize thread
        t.start()

    elif(direction == 'LineDetected.'):    
        # Criada thread de virar 180
        t = threading.Thread(target=turn180, name=direction, args=(clientID, rightMotor, leftMotor))

        # Inicializa thread
        t.start()

    elif(direction == 'EnemyOnFront.'):
        # Create thread to acelerate
        t = threading.Thread(target=accelerate, name=direction, args=(clientID, rightMotor, leftMotor))
        
        # Initialize thread
        t.start()
        
        
        
# Function to set a velocity ------------------------------------------
def setVelocity(leftV, rightV, clientID, rightMotor, leftMotor):
    sim.simxSetJointTargetVelocity(
        clientID, leftMotor, leftV, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(
        clientID, rightMotor, rightV, sim.simx_opmode_oneshot_wait)




# Function to set a velocity measurement ------------------------------------------
def setVelocityMeasurement(leftV, rightV, clientID, rightMotor, leftMotor):
    startTime = time.time()
    global measurementsList
    sim.simxSetJointTargetVelocity(
        clientID, leftMotor, leftV, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(
        clientID, rightMotor, rightV, sim.simx_opmode_oneshot_wait)

    threadTime = time.time()- startTime
    measurementsList.append(threadTime)

# Threads -------------------------------------------------------------
def turnRight(clientID, rightMotor, leftMotor):
    startTime = time.time()
    global measurementsList
    global currentThreadName
    global stopCurrentThread
    stopCurrentThread = False

    print("Thread: ", threading.current_thread().name, "started.")

    # Starts to turn
    setVelocity(0.5*TIME_CONST, -0.5*TIME_CONST, clientID, rightMotor, leftMotor)
    
    finalTime = time.time() + 1/TIME_CONST
    while(time.time() < finalTime):
        if(stopCurrentThread):
            print("Thread: ", threading.current_thread().name, "stopped")
            stopCurrentThread = False
            currentThreadName = ""

            # setVelocity(0, 0, clientID, rightMotor, leftMotor)
            return

    # Finalizes thread if not interrupted before
    print("Thread: ", threading.current_thread().name, "finished.")
    currentThreadName = ""
    setVelocity(0, 0, clientID, rightMotor, leftMotor)

    threadTime = time.time()- startTime
    measurementsList.append(threadTime)

def turnLeft(clientID, rightMotor, leftMotor):
    startTime = time.time()
    global measurementsList
    global currentThreadName
    global stopCurrentThread
    stopCurrentThread = False

    print("Thread", threading.current_thread().name, "started.")

    # Starts to turn
    setVelocity(-0.5*TIME_CONST, 0.5*TIME_CONST, clientID, rightMotor, leftMotor)

    finalTime = time.time() + (1/TIME_CONST)
    while(time.time() < finalTime):
        if(stopCurrentThread):
            print("Thread: ", threading.current_thread().name, "stopped")
            stopCurrentThread = False
            currentThreadName = ""

            # setVelocity(0, 0, clientID, rightMotor, leftMotor)
            return

    # Finalizes thread if not interrupted before
    print("Thread: ", threading.current_thread().name, "finished.")
    currentThreadName = ""
    setVelocity(0, 0, clientID, rightMotor, leftMotor)

    threadTime = time.time()- startTime
    measurementsList.append(threadTime)


def turn180(clientID, rightMotor, leftMotor):
    startTime = time.time()
    global measurementsList
    global currentThreadName
    global stopCurrentThread
    stopCurrentThread = False
    print(RED, "Thread: ", threading.current_thread().name, "started.", RESET)
    
    # Starts to go back
    setVelocity(1.15*TIME_CONST, 1.15*TIME_CONST, clientID, rightMotor, leftMotor)
    
    finalTime = time.time() + (1/TIME_CONST)
    while(time.time() < finalTime):
        if(stopCurrentThread):
            print("Thread: ", threading.current_thread().name, "stopped")
            stopCurrentThread = False
            currentThreadName = ""
            return
    
    # Starts to turn
    setVelocity(-1.1*TIME_CONST, 1.1*TIME_CONST, clientID, rightMotor, leftMotor)
    
    finalTime = time.time() + (1/TIME_CONST)
    while(time.time() < finalTime):
        if(stopCurrentThread):
            print(RED, "Thread: ", threading.current_thread().name, "stopped", RESET)
            stopCurrentThread = False
            currentThreadName = ""
            return

    threadTime = time.time()- startTime
    measurementsList.append(threadTime)


    # Finalizes thread if not interrupted before
    print(RED, "Thread: ", threading.current_thread().name, "finished.", RESET)
    currentThreadName = ""
    setVelocity(0, 0, clientID, rightMotor, leftMotor)

def accelerate(clientID, rightMotor, leftMotor):
    startTime = time.time()
    global measurementsList
    global currentThreadName
    global stopCurrentThread
    stopCurrentThread = False

    print("Thread: ", threading.current_thread().name, "started.")
    duration = 0.2 # Impulse of 200ms

    # Sets velocity
    setVelocity(-3, -3, clientID, rightMotor, leftMotor)
    
    finalTime = time.time() + duration
    while(time.time() < finalTime):
        if(stopCurrentThread):
            print("Thread: ", threading.current_thread().name, "stopped")
            stopCurrentThread = False
            currentThreadName = ""

            # setVelocity(0, 0, clientID, rightMotor, leftMotor)
            return

    # Finalizes thread if not interrupted before
    print("Thread: ", threading.current_thread().name, "finished.")
    currentThreadName = ""
    setVelocity(0, 0, clientID, rightMotor, leftMotor)

    threadTime = time.time()- startTime
    measurementsList.append(threadTime)

def readTemperature(clientID, temperatureSensor):
    startTime = time.time()
    global measurementsList
    global stopCurrentThread
    global temperatureFlag
    stopCurrentThread = False

    print(CYAN, "Thread: ", threading.current_thread().name, "started", RESET)

    # Read temperature
    returnCodeEmergency, detectionStateEmergency, detectedPointEmergency, detectedObjectHandEmergency, detectedSurfaceNormalVectorEmergency = sim.simxReadProximitySensor(
        clientID, temperatureSensor, sim.simx_opmode_buffer)

    # If temperature is too high
    if(detectionStateEmergency):
        print(RED, "Tá quentão, mané, melhor parar!", RESET)
        setVelocity(0, 0, clientID, rightMotor, leftMotor)
        temperatureFlag = True
        return

    # Finalizes thread if not interrupted before
    print(CYAN, "Thread: ", threading.current_thread().name, "finished.", RESET)
    # currentThreadName = ""
    # setVelocity(0, 0, clientID, rightMotor, leftMotor)
    threadTime = time.time()- startTime
    measurementsList.append(threadTime)


# Client requests
clientRequests = ["Emergency.", "LineDetected.", "EnemyOnLeft.", "EnemyOnRight.", "EnemyOnFront.", "Break."]


# Main ----------------------------------------------------------------------------------------------------
sim.simxAddStatusbarMessage(clientID, "ControllerWaiting!", sim.simx_opmode_oneshot_wait)


# Inicializa sensor de temperatura
returnCode, detectionStateTemp, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, temperatureSensor, sim.simx_opmode_streaming)


# Variáveis globais
stopCurrentThread = False
currentThreadName = ""
temperatureFlag = False
lastTime = 0

temperatureCounter = 0
startTime = time.time()

measurementsList = []


# MEASUREMENTS TURN180
# for _ in range(250):
#     # Criada thread de virar 180
#     t = threading.Thread(target=turn180, name='direction', args=(clientID, rightMotor, leftMotor))
#     # Inicializa thread
#     t.start()
#     print("Detectou linha!!")
#     t.join()


# MEASUREMENTS TURN LEFT
# for _ in range(250):
#     # Criada thread de virar 180
#     t = threading.Thread(target=turnLeft, name='direction', args=(clientID, rightMotor, leftMotor))
#     # Inicializa thread
#     t.start()
#     t.join()


# MEASUREMENTS TURN RIGHT
# for _ in range(250):
#     # Criada thread de virar 180
#     t = threading.Thread(target=turnRight, name='direction', args=(clientID, rightMotor, leftMotor))
#     # Inicializa thread
#     t.start()
#     t.join()

# MEASUREMENTS ACCELERATE
# for _ in range(250):
#     # Criada thread de virar 180
#     t = threading.Thread(target=accelerate, name='direction', args=(clientID, rightMotor, leftMotor))
#     # Inicializa thread
#     t.start()
#     t.join()

# MEASUREMENTS READ TEMPERATURE
for i in range(250):
    # Criada thread de virar 180
    t = threading.Thread(target=readTemperature, name='direction', args=(clientID, temperatureSensor))
    # Inicializa thread
    t.start()
    t.join()

# MEASUREMENTS EMERGENCY
# for _ in range(250):
#     # Criada thread de virar 180
#     t = threading.Thread(target=setVelocityMeasurement, name='direction', args=(0,0, clientID, rightMotor, leftMotor))
#     # Inicializa thread
#     t.start()
#     t.join()

# MEASUREMENTS MOVE
# for _ in range(250):
#     # Criada thread de virar 180
#     t = threading.Thread(target=setVelocityMeasurement, name='direction', args=(-3,-3, clientID, rightMotor, leftMotor))
#     # Inicializa thread
#     t.start()
#     t.join()


print(measurementsList)
print(GREEN, 'MÉDIA: ', np.mean(measurementsList), RESET)
print(GREEN, 'MAX: ', np.max(measurementsList), RESET)
print(GREEN, 'DESVIO PADRÃO: ', np.std(measurementsList), RESET)

date_index = pd.date_range('12/29/2009', periods=250, freq='D')
measurementsSeries = pd.Series(measurementsList, index=date_index)     


# Cria um modelo a partir das medidas
model = EVA(measurementsSeries)

# Obtém os extremos pelo método blocos de máximos
model.get_extremes(method="BM",extremes_type="high",
    block_size="10D")

fig, ax = model.plot_extremes()
fig.savefig("images/extremes-temp.png", dpi=96, bbox_inches="tight")


# Encontra o modelo para a distribuição generalizada de valores extremos
model.fit_model()


# Obtém gráficos do modelo
fig2, ax2 = model.plot_diagnostic(alpha=0.95,  figsize=(16, 10))
fig2.savefig("images/analisys-temp.png", dpi=96, bbox_inches="tight")


# Statistics
programDuration = time.time()-startTime
print("Program duration:", programDuration)


# Ends communication with server
setVelocity(0, 0, clientID, rightMotor, leftMotor) 
sim.simxFinish(clientID)




