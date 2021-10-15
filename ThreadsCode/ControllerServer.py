# Import modules 
import sys  
import time 
import numpy as np
import socket
import threading
 

# Adding directories for linux and mac
if(sys.platform == "linux" or sys.platform == "linux2"):
    sys.path.append('/home/rafael-barbosa/ptr_project/PyBinding')
elif(sys.sys.platform == 'darwin'):
    sys.path.append('/Users/admin/Documents/Mecatronica/8o semestre/PTR/Projeto 2/codigo')

# Module to connect python to Coppelia
import sim  


# Create server connection and listen 
HOST = 'localhost'
PORT = 50006
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    s.bind((HOST, PORT))
except :
    print("Failed to create server!")
    sys.exit()
s.listen()
print("Controller waiting client...")
# Connect to client
conn, add = s.accept()
print("Connected to", add)


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

# Print in handlers connections
print("Handlers: (0 == alright)")
print(errorLeftMotor, errorRightMotor)


# Turn options right, left, 180
def initiateThread(direction, clientID, rightMotor, leftMotor):

    global clientRequests
    global currentThreadName
    global stopCurrentThread

    
    if(currentThreadName != ""):

        actionPriority  = clientRequests.index(direction)
        currentPriority = clientRequests.index(currentThreadName)

        # Se a tarefa for de prioridade inferior ou igual não executará
        if(currentPriority <= actionPriority):
            print("Action {} is less important or equal.".format(direction)) 
            return

        # Se a prioridade for maior, é interrompida a thread atual e a nova assume
        else:
            print("Interrupts {} e runs {}!".format(currentThreadName, direction))
            stopCurrentThread = True
            currentThreadName = direction
    else:
        print("Run {}!".format(direction))
        currentThreadName = direction

    
    if(direction == 'EnemyOnLeft.'):
        # Criada thread de virar a esquerda
        t = threading.Thread(target=turnLeft, name=direction, args=(clientID, rightMotor, leftMotor))

        # Inicialize thread
        t.start()

    elif(direction == 'EnemyOnRight.'):
        # Criada thread de virar a direita
        t = threading.Thread(target=turnRight, name=direction, args=(clientID, rightMotor, leftMotor))

        # Inicialize thread
        t.start()

    elif(direction == 'LineDetected.'):
        # Stops
        setVelocity(0, 0, clientID, rightMotor, leftMotor)
    
        # Criada thread de virar 180
        t = threading.Thread(target=turn180, name=direction, args=(clientID, rightMotor, leftMotor))

        # Inicializa thread
        t.start()

    elif(direction == 'EnemyOnFront.'):
        # Create thread to acelerate
        t = threading.Thread(target=accelerate, name=direction, args=(clientID, rightMotor, leftMotor))
        
        # Initialize thread
        t.start()
        
        
        




# Function to set a velocity
def setVelocity(leftV, rightV, clientID, rightMotor, leftMotor):
    sim.simxSetJointTargetVelocity(
        clientID, leftMotor, leftV, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(
        clientID, rightMotor, rightV, sim.simx_opmode_oneshot_wait)

# Threads
def turnRight(clientID, rightMotor, leftMotor):
    print("Thread: ", threading.current_thread().name, "started.")
    timeConstant = 5 

    # Starts to turn
    setVelocity(0.5*timeConstant, -0.5*timeConstant, clientID, rightMotor, leftMotor)
    
    finalTime = time.time() + 1/timeConstant
    while(time.time() < finalTime):
        global stopCurrentThread
        if(stopCurrentThread):
            print("Thread: ", threading.current_thread().name, "stopped")
            stopCurrentThread = False
            # setVelocity(0, 0, clientID, rightMotor, leftMotor)
            return

    # Finalizes thread if not interrupted before
    print("Thread: ", threading.current_thread().name, "finished.")
    global currentThreadName
    currentThreadName = ""
    setVelocity(0, 0, clientID, rightMotor, leftMotor)

def turnLeft(clientID, rightMotor, leftMotor):
    print("Thread", threading.current_thread().name, "started.")
    timeConstant = 5

    # Starts to turn
    setVelocity(-0.5*timeConstant, 0.5*timeConstant, clientID, rightMotor, leftMotor)
    
    finalTime = time.time() + (1/timeConstant)
    while(time.time() < finalTime):
        global stopCurrentThread
        if(stopCurrentThread):
            print("Thread: ", threading.current_thread().name, "stopped")
            stopCurrentThread = False
            # setVelocity(0, 0, clientID, rightMotor, leftMotor)
            return

    # Finalizes thread if not interrupted before
    print("Thread: ", threading.current_thread().name, "finished.")
    global currentThreadName
    currentThreadName = ""
    setVelocity(0, 0, clientID, rightMotor, leftMotor)


def turn180(clientID, rightMotor, leftMotor):
    print("Thread: ", threading.current_thread().name, "started.")
    timeConstant = 5 
    
    # Starts to turn
    setVelocity(-1.0*timeConstant, 1.0*timeConstant, clientID, rightMotor, leftMotor)
    
    finalTime = time.time() + (1/timeConstant)
    while(time.time() < finalTime):
        global stopCurrentThread
        if(stopCurrentThread):
            print("Thread: ", threading.current_thread().name, "stopped")
            stopCurrentThread = False
            # setVelocity(0, 0, clientID, rightMotor, leftMotor)
            return

    # Finalizes thread if not interrupted before
    print("Thread: ", threading.current_thread().name, "finished.")
    global currentThreadName
    currentThreadName = ""
    setVelocity(0, 0, clientID, rightMotor, leftMotor)


def goBack(clientID, rightMotor, leftMotor):
    print("Thread: ", threading.current_thread().name, "started.")
    timeConstant = 5

    # Starts to go back
    setVelocity(1.15*timeConstant, 1.15*timeConstant, clientID, rightMotor, leftMotor)

    finalTime = time.time() + (1/timeConstant)
    while(time.time() < finalTime):
        global stopCurrentThread
        if(stopCurrentThread):
            print("Thread: ", threading.current_thread().name, "stopped")
            stopCurrentThread = False
            # setVelocity(0, 0, clientID, rightMotor, leftMotor)
            return

    # Finalizes thread if not interrupted before
    print("Thread: ", threading.current_thread().name, "finished.")
    global currentThreadName
    currentThreadName = ""
    setVelocity(0, 0, clientID, rightMotor, leftMotor)

def accelerate(clientID, rightMotor, leftMotor):
    print("Thread: ", threading.current_thread().name, "started.")
    duration = 0.2 # Impulse of 200ms

    # Sets velocity
    setVelocity(-3, -3, clientID, rightMotor, leftMotor)
    
    finalTime = time.time() + duration
    while(time.time() < finalTime):
        global stopCurrentThread
        if(stopCurrentThread):
            print("Thread: ", threading.current_thread().name, "stopped")
            stopCurrentThread = False
            # setVelocity(0, 0, clientID, rightMotor, leftMotor)
            return

    # Finalizes thread if not interrupted before
    print("Thread: ", threading.current_thread().name, "finished.")
    global currentThreadName
    currentThreadName = ""
    setVelocity(0, 0, clientID, rightMotor, leftMotor)



# Client requests
clientRequests = ["Emergency.", "LineDetected.", "EnemyOnLeft.", "EnemyOnRight.", "EnemyOnFront.", "Break."]


# Main ----------------------------------------------------------------------------------------------------
sim.simxAddStatusbarMessage(clientID, "ControllerWaiting!", sim.simx_opmode_oneshot_wait)


# Variáveis globais
stopCurrentThread = False
currentThreadName = ""


startTime = time.time()
while(True):
    # Go ahead
    print("Thread count: {}".format(threading.active_count()))

    # If there is no other active thread, go 
    if(threading.active_count() == 1):
        setVelocity(-1, -1, clientID, rightMotor, leftMotor)

    # Receive sensor information
    data = conn.recv(64)
    print(data)

    # Processa data
    if(data not in clientRequests):
        tokens = data.decode().split(".")
        correctData = tokens[-2] + "."

    print("CorrectedData: ", correctData)

    # Basic funcionality
    if(correctData == "LineDetected."):
        print("Detectou linha!!")
        setVelocity(0, 0, clientID, rightMotor, leftMotor)
        initiateThread('LineDetected.', clientID, leftMotor, rightMotor)
    elif(correctData == "Emergency."): # Emergência tá com prioridade menor que linha aqui, mas era só pra testar
        print("Emergência - Finalizando programa")
        setVelocity(0, 0, clientID, rightMotor, leftMotor) 
        sim.simxAddStatusbarMessage(clientID, "Emergência, irmão, para tudo!!!!", sim.simx_opmode_oneshot_wait)
        break
    elif(correctData == "EnemyOnLeft."):
        print("Detectou esquerda!!")
        initiateThread('EnemyOnLeft.', clientID, rightMotor,leftMotor)
        sim.simxAddStatusbarMessage(clientID, "Adversário à esquerda!!!!", sim.simx_opmode_oneshot_wait)
    elif(correctData == "EnemyOnRight."):
        print("Detectou direita!!")
        initiateThread('EnemyOnRight.', clientID, rightMotor,leftMotor)
        sim.simxAddStatusbarMessage(clientID, "Adversário à direita!!!!", sim.simx_opmode_oneshot_wait)
    elif(correctData == "EnemyOnFront."):
        print("Inimigo na frente, mané!")
        initiateThread('EnemyOnFront.', clientID, leftMotor, rightMotor)
        sim.simxAddStatusbarMessage(clientID, "Adversário à Frente!!!!", sim.simx_opmode_oneshot_wait)
    elif(correctData == 'Break.' or not data):
        setVelocity(0, 0, clientID, rightMotor, leftMotor) 
        conn.close()
        break

# Statistics
print("Program duration: ", time.time()-startTime)

# Ends communication with server
print("Server finished!")
sim.simxAddStatusbarMessage(clientID, "ControllerFinished!", sim.simx_opmode_oneshot_wait)
sim.simxFinish(clientID)


