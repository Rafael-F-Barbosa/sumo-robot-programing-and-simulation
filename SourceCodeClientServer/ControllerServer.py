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
PORT = 50013
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


# Turn options right, left, 180 - Talvez não usaremos isso aqui
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


# Function to set a velocity
def setVelocity(leftV, rightV, clientID, rightMotor, leftMotor):
    sim.simxSetJointTargetVelocity(
        clientID, leftMotor, leftV, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(
        clientID, rightMotor, rightV, sim.simx_opmode_oneshot_wait)

# Inicializei essa divisão aqui pra usar threads - mas n apliquei ainda
def turn_right(direction, clientID, rightMotor, leftMotor):
    time_multiplier = 5;
    setVelocity(time_multiplier, -1*time_multiplier, clientID, rightMotor, leftMotor)
    time.sleep(1/time_multiplier)
    setVelocity(0, 0, clientID, rightMotor, leftMotor)

def turn_left(direction, clientID, rightMotor, leftMotor):
    time_multiplier = 5;
    setVelocity(-1*time_multiplier, time_multiplier, clientID, rightMotor, leftMotor)
    time.sleep(1/time_multiplier)
    setVelocity(0, 0, clientID, rightMotor, leftMotor)

def turn180(direction, clientID, rightMotor, leftMotor):
    time_multiplier = 5;
    setVelocity(-1.15*time_multiplier, 1.15*time_multiplier, clientID, rightMotor, leftMotor)
    time.sleep(1/time_multiplier)
    setVelocity(0, 0, clientID, rightMotor, leftMotor)

def goBack(direction, clientID, rightMotor, leftMotor):
    time_multiplier = 5;
    setVelocity(1.15*time_multiplier, 1.15*time_multiplier, clientID, rightMotor, leftMotor)
    time.sleep(1/time_multiplier)
    setVelocity(0, 0, clientID, rightMotor, leftMotor)



# Client requests
clientRequests = ["Emergency.", "LineDetected.", "EnemyOnLeft.", "EnemyOnRight.", "EnemyOnFront.", "Break."]


# Main ----------------------------------------------------------------------------------------------------
sim.simxAddStatusbarMessage(clientID, "ControllerWaiting!", sim.simx_opmode_oneshot_wait)



startTime = time.time()
while(True):
    # Go ahead
    setVelocity(-4, -4, clientID, rightMotor, leftMotor)


    # Receive sensor information
    data = conn.recv(1024)

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
        turn('back', clientID, leftMotor, rightMotor)
        turn('180', clientID, leftMotor, rightMotor)
        sim.simxAddStatusbarMessage(clientID, "Detectei linha!!!!", sim.simx_opmode_oneshot_wait)
    elif(correctData == "Emergency."): # Emergência tá com prioridade menor que linha aqui, mas era só pra testar
        sim.simxAddStatusbarMessage(clientID, "Emergência, irmão, para tudo!!!!", sim.simx_opmode_oneshot_wait)
        print("Emergência - Finalizando programa")
        break
    elif(correctData == "EnemyOnLeft."):
        turn('left', clientID, rightMotor,leftMotor)
        print("Detectou esq!!")
        sim.simxAddStatusbarMessage(clientID, "Adversário à esquerda!!!!", sim.simx_opmode_oneshot_wait)
    elif(correctData == "EnemyOnRight."):
        turn('right', clientID, rightMotor,leftMotor)
        print("Detectou dire!!")
        sim.simxAddStatusbarMessage(clientID, "Adversário à direita!!!!", sim.simx_opmode_oneshot_wait)
    elif(correctData == "EnemyOnFront."):
        setVelocity(-7, -7, clientID, rightMotor, leftMotor)
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