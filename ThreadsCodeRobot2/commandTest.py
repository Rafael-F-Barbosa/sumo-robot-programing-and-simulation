# Import modules 
import sys  
import time 
import numpy as np
import socket
import threading
 

# Adding directories for linux and mac
if(sys.platform == "linux" or sys.platform == "linux2"):
    sys.path.append('/home/rafael-barbosa/ptr_project/PyBinding')
elif(sys.platform == 'darwin'):
    sys.path.append('/Users/admin/Documents/Mecatronica/8o semestre/PTR/Projeto 2/codigo')

# Module to connect python to Coppelia
import sim  

# Function to set a velocity
def setVelocity(leftV, rightV, clientID, rightMotor, leftMotor):
    sim.simxSetJointTargetVelocity(
        clientID, leftMotor, leftV, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(
        clientID, rightMotor, rightV, sim.simx_opmode_oneshot_wait)


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



def turnLeft(clientID, rightMotor, leftMotor):
    print("Thread", threading.current_thread().name, "started.")
    
    timeConstant = 5
    finalTime = time.time() + (1/timeConstant)

    # Starts to turn
    setVelocity(-1.15*timeConstant, 1.15*timeConstant, clientID, rightMotor, leftMotor)
    
    while(time.time() < finalTime):
        global stopCurrentThread
        if(stopCurrentThread):
            print("Thread: ", threading.current_thread().name, "stopped")
            stopCurrentThread = False
            setVelocity(0, 0, clientID, rightMotor, leftMotor)
            return

    # Finalizes Thread if no interrupted before
    global currentThreadName
    print("Thread: ", threading.current_thread().name, "finished")
    currentThreadName = ""
    setVelocity(0, 0, clientID, rightMotor, leftMotor)




setVelocity(-0.5, -0.5, clientID, rightMotor, leftMotor)
time.sleep(3)

stopCurrentThread = False
# Criada thread de virar a esquerda
t = threading.Thread(target=turnLeft, name="Vira esquerda!", args=(clientID, rightMotor, leftMotor))

# Inicializa thread
t.start()

print("jorge")

setVelocity(-0, -0, clientID, rightMotor, leftMotor)


t.join()


setVelocity(-0.5, -0.5, clientID, rightMotor, leftMotor)
time.sleep(3)

setVelocity()