# Python program killing
# a thread using multiprocessing
# module
 
import threading
import time


def turn_right():

    global global_thread_name

    global_thread_name = threading.current_thread().name;

    print("Thread: ", threading.current_thread().name, "started")
    finalTime = time.time() + 5

    while time.time() < finalTime:
        global stop_global_thread
        if(stop_global_thread):
            print("Thread: ", threading.current_thread().name, "stopped")
            stop_global_thread = False
            return
    
    
    print("Thread",threading.current_thread().name,"finalize.", time.time())


def turn_left():

    global global_thread_name
    print("Global:", global_thread_name)
    if(global_thread_name != ""):
        print("Impossible to continue!")
        return

    print("Thread: ", threading.current_thread().name, "started")
    finalTime = time.time() + 5

    while time.time() < finalTime:
        global stop_global_thread
        if(stop_global_thread):
            print("Thread: ", threading.current_thread().name, "stopped")
            stop_global_thread = False
            return
    
    
    print("Thread",threading.current_thread().name,"finalize.", time.time())

stop_global_thread = False
global_thread_name = ""

t1 = threading.Thread(target=turn_right, name="ThreadTurnRight")
t2 = threading.Thread(target=turn_left, name="ThreadTurnLeft")


t1.start()
time.sleep(1)

t2.start()
time.sleep(1)

stop_global_thread = True
print("The end!", time.time())


def main():
    