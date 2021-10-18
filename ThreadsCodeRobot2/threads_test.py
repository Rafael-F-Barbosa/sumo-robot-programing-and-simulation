# Python program killing
# a thread using multiprocessing
# module
 
import multiprocessing
import time
 
def func():
    print("Process started!")
    time.sleep(3)
    print("Process finished, not interrupted!")

 

proccess = multiprocessing.Process(target=func)
proccess.start()
time.sleep(1)

# proccess.terminate()
print("Process terminated!")
