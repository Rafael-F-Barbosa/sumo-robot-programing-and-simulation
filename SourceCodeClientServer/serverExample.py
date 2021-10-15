# Importa biblioteca socket
import socket
import time


HOST = 'localhost'
PORT = 50051

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.bind((HOST, PORT))

s.listen()

print("Controller waiting")

conn, add = s.accept()

print("Connectado em", add)

while True:
    data = conn.recv(1024)
    
    print(data.decode())
    
    # conn.sendall(data) # Ecoa informação
    
    if (data.decode() == '0') :
        print("Closing connection")
        break
    else:
        print(data)


time.sleep(1)
conn.close()