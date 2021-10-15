import socket
import time


HOST = 'localhost'
PORT = 50051

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.connect((HOST, PORT))

# while(True):
#     # Envia mensagem do teclado para o servidor
#     # msg = input("Digite (0 - sair): ")
#     # s.sendall(str.encode(msg))
#     for x in range(15):
#         s.sendall(str.encode(str(x)))
    
#     # Recebe de volta informações do servidor
#     data = s.recv(1024)
#     print(data.decode())
#     if(data.decode() == '0'):
#         break

for x in range(1, 10):
    s.send(str.encode(str(x)))
    # time.sleep(.01)

time.sleep(1)
s.sendall(str.encode(str('0')))