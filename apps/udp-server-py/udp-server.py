import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-p", "--port", help="Port to bind to", default=3000, type=int)
parser.add_argument("-b", "--buffer", help="Buffer size", default=1024, type=int)
args = parser.parse_args()

import socket 
IP = "aaaa::1"
PORT = args.port

sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
sock.bind((IP, PORT))
print("Server inicializado, esperando datos. Usar Ctrl+C para detener")
while True:
	data, address = sock.recvfrom(args.buffer)
	print(data)
	#print(address)