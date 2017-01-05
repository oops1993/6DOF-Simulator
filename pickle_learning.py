import socket
import struct
import time

udp2_ip = "127.0.0.1"
udp2_port=27151
udp2_socket=0
udp2_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
udp2_socket.settimeout(1)
	
def connnetServer():
	global udp2_ip, udp2_socket, udp2_port
	try:
		udp2_socket.connect((udp2_ip, udp2_port))
		print("udp2_socket connected server")
	except Exception,e:
		print("udp2_socket failed to connect server: {0}".format(e))
connnetServer()
def sendData(PackingString, PackingList):
	global udp2_ip, udp2_socket, udp2_port
	info = struct.pack(PackingString, *PackingList)
	try:
		udp2_socket.sendall(info)
	except Exception,e:
		print(str(e))
		connnetServer()