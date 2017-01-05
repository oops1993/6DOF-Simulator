import socket
import struct
HOST = '' # Symbolic name meaning all available interfaces
PORT = 27151 # Arbitrary non-privileged port
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen(1)
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        while True:
            data = conn.recv(192)
            data = struct.unpack('<ffffff',data)
            if not data: break
            #print("({0:>7,.2f},{1:>7,.2f},{2:>7,.2f},{3:>7,.2f},{4:>7,.2f},{5:>7,.2f})".format(data[0],data[1],data[2],data[3],data[4],data[5]))