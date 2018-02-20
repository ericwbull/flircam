#!/usr/bin/env python
import sys
import socket
import os, os.path
    
SOCKET_PATHNAME='/tmp/SerialStream.s'
RECV_BUFFER_SIZE=1024

# The server creates the socket and listens for the client to connect.
class Server():
    def __init__(self):
        if os.path.exists(SOCKET_PATHNAME):
            os.remove(SOCKET_PATHNAME)

        # Wait for the connection from the one and only client.
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind(SOCKET_PATHNAME)
        self.sock.listen(1)

        print "waiting for connection"
        self.connection, client_address = self.sock.accept()

        print "connection from '{}'".format(client_address)
        

    # Return whatever data is available from the socket stream
    def receive(self):
        data = self.connection.recv(RECV_BUFFER_SIZE)
        return data
    
    def send(self,data):
        self.connection.sendall(data)
