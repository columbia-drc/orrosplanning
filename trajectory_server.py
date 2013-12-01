import socket
import threading
import numpy
import math
import ipdb

class JointAngleSender():
    def __init__(self, port=4765):
        host=''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((host, port))
        self.client = []
        self.server_thread = threading.Thread(target=self.wait_for_connection)
        self.server_thread.start()

    def send_trajectory(self, waypoint_list, side):

        waypoint_string = side + '\n'+ '\n'.join([' '.join([str(s/math.pi * 180.0) for s in way_point]) for way_point in waypoint_list])
        if self.client:           
            self.client.send(waypoint_string)
        else:
            print "no connected client"
            print waypoint_string


    def wait_for_connection(self):
        self.sock.listen(1)
        self.client, unused_addr = self.sock.accept()
        
        
