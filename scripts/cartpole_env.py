import numpy as np
import socket
import struct
import time

class CartpoleEnv:
    def __init__(self, port_self, port_remote):
        print("Init CartpoleEnv")
        
        self.port_self_ = port_self
        self.port_remote_ = port_remote

        self.fd_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.fd_.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 112)       # set receive buffer size to be 112 bytes (1 double = 8 bytes)
        #self.fd_.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 40)        # set send buffer size to be 40 bytes. Both 112 bytes and 40 bytes are lower than the minimum value, so buffer size will be set as minimum value. See "'man 7 socket"
        self.fd_.bind( ("", self.port_self_) )
        self.addr_remote_send_ = ("", self.port_remote_)
        self.msg_len_ = 30
        # data = [0.0]*self.msg_len_
        # data[0], data[1], data[2], data[3] = 0, 10, 0, 0.1
        buf = struct.pack('30f', 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.fd_.sendto(buf, self.addr_remote_send_)

    def __del__(self):
        self.fd_.close() 

    def step(self, action):

        buf = struct.pack('30f', 0, 0, 0, action, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.fd_.sendto(buf, self.addr_remote_send_)

        data, addr_remote_ = self.fd_.recvfrom(4*self.msg_len_)     # 1 float = 4 bytes, recvfrom is blocking
        model_id,data_type,time,px,vx,theta,omega,_,_,_, _,_,_,_,_,_,_,_,_,_, _,_,_,_,_,_,_,_,_,_ = struct.unpack('30f',data)
        self.state_current_ = np.array([model_id,data_type,time,px,vx,theta,omega])

        return self.state_current_

    def reset(self):
        return self.state_current_
             

    def getTime(self):
        return self.state_current_[2]


