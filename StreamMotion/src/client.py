from src.utils import *
from socket import *
import numpy as np


class UDPClient():
  def __init__(self, ip, port=60015):
    self.UDP_IP = ip
    self.UDP_PORT = port
    
    self.sock = socket(AF_INET, SOCK_DGRAM)
  
  def connect(self):
    self.sock.connect((self.UDP_IP, self.UDP_PORT))

# Communication Packs 
  def send_init_pack(self):
    data = initpack()
    self.sock.sendto(data, (self.UDP_IP, self.UDP_PORT))
      
    resp = self.sock.recv(132)
    return explainRobData(resp)
  
  def send_command_pack(self, data):
    # data = commandpack(data) already called within main file
    self.sock.sendto(data, (self.UDP_IP, self.UDP_PORT))
    
    resp = self.sock.recv(132)
    return explainRobData(resp)
  
  def send_end_pack(self):
    data = endpack()
    self.sock.sendto(data, (self.UDP_IP, self.UDP_PORT))
    self.sock.close()

# Limit Packs

  def send_vel_pack(self, axis):
    data = velocitypack(axis)
    self.sock.sendto(data, (self.UDP_IP, self.UDP_PORT))
    
    resp = self.sock.recv(184)
    return explainLimitResponse(resp)
  
  def send_acc_pack(self, axis):
    data = accelerationpack(axis)
    self.sock.sendto(data, (self.UDP_IP, self.UDP_PORT))

    resp = self.sock.recv(184)
    return explainLimitResponse(resp)
  
  def send_jerk_pack(self, axis):
    data = jerkpack(axis)
    self.sock.sendto(data, (self.UDP_IP, self.UDP_PORT))

    resp = self.sock.recv(184)
    return explainLimitResponse(resp)