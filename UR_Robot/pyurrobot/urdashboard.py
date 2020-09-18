#!/usr/bin/env python
# # -*- coding:utf-8 -*-
'''
@文件    :urdashboard.py
@说明    :The class to connect dashboard of universal robot, control the robot system with dashboard command.
@时间    :2020/09/01 11:20:57
@作者    :yuqiuda
@版本    :1.0
'''
import socket
import urx
import time
import RG_gripper as rg


class URDashboard():
    """
    @function    :initialize the URDashboard class
    @parameter   :ip----type(str), e.g. '192.168.1.12'
    """
    def __init__(self, ip='192.168.1.19'):
        # self.arm = urx.Robot(ip)
        # self.rg2 = rg.RG2(self.arm)
        self.addr = (ip, 29999)
        self.client = socket.socket()
        self.client.settimeout(3)
        try:
            self.client.connect(self.addr)
            print(self.client.recv(1024))
        except:
            raise


    def sendCommand(self, cmd):    
        '''
        @function    :send dashboard command to the robot dashboard
        @parameter   :cmd----type(str), e.g. 'power on'
        '''
        cmd += '\n'
        self.client.send(cmd.encode('utf-8'))
        return self.client.recv(1024)

    
    def poweron(self):
        """
        power on the robot and release the brake
        """
        self.sendCommand('power on')
        t = 0
        while True:
            if self.sendCommand('robotmode')=='Robotmode: POWER_OFF\n':
                time.sleep(1)
                t = t + 1
                if t > 5:
                    raise Exception('robot bring up time out!')
        self.sendCommand('brake release')
        print("robot is bring up")
        return True

    
    def poweroff(self):
        """
        power off the robot and lock the brake
        """
        self.sendCommand('power off')
