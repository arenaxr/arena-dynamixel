#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Read and Write Example      *********
#
#
# Available DXL model on this example : All models using Protocol 1.0
# This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
# Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 30
ADDR_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_PAN_ID                  = 1                 # Dynamixel ID : 1
DXL_TILT_ID                 = 2                 # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/tty.usbserial-FT6RW6MQ'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
FULL_REVOLUTION             = 360

APPROX_CENTER_POS           = 518
PAN_UPPER_BOUND             = 1020
PAN_LOWER_BOUND             = 20

TILT_UPPER_BOUND            = 660
TILT_LOWER_BOUND            = 390

JUMP_THRESHOLD              = 50

class Dynamixel_Servo:
    def __init__(self, port):
        self.port = port
        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.dxl_goal_position_pan = APPROX_CENTER_POS
        self.dxl_goal_position_tilt = APPROX_CENTER_POS
    
    def connect_Dynamixel(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

    def init_Dynamixel(self):
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_PAN_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel Pan has been successfully connected")
        
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_TILT_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel Tilt has been successfully connected")
        
         
        # set intial position
        dxl_present_position_pan, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL_PAN_ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        
        self.dxl_goal_position_pan = dxl_present_position_pan


        dxl_present_position_tilt, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL_TILT_ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        
        self.dxl_goal_position_tilt = dxl_present_position_tilt

       
        print("Dynamixel Intialized")
        #print(self.dxl_goal_position)



    def __rotate_Motor(self, id):
        # print("Press any key to continue! (or press ESC to quit!)")
        # if getch() == chr(0x1b):
        #     quit()

        # Write goal position
        if id == DXL_PAN_ID:
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_GOAL_POSITION, self.dxl_goal_position_pan)
        else:
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_GOAL_POSITION, self.dxl_goal_position_tilt)
     
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("in rotate motor %s" % self.packetHandler.getRxPacketError(dxl_error))

        while 1:
            # Read present position
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, id, ADDR_PRESENT_POSITION)
            
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
    
            #currAngle = ((dxl_present_position * FULL_REVOLUTION) // DXL_MAXIMUM_POSITION_VALUE) % FULL_REVOLUTION
            #print("[ID:%03d] GoalAngle:%03d  PresAngle:%03d" % (DXL_ID, angle % FULL_REVOLUTION, currAngle))
            #print("id: %d   pres_pos: %d  goal_pos_pan: %d    goal_pos_tilt = %d" % (id, dxl_present_position, self.dxl_goal_position_pan, self.dxl_goal_position_tilt))
            if id == DXL_PAN_ID and not abs(self.dxl_goal_position_pan - (dxl_present_position) ) > DXL_MOVING_STATUS_THRESHOLD:
                return
            elif not abs(self.dxl_goal_position_tilt  - (dxl_present_position) ) > DXL_MOVING_STATUS_THRESHOLD:
                return
        
            
    def rotate_Degrees(self, degrees, id):
        if id == DXL_PAN_ID:
            self.dxl_goal_position_pan = (self.dxl_goal_position_pan + int(float(degrees) * float(DXL_MAXIMUM_POSITION_VALUE) / float(FULL_REVOLUTION)))
            if not (PAN_UPPER_BOUND >= self.dxl_goal_position_pan >= PAN_LOWER_BOUND):
                return
        else:
            self.dxl_goal_position_tilt = (self.dxl_goal_position_tilt + int(float(degrees) * float(DXL_MAXIMUM_POSITION_VALUE) / float(FULL_REVOLUTION)))
            if not (TILT_UPPER_BOUND >= self.dxl_goal_position_tilt >= TILT_LOWER_BOUND):
                return
        #print(self.dxl_goal_position)
        self.__rotate_Motor(id) 

    def pan_To_Angle(self, angle):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL_PAN_ID, ADDR_PRESENT_POSITION)
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        prev_angle = int(float(dxl_present_position % DXL_MAXIMUM_POSITION_VALUE) / float(DXL_MAXIMUM_POSITION_VALUE) * FULL_REVOLUTION) 
        displacement = (angle-prev_angle)
        
        if dxl_present_position == PAN_UPPER_BOUND and not (displacement < 0):
            return

        if dxl_present_position == PAN_LOWER_BOUND and not (displacement > 0):
            return

        if abs(displacement) > JUMP_THRESHOLD:
            return

        #print(displacement)
        self.rotate_Degrees(int(displacement), DXL_PAN_ID)
    

    def tilt_To_Angle(self, angle):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL_TILT_ID, ADDR_PRESENT_POSITION)

       
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        prev_angle = int(float(dxl_present_position % DXL_MAXIMUM_POSITION_VALUE) / float(DXL_MAXIMUM_POSITION_VALUE) * FULL_REVOLUTION) 
        displacement = (angle-prev_angle)

        if abs(displacement) > JUMP_THRESHOLD :
            return

        #print(displacement)
        self.rotate_Degrees(int(displacement), DXL_TILT_ID)
        
