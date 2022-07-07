
# Dynamixel Wrapper code taken from DYNAMIXEL protocol 2 exmaple
# angles must be in degrees 

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
    
from dynamixel_sdk import * # Uses Dynamixel SDK library

MY_DXL = 'X_SERIES'

ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095     # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 57600

ADDR_OPERATING_MODE = 11
ADDR_GOAL_VELOCITY = 104


OPERATING_MODE              = 4     # Extended Position Control Mode
TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 10    # Dynamixel moving status threshold

DXL_ID                      = 1
PROTOCOL_VERSION            = 2.0

FULL_REVOLUTION             = 360

class Dynamixel_Servo:
    def __init__(self, port):
        self.port = port
        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.dxl_goal_position = 0
    
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

        
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        

        # Switch to Extended Posiiton Control Mode
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_OPERATING_MODE, OPERATING_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been set to extended position control mode")


        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")
        
        # set intial position
        if (MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        else:
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        
        self.dxl_goal_position = dxl_present_position


    def __rotate_Motor(self, angle):
        # print("Press any key to continue! (or press ESC to quit!)")
        # if getch() == chr(0x1b):
        #     quit()

        # Write goal position
        if (MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID, ADDR_GOAL_POSITION, self.dxl_goal_position)
        else:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, ADDR_GOAL_POSITION, self.dxl_goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        while 1:
            # Read present position
            if (MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL_ID, ADDR_PRESENT_POSITION)
            else:
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            currAngle = ((dxl_present_position * FULL_REVOLUTION) // DXL_MAXIMUM_POSITION_VALUE) % FULL_REVOLUTION
            #print("[ID:%03d] GoalAngle:%03d  PresAngle:%03d" % (DXL_ID, angle % FULL_REVOLUTION, currAngle))
            if not abs(self.dxl_goal_position  - (dxl_present_position) ) > DXL_MOVING_STATUS_THRESHOLD:
                return
    

    def rotate_Degrees(self, degrees):
        self.dxl_goal_position = (self.dxl_goal_position + int(float(degrees) * float(DXL_MAXIMUM_POSITION_VALUE) / float(FULL_REVOLUTION)))
        self.__rotate_Motor(self.dxl_goal_position) 

    def rotate_To_Angle(self, angle):
        if (MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        else:
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        prev_angle = int(float(dxl_present_position % DXL_MAXIMUM_POSITION_VALUE) / float(DXL_MAXIMUM_POSITION_VALUE) * FULL_REVOLUTION) 
        displacement = (angle-prev_angle)

        if displacement < 0 and (displacement%FULL_REVOLUTION) < -1 * displacement:
            displacement = (displacement%FULL_REVOLUTION)
        elif FULL_REVOLUTION - displacement < displacement:
            displacement = -1 * (FULL_REVOLUTION - displacement)
    
        self.rotate_Degrees(int(displacement))
        
    





