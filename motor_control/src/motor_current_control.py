#!/usr/bin/env python3

# --- Import libraries
import os,rospy
import numpy as np

from std_msgs.msg import Int32MultiArray, Float64MultiArray
from dynamixel_sdk import *


# --- Serial setup
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



# --- Define a Robot with multiple DYNAMIXEL motors. 
class Robot(object):

    # --- Joint Variables
    current_desired = np.array([0,0,0,0,0]) # --- Desired Current
    DXL_current_present = np.array([0,0,0,0,0]) # --- Present Current

    # --- General Variables
    DXL_ID = np.array([0,1,2,3,4])

    # --- Addresses EEPROM
    ADDR_OP_MODE = 11
    ADDR_CURR_LIMIT = 38

    # --- Addresses RAM
    ADDR_TORQUE_EN = 64
    ADDR_LED_EN = 65 
    ADDR_GOAL_CURR = 102
    ADDR_PRESENT_CURR = 126

    # --- Byte Length
    LEN_GOAL_CURR = 2
    LEN_PRESENT_CURR = 2


    # --- General Settings
    PROT_VR = 2.0
    BRATE = 1000000
    DEVICE = '/dev/ttyUSB0'

    portHandler = PortHandler(DEVICE)
    packetHandler = PacketHandler(PROT_VR)

    # --- Sync write and read instances
    curr_groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURR, LEN_GOAL_CURR)
    curr_groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURR, LEN_PRESENT_CURR)



    def __init__(self):

        # --- Configure port
        try:
            self.portHandler.openPort()
            print("\nSucceeded to open the port.\n")

        except:
            print("\nFailed to open the port.\n")
            getch()
            quit()

        try:
            self.portHandler.setBaudRate(self.BRATE)
            print("Baudrate changed to " + str(self.BRATE) + "\n")
        except:
            print("Failed to change the baudrate.\n")
            getch()
            quit()


        # --- ROS COMMS
        self.currSubscriber = rospy.Subscriber("curr_goal_value",Float64MultiArray, self.update_goal_current)
        rospy.sleep(0.005)

        self.currPublisher = rospy.Publisher("curr_present_value",Float64MultiArray, queue_size=10)
        rospy.sleep(0.005)


        # --- Set up Servomotors 
        for ID in self.DXL_ID:

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_LED_EN, True)
            
            # --- Define Current Control / 0~1193 at 2.69mA per unit /125 == 336.25 mA
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_OP_MODE, 0)
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_CURR_LIMIT, 125) 

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_TORQUE_EN, True)

            # --- Verify errors
            if dxl_comm_result != COMM_SUCCESS:

                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                print("Failed setup for Motor ID: " + str(ID) + "\n")
                self.shutdown()

            elif dxl_error != 0:

                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                print("Failed setup for Motor ID: " + str(ID) + "\n")
                self.shutdown()

            else:
                print("Motor ID: " + str(ID) + " ready to use!\n")



    def update_goal_current(self,msg):

        data_array = np.array(msg.data) * 1000/2.69 # --- Float64MultiArray

        if len(data_array) == len(self.DXL_ID):
            self.current_desired = data_array.astype(int)

        else:
            print("Invalid Current array, try again.\n")



    def publish_present_current(self):

        pub_array = Float64MultiArray()
        pub_array.data = (self.DXL_current_present * 2.69/1000.0).tolist()

        self.currPublisher.publish(pub_array)
    


    def set_goal_current(self):

        for ID in self.DXL_ID:

            param_goal_curr = [DXL_LOBYTE(self.current_desired[ID]), DXL_HIBYTE(self.current_desired[ID])]

            dxl_addparam_result = self.curr_groupSyncWrite.addParam(ID, param_goal_curr)

            if dxl_addparam_result != True:

                print("Failed to set Goal Current for Motor ID: " + str(ID) + ". groupSyncWrite addparam failed.\n")
                self.shutdown()

            else: 
                pass
                #print("Goal Current set to Motor ID: " + str(ID) + " to " + str(self.current_desired[ID] * 2.69) + "mA." + "\n")


        dxl_comm_result = self.curr_groupSyncWrite.txPacket()

        if dxl_comm_result != COMM_SUCCESS:

            print("Failed to group write Goal Current.\n")
            self.shutdown()

        self.curr_groupSyncWrite.clearParam()



    def read_present_current(self):

        for ID in self.DXL_ID:

            dxl_addparam_result = self.curr_groupSyncRead.addParam(ID)

            if dxl_addparam_result != True:
                print("Failed setup for Motor ID: " + str(ID) + ". groupSyncRead addparam failed.\n")
                self.shutdown()

        dxl_comm_result = self.curr_groupSyncRead.txRxPacket()

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print("Failed to group read Present Current.\n")
            self.shutdown()

        for ID in self.DXL_ID:

            dxl_getdata_result = self.curr_groupSyncRead.isAvailable(ID, self.ADDR_PRESENT_CURR, self.LEN_PRESENT_CURR)

            if dxl_getdata_result != True:
                print("Failed to read Present Current from Motor ID: " + str(ID) + ". groupSyncRead getdata failed.\n")
                self.shutdown()

            self.DXL_current_present[ID] = self.curr_groupSyncRead.getData(ID, self.ADDR_PRESENT_CURR, self.LEN_PRESENT_CURR)

        self.curr_groupSyncRead.clearParam()



    def shutdown(self):

        for ID in self.DXL_ID:
            self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_LED_EN, False)
            self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_TORQUE_EN, False)
            print("Shutting down Motor ID: " + str(ID) + "\n")
        
        self.portHandler.closePort()
        getch()
        quit()



def main():

    rospy.init_node("current_node")
    freq = 10
    rate = rospy.Rate(freq)

    robot = Robot()

    while not rospy.is_shutdown():

        robot.set_goal_current()
        robot.read_present_current()
        robot.publish_present_current()

    rospy.on_shutdown(robot.shutdown())


if __name__ == '__main__':
    main()