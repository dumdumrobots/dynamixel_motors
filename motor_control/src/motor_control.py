#!/usr/bin/env python3

# --- Import libraries
import os,rospy
import numpy as np

from std_msgs.msg import Int32MultiArray, Bool
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
    pwm_desired = np.array([400,200]) # --- Desired Position 
    DXL_pwm_present = np.array([0,0]) # --- Current Position

    # --- General Variables
    DXL_ID = np.array([0,1])

    # --- Addresses EEPROM
    ADDR_OP_MODE = 11
    ADDR_PWM_LIMIT = 36

    # --- Addresses RAM
    ADDR_TORQUE_EN = 64
    ADDR_LED_EN = 65 
    ADDR_GOAL_PWM = 100
    ADDR_PRESENT_PWM = 124

    # --- Byte Length
    LEN_GOAL_PWM = 2
    LEN_PRESENT_PWM = 2


    # --- General Settings
    PROT_VR = 2.0
    BRATE = 57600
    DEVICE = '/dev/ttyUSB0'

    portHandler = PortHandler(DEVICE)
    packetHandler = PacketHandler(PROT_VR)

    # --- Sync write and read instances
    pwm_groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_PWM, LEN_GOAL_PWM)
    pwm_groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_PWM, LEN_PRESENT_PWM)



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
        self.pwmSubscriber = rospy.Subscriber("pwm_goal_value",Int32MultiArray, self.update_goal_pwm)
        rospy.sleep(0.005)

        self.pwmPublisher = rospy.Publisher("pwm_present_value",Int32MultiArray, queue_size=10)
        rospy.sleep(0.005)


        # --- Set up Servomotors 
        for ID in self.DXL_ID:

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_LED_EN, True)
            
            # --- Define PWM Control / -885(-100%) to 885(100%) at 0.113% per unit 
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_OP_MODE, 16)
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_PWM_LIMIT, 885) 

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


            # --- Storage parameters for Present PWM
            
                        


    def update_goal_pwm(self,msg):

        data_array = msg.data # --- Int32MultiArray data

        if len(data_array) == len(self.DXL_ID):
            self.pwm_desired = np.array(data_array)

        else:
            print("Invalid PWM array, try again.\n")



    def publish_present_pwm(self):

        pub_array = Int32MultiArray()
        pub_array.data = self.DXL_pwm_present.tolist()

        self.pwmPublisher.publish(pub_array)



    def set_goal_pwm(self):

        for ID in self.DXL_ID:

            param_goal_pwm = [DXL_LOBYTE(self.pwm_desired[ID]), DXL_HIBYTE(self.pwm_desired[ID])]
            
            print(param_goal_pwm)

            dxl_addparam_result = self.pwm_groupSyncWrite.addParam(ID, param_goal_pwm)

            if dxl_addparam_result != True:

                print("Failed to set Goal PWM for Motor ID: " + str(ID) + ". groupSyncWrite addparam failed.\n")
                self.shutdown()

            else: 
                print("Goal PWM set to Motor ID: " + str(ID) + " to " + str(self.pwm_desired[ID] * 0.113) + "%." + "\n")


        dxl_comm_result = self.pwm_groupSyncWrite.txPacket()

        if dxl_comm_result != COMM_SUCCESS:

            print("Failed to group write Goal PWM.\n")
            self.shutdown()

        self.pwm_groupSyncWrite.clearParam()



    def read_present_pwm(self):

        for ID in self.DXL_ID:

            dxl_addparam_result = self.pwm_groupSyncRead.addParam(ID)

            if dxl_addparam_result != True:
                print("Failed setup for Motor ID: " + str(ID) + ". groupSyncRead addparam failed.\n")
                self.shutdown()

        dxl_comm_result = self.pwm_groupSyncRead.txRxPacket()

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            print("Failed to group read Present PWM.\n")
            self.shutdown()

        for ID in self.DXL_ID:

            dxl_getdata_result = self.pwm_groupSyncRead.isAvailable(ID, self.ADDR_PRESENT_PWM, self.LEN_PRESENT_PWM)

            if dxl_getdata_result != True:
                print("Failed to read Present PWM from Motor ID: " + str(ID) + ". groupSyncRead getdata failed.\n")
                self.shutdown()

            self.DXL_pwm_present[ID] = self.pwm_groupSyncRead.getData(ID, self.ADDR_PRESENT_PWM, self.LEN_PRESENT_PWM)

        self.pwm_groupSyncRead.clearParam()



    def shutdown(self):

        for ID in self.DXL_ID:
            self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_LED_EN, False)
            self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_TORQUE_EN, False)
            print("Shutting down Motor ID: " + str(ID) + "\n")
        
        self.portHandler.closePort()
        getch()
        quit()



def main():

    rospy.init_node("pwm_node")
    freq = 10
    rate = rospy.Rate(freq)

    robot = Robot()

    while not rospy.is_shutdown():

        robot.set_goal_pwm()
        robot.read_present_pwm()
        robot.publish_present_pwm()

    rospy.on_shutdown(robot.shutdown())


if __name__ == '__main__':
    main()