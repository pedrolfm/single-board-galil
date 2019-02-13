#!/usr/bin/env python

import rospy
import numpy
import serial
import time

from std_msgs.msg import Int16MultiArray
from ros_igtl_bridge.msg import igtlpoint, igtltransform, igtlstring
IDLE = 0
INIT = 1


class Controller:

    def __init__(self):

        rospy.Subscriber('IGTL_STRING_IN', igtlstring, self.callbackString)
        rospy.Subscriber('IGTL_TRANSFORM_IN', igtltransform, self.callbackTransformation)
        self.pub = rospy.Publisher('IGTL_STRING_OUT', igtlstring, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        # Define the variables
        self.TransferData = igtlstring
        self.CartesianPositionA = 0
        self.CartesianPositionB = 0
        self.OrientationA = 0
        self.OrientationB = 0
        self.state = IDLE
        self.MotorsReady = 0
        self.zTransReady = False
        self.zTrans = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')

####################CALLBACKS
    def callbackString(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        if data.data == "INIT":
            self.state = INIT
        else:
            self.state = IDLE
            rospy.loginfo('Invalid message, returning to IDLE state')

    def callbackTransformation(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard')
        if data.name == "zTrans":
            self.zTransReady = True
            pos = numpy.array([data.transform.translation.x,data.transform.translation.y,data.transform.translation.z])
            quat = numpy.array([data.transform.rotation.w, data.transform.rotation.x,data.transform.rotation.y,data.transform.rotation.z])
            self.zTrans = self.quaternion2HTrans(quat,pos)
            print(self.zTrans)
            print(data.transform.rotation)
        else:
            self.state = IDLE
            rospy.loginfo('Invalid message, returning to IDLE state')
#########################################

    def quaternion2HTrans(self,quat,pos):
        H = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')
        H[0, 3] = pos[0]
        H[1, 3] = pos[1]
        H[2, 3] = pos[2]

        H[0, 0] = 1.0-2.0*(quat[2]*quat[2]+quat[3]*quat[3])
        H[0, 1] = 2.0*(quat[1]*quat[2]-quat[0]*quat[3])
        H[0, 2] = 2.0*(quat[1]*quat[3]+quat[0]*quat[2])

        H[1, 0] = 2.0*(quat[1]*quat[2]+quat[0]*quat[3])
        H[1, 1] = 1.0-2.0*(quat[1]*quat[1]+quat[3]*quat[3])
        H[1, 2] = 2.0*(quat[2]*quat[3]-quat[0]*quat[1])

        H[2, 0] = 2.0*(quat[1]*quat[3]-quat[0]*quat[2])
        H[2, 1] = 2.0*(quat[2]*quat[3]+quat[0]*quat[1])
        H[2, 2] = 1.0-2.0*(quat[1]*quat[1]+quat[2]*quat[2])

        return H


    def OpenConnection(self):
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)  # open serial port
            #self.ser.open()
            return 1
        except:
            rospy.loginfo("\n*** could not open the serial communication ***\n")
            return 0
        if self.ser.is_open():
            return 1
        else:
            rospy.loginfo("\n*** could not open the serial fd  communication ***\n")
            return 0

    def SendMovementInCounts(self,X,Channel):
        try:
            self.ser.write(str("PR%s=%d\r" % (Channel,X)))
            time.sleep(0.1)
        except:
            print("*** could not send command ***")
            return 0

        self.ser.flushInput()
        time.sleep(0.05)
        self.ser.write(str("PR%s=?\r" % Channel))
        time.sleep(0.1)
        bytesToRead = self.ser.inWaiting()
        data_temp = self.ser.read(bytesToRead-3)
        if int(data_temp) == X:
            return 1
        else:
            print("*** could not properly send command ***")
            return 0

    def ExecMotion(self):
        try:
            self.ser.write(str("BG \r"))
            time.sleep(0.01)
            self.ser.write(str("PR 0,0,0,0 \r"))
        except:
            print("*** could not send command ***")
            return 0

    #TODO:
    def CheckController(self):
        try:
            self.ser.write(str("EO 0\r"))
            time.sleep(0.1)

            self.ser.flushInput()
            self.ser.write(str("MT ?\r"))
            time.sleep(0.1)
            MT1 = float(self.ser.read(4))

            self.ser.flushInput()
            self.ser.write(str("MT ,?\r"))
            time.sleep(0.1)
            MT2 = float(self.ser.read(4))

            self.ser.flushInput()
            self.ser.write(str("CE ?\r"))
            time.sleep(0.1)
            CE1 = float(self.ser.read(2))
  
            self.ser.flushInput()
            self.ser.write(str("CE ,?\r"))
            time.sleep(0.1)
            CE2 = float(self.ser.read(2))

            if MT1 == 1.0 and MT2 == 1.0 and CE1 == 0.0 and CE2 == 0.0:
                return 1
            else:
                rospy.loginfo('Wrong motor or encoder configuration\n')
                return 0

        except:
            rospy.loginfo("*** could not send command ***")
            return 0

    def SetAbsoluteMotion(self,Channel):
        try:
            self.ser.write(str("DP%s 0\r" % Channel))
            self.ser.write(str("PT%s 1\r" % Channel))
            self.AbsoluteMode = True
            return 1
        except:
            print("*** could not set Absolute mode ***")
            return 0

    def SendAbsolutePosition(self,Channel,X):
        try:
            if self.AbsoluteMode == True:
                self.ser.write(str("PA%s %d\r" % Channel,X))
                return 1
            else:
                print("*** PA not available ***")
                return 0

        except:
            print("*** could not send command ***")
            return 0

#TODO: All motors
    def InitiUSmotors(self):
        try:
            #self.ser.flushInput()
            #self.ser.write(str("MG _LRA\r"))
            #bytesToRead = self.ser.inWaiting()
            #LRA = self.ser.read(bytesToRead)
            #print(LRA)
            #print(float(LRA))
            print("aqui 1")
            self.ser.write(str("PR 1000,1000\r")) #Check the values depending on the hardware
            #self.ser.write(str("BG \r"))
            time.sleep(1)
            LRA = 1.0
            LRB = 1.0

            while LRA==1.0 or LRB==1.0:
                # Motor A
                self.ser.flushInput()
                time.sleep(0.01)
                self.ser.write(str("MG _LRA\r"))
                time.sleep(0.01)
                LRAstring = self.ser.read(4)
                LRA = float(LRAstring)
                # Motor B
                self.ser.flushInput()
                time.sleep(0.01)
                self.ser.write(str("MG _LRB\r"))
                time.sleep(0.01)
                LRBstring = self.ser.read(4)
                LRB = float(LRBstring)

            self.ser.write(str("DPA 0\r"))
            self.ser.write(str("DPB 0\r"))
            rospy.loginfo("*** initialization done***")
            return 1
        except:
            rospy.loginfo("*** could not initialize motors ***")
            return 0




def main():
    rospy.loginfo('Welcome to the Smart Template controller\n')

    controller = Controller()
    time.sleep(3)

    controller.OpenConnection()

    if controller.CheckController()==0:
        rospy.loginfo('Check Galil controller setup\nClosing the software')
        controller.TransferData.data = "Wrong Galil Config"
        controller.pub.publish(controller.TransferData)

    controller.SendMovementInCounts(35,"A")

    while controller.state != INIT:
        time.sleep(3)
        print("waiting...")

    print("recebeu!")


    if controller.InitiUSmotors():
        controller.MotorsReady = 1
    else:
        print("Not ready")






    ser.close()
    print(ser.is_open)# close port

if __name__ == '__main__':
    main()
