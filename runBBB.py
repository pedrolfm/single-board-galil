#!/usr/bin/env python

import rospy
import numpy
import serial
import time
import sys

from std_msgs.msg import Int16MultiArray
from ros_igtl_bridge.msg import igtlpoint, igtltransform, igtlstring
from utils import Target

#states:
IDLE = 0
INIT = 1
TARGET = 2
MOVE = 3

PIEZO_INIT_VERTICAL = 0
PIEZO_INIT_HORIZONTAL = 0
SHARP = chr(35)

# relationship between encoder counts and mm:
US_MM_2_COUNT = 2000.0/2.5349
PE_MM_2_COUNT = 500 #5500.0/16.51

class Controller:

    def __init__(self):

        # ROS Topics:
        rospy.Subscriber('IGTL_STRING_IN', igtlstring, self.callbackString)
        rospy.Subscriber('IGTL_TRANSFORM_IN', igtltransform, self.callbackTransformation)
        self.pub1 = rospy.Publisher('IGTL_STRING_OUT', igtlstring, queue_size=10)
        self.pub2 = rospy.Publisher('IGTL_STRING_OUT', igtlstring, queue_size=10)
        self.motors = rospy.Publisher('IGTL_STRING_OUT', igtlstring, queue_size=10)
        self.galilStatus = rospy.Publisher('IGTL_STRING_OUT', igtlstring, queue_size=10)

        rospy.init_node('talker', anonymous=True)

        # Define the variables for openigtlink
        self.TransferData1 = igtlstring()
        self.TransferData1.name = "statusTarget"
        self.TransferData2 = igtlstring()
        self.TransferData2.name = "statusZ-Frame"
        self.motorsData = igtlstring()
        self.motorsData.name = "motorPosition"
        self.galilStatusData = igtlstring()
        self.galilStatusData.name = "status"

        #Variables:
        self.status = 0
        self.CartesianPositionA = 0
        self.CartesianPositionB = 0
        self.OrientationA = 0
        self.OrientationB = 0
        self.state = IDLE
        self.MotorsReady = 0
        self.targetRAS = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')
        self.targetRAS_angle = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')
        self.targetRobot = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')
        self.zTransReady = False
        self.zTrans = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')
        self.target = Target()
        self.connectionStatus = False
        self.save_position_A = 0 
        self.save_position_B = 0
        self.save_position_C = 0
        self.save_position_D = 0


    def mm2counts_us_motor(self,distance):
        return int(US_MM_2_COUNT*distance)

    def counts2mm_us_motor(self,counts):
        return int((1.0/US_MM_2_COUNT)*counts)

    def mm2counts_piezomotor(self,distance):
        return int(PE_MM_2_COUNT*distance)

    def counts2mm_piezomotor(self,counts):
        return int((1.0/PE_MM_2_COUNT)*counts)

######################################################
# ROS topic callbacks:
######################################################
    def callbackString(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.name)
        if data.name == "INIT":
            self.state = INIT
            self.initCondition = data.data[4]+data.data[5]
            print(self.initCondition)
        elif data.data == "MOVE":
            self.state = MOVE
        elif data.data == "SERIAL":
            self.reconnect()
        else:
            self.state = IDLE
            rospy.loginfo('Invalid message, returning to IDLE state')

    def callbackTransformation(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard ' + data.name)
        if data.name == "zFrameTransformation":
            pos = numpy.array([data.transform.translation.x,data.transform.translation.y,data.transform.translation.z])
            quat = numpy.array([data.transform.rotation.w, data.transform.rotation.x,data.transform.rotation.y,data.transform.rotation.z])
            self.zTrans = self.quaternion2ht(quat,pos)
            print(self.zTrans)
            self.zTransReady = True
        elif data.name == "targetTransformation":
            self.state = TARGET
            pos = numpy.array([data.transform.translation.x,data.transform.translation.y,data.transform.translation.z])
            quat = numpy.array([data.transform.rotation.w, data.transform.rotation.x,data.transform.rotation.y,data.transform.rotation.z])
            self.targetRAS = self.quaternion2ht(quat,pos)
            self.target.set_target_RAS(self.targetRAS)
            self.targetRAS_angle = self.quaternion2ht(quat, pos)
            self.target.set_target_RAS_angle(self.targetRAS_angle)
        elif data.name == "angleTransformation":
            self.state = TARGET
            pos = numpy.array(
                [data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
            quat = numpy.array([data.transform.rotation.w, data.transform.rotation.x, data.transform.rotation.y,
                                data.transform.rotation.z])
            self.targetRAS_angle = self.quaternion2ht(quat, pos)
            self.target.set_target_RAS_angle(self.targetRAS_angle)
        else:
            self.state = IDLE
            rospy.loginfo('Invalid message, returning to IDLE state')

##################################################################



    def quaternion2ht(self,quat,pos):
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


    def open_connection(self):
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)  # open serial port
            #self.ser.open()
            self.connectionStatus = True
            print('connection open')
            return 1
        except:
            try:
               self.ser = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=1)  # open serial port
               #self.ser.open()
               self.connectionStatus = True
               print('connection open')
               return 1
            except:
               self.connectionStatus = False
               rospy.loginfo("\n*** could not open the serial communication ***\n")
               return 0


    def reconnect(self):
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)  # open serial port
            #self.ser.open()
            self.connectionStatus = True
            print('connection open')
           # return 1
        except:
            try:
               self.ser = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=1)  # open serial port
               #self.ser.open()
               self.connectionStatus = True
               print('connection open')
              # return 1
            except:
               self.connectionStatus = False
               rospy.loginfo("\n*** could not open the serial communication ***\n")
               return 0
        time.sleep(0.5)
        self.ser.write(str("DPA=%d\r" % self.save_position_A))
        time.sleep(0.1)
        self.ser.write(str("DPB=%d\r" % self.save_position_B))
        time.sleep(0.1)
        self.ser.write(str("DPC=%d\r" % self.save_position_C))
        time.sleep(0.1)
        self.ser.write(str("DPD=%d\r" % self.save_position_D))
        time.sleep(0.1)
        return 1
    
    def getFTSWstatus(self):
        # This function asks Galil the footswitch status
        try:
            self.ser.flushInput()
            time.sleep(0.1)
            self.ser.write(str("MG @IN[1];"))
            time.sleep(0.1)
            bytesToRead = self.ser.inWaiting()
            print(bytesToRead)
            data_temp = self.ser.read(bytesToRead-4)
            print("what do we receive?")
            print(data_temp)
        except:
            print("*** could not send command ***")
            self.status = 0
            self.galilStatusData.data = "No Galil connection"
            return
        print("===")
        print(data_temp)
        if (float(data_temp) == 1):
            self.status = 1
            self.galilStatusData.data = "FTSW OFF"
        elif (float(data_temp) == 0):
            self.status = 2
            self.galilStatusData.data = "FTSW ON"
        else:
            self.status = 3
            self.galilStatusData.data = "Check conection"
        print(float(data_temp))
        return 

    def getMotorPosition(self):
        try:
            self.ser.flushInput()
            time.sleep(0.5)
            self.ser.write(str("TP;"))
            time.sleep(0.1)
            bytesToRead = self.ser.inWaiting()
            data_temp = self.ser.read(bytesToRead-3)
            print(data_temp)
        except:
            print("*** could not send command ***")
            self.status = 0
            return str(0)
        return data_temp
    

    def send_movement_in_counts(self,X,Channel):
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

    def exec_motion(self):
        try:
            self.ser.write(str("BG \r"))
            time.sleep(0.01)
            self.ser.write(str("PR 0,0,0,0 \r"))
        except:
            print("*** could not send command ***")
            return 0

    #TODO:
    def check_controller(self):
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
            self.ser.write(str("DP%s=0\r" % Channel))
            self.ser.write(str("PT%s=1\r" % Channel))
            self.ser.write(str("SH;"))
            self.AbsoluteMode = True
            return 1
        except:
            print("*** could not set Absolute mode ***")
            return 0

    def SendAbsolutePosition(self,Channel,X):
        try:
            if self.AbsoluteMode:
                self.ser.write(str("SH;"))
                time.sleep(0.01)
                self.ser.write(str("PT%s=1;" % Channel))
                time.sleep(0.01)
                self.ser.write(str("PA%s=%d;" % (Channel,X)))
                return 1
            else:
                rospy.loginfo("*** PA not available ***")
                return 0
        except:
            rospy.loginfo("*** could not send command ***")
            return 0


    def init_motors(self):
        try:
            if self.initCondition == "US":
                self.init_us_motors()
            # First horizontal, second vertical
            elif self.initCondition == "LT":
                self.init_piezo("HPELF","HPEUP")
            elif self.initCondition == "LC":
                self.init_piezo("HPELF","HPEVC")
            elif self.initCondition == "LB":
                self.init_piezo("HPELF","HPEDW")
            elif self.initCondition == "RT":
                self.init_piezo("HPERT","HPEUP")
            elif self.initCondition == "RC":
                self.init_piezo("HPERT","HPEVC")
            elif self.initCondition == "RB":
                self.init_piezo("HPERT","HPEDW")
            elif self.initCondition == "CT":
                self.init_piezo("HPEHC","HPEUP")
            elif self.initCondition == "CC":
                self.init_piezo("HPEHC","HPEVC")
            elif self.initCondition == "CB":
                self.init_piezo("HPEHC","HPEDW")
            return 1
        except:
            rospy.loginfo("*** could not initialize motors ***")
            return 0


    def init_us_motors(self):
        try:
            rospy.loginfo("XQ "+SHARP+"HUSA\n")
            self.ser.write("XQ "+SHARP+"HUSA;")
            time.sleep(20.0)
            rospy.loginfo("XQ "+SHARP+"HUSB\n")
            self.ser.write("XQ "+SHARP+"HUSB;")
            rospy.loginfo("DONE INIT...")
        except:
            rospy.loginfo("NO INIT...")
        return
    
    def init_piezo(self,positionHorizontal,positionVertical):
        try:
            rospy.loginfo(str("XQ "+SHARP+positionVertical+",0;"))
            self.ser.write(str("XQ "+SHARP+positionVertical+",0;")) 
            time.sleep(6.0)
            rospy.loginfo(str("XQ "+SHARP+positionHorizontal+",1;"))
            self.ser.write(str("XQ "+SHARP+positionHorizontal+",1;"))
            time.sleep(3.0)
            rospy.loginfo("*** initialization done***")
            return 1
        except:
            rospy.loginfo("*** could not initialize Piezo motors ***")
            return 0


    def define_target(self):
        #Get target (x,y,z)
        if self.target.ht_RAS_target[0,3] != 0.0 and self.target.ht_RAS_target[1,3] != 0.0 and self.target.ht_RAS_target[2,3] != 0.0:
            self.target.define_position_piezo()
            return self.target.define_target_robot(self.zTrans)
        else:
            rospy.loginfo( "Please check the target location")
            return 0


def myhook():
    print('time to go...')
    recsys.exit()

def main():

    rospy.loginfo('Welcome to the Smart Template controller\n')

    control = Controller()
    time.sleep(3)

    control.state = IDLE
    control.open_connection()
    control.SetAbsoluteMotion('A')
    control.SetAbsoluteMotion('B')
    control.SetAbsoluteMotion('C')
    control.SetAbsoluteMotion('D')

    #TODO: check if it is necessary
    if control.check_controller()==0:
        rospy.loginfo('Check Galil controller setup\n')

    while 1: #not rospy.is_shutdown():

        while control.state == IDLE:
            print("test")
            strg_temp = "(%02f, %02f, %02f)mm - (%02f, %02f)rad" % (control.target.ht_RAS_target[0,3],control.target.ht_RAS_target[1,3],control.target.ht_RAS_target[2,3],control.target.phi,control.target.teta)            
            control.TransferData1.data = strg_temp
            control.pub1.publish(control.TransferData1)
            time.sleep(2)
#           strg_temp = "No Connection"+control.zTransReady+"-"
            control.TransferData2.data = "zFrame" + str(control.zTransReady)
            control.pub2.publish(control.TransferData2)

            #Load information
            strg_temp = control.getMotorPosition()
            control.motorsData.data = strg_temp #str(control.target.y) + "#" + str(control.target.x) + "#" + str(control.target.piezo[1]) + "#" + str(control.target.piezo[0])
            control.motors.publish(control.motorsData)
            control.getFTSWstatus()
            control.galilStatus.publish(control.galilStatusData)

        if control.state == INIT:

            if control.init_motors():
                control.SetAbsoluteMotion('A')
                control.SetAbsoluteMotion('B')
                control.SetAbsoluteMotion('C')
                control.SetAbsoluteMotion('D')
                control.MotorsReady = 1
                control.state = IDLE
            else:
                rospy.loginfo("US motor not ready")
                control.state = IDLE


        # IMPORTANT: due to the hardware implementation, the axis y is in oposite direction to the motor direction (channel A).
        # The same thing happens to channel C
        # Axis X is channel B
        # Axis Y is channel A


        if control.state == TARGET and control.zTransReady:
            if control.define_target():
                rospy.loginfo("Target set, waiting for command")
                rospy.loginfo("Movement axis A: %f mm -  %f counts" % (control.target.y,control.mm2counts_us_motor(control.target.y)))
                rospy.loginfo("Movement axis B: %f mm -  %f counts" % (control.target.x,control.mm2counts_us_motor(control.target.x)))
                rospy.loginfo("Movement axis C: %f mm -  %f counts" % (control.target.piezo[0],control.mm2counts_piezomotor(-control.target.piezo[0])))
                rospy.loginfo("Movement axis D: %f mm -  %f counts" % (control.target.piezo[1],control.mm2counts_piezomotor(control.target.piezo[1])))
                control.state = IDLE
                control.target.ready = True
                print(control.target.ready)
                print("out of the loop")
            else:
                rospy.loginfo("Check target location and try again")
                control.state = IDLE

        if control.state == MOVE and control.target.ready == True:
            print("target ready, start movement...")
            control.SendAbsolutePosition('C', control.mm2counts_piezomotor(-control.target.piezo[0]))
            time.sleep(0.01)
            control.SendAbsolutePosition('D', control.mm2counts_piezomotor(control.target.piezo[1]))
            time.sleep(0.01)
            control.SendAbsolutePosition('A', control.mm2counts_us_motor(control.target.y))
            time.sleep(0.01)
            control.SendAbsolutePosition('B', control.mm2counts_us_motor(control.target.x))
            time.sleep(0.01)

            # Save that in case Galil is shutdown
            control.save_position_A = control.mm2counts_us_motor(control.target.y)
            control.save_position_B = control.mm2counts_us_motor(control.target.x)
            control.save_position_C = control.mm2counts_piezomotor(-control.target.piezo[0])
            control.save_position_D = control.mm2counts_piezomotor(control.target.piezo[1])
            control.target.ready = False
            control.state = IDLE


if __name__ == '__main__':
    main()

 
