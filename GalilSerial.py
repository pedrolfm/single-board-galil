import serial
import time
import rospy
from std_msgs.msg import Int16MultiArray
from ros_igtl_bridge.msg import igtlpoint, igtltransform
IDLE = 0
INIT = 1


class Controller:

    def __init__(self):

        rospy.Subscriber('IGTL_STRING_IN', igtlstring, self.callbackString)
        self.pub = rospy.Publisher('IGTL_STRING_OUT', igtlstring, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        # Define the variables
        self.TransferData = igtlstring
        self.CartesianPositionA = 0
        self.CartesianPositionB = 0
        self.OrientationA = 0
        self.OrientationB = 0
        self.state = ILDE
        self.MotorsReady = 0

    def callbackString(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        if data.data == "INIT":
            self.state = INIT
        else:
            self.state = ILDE
            rospy.loginfo('Invalid message, returning to IDLE state')


    def OpenConnection(self):
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)  # open serial port
            self.ser.open()
        except:
            rospy.loginfo("\n*** could not open the serial communication ***\n")
            return 0
        if self.ser.is_open():
            return 1
        else:
            rospy.loginfo("\n*** could not open the serial communication ***\n")
            return 0

    def SendMovementInCounts(self,X,Channel):
        try:
            self.ser.write(str("PR%s%d\r" % (Channel,X)))
        except:
            print("*** could not send command ***")
            return 0

        time.sleep(0.05)
        self.ser.write(str("PR%s?\r" % Channel))
        time.sleep(0.01)
        bytesToRead = self.ser.inWaiting()
        data_temp = self.ser.read(bytesToRead)
        #print(data_temp)
        if data_temp == X:
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
            self.ser.write(str("MT ?,?\r"))
            bytesToRead = self.ser.inWaiting()
            MT = self.ser.read(bytesToRead)
            time.sleep(0.1)
            self.ser.write(str("CE ?,?\r"))
            bytesToRead = self.ser.inWaiting()
            CE = self.ser.read(bytesToRead)
            time.sleep(0.1)
            if MT == "1.0, 1.0" and CE == "0, 0":
                return 1
            else:
                rospy.loginfo('Wrong motor or encoder configuration\n')

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

    def InitiUSmotors(self):
        try:
            self.ser.write(str("PR 1000,1000\r")) #Check the values depending on the hardware
            self.ser.write(str("BG \r"))
            time.sleep(1)
            LRA = 0
            LRB = 0
            while LRA==0 and LRB==0:
                self.ser.write(str("MG_LRA \r"))
                bytesToRead = self.ser.inWaiting()
                LRA = self.ser.read(bytesToRead)
                self.ser.write(str("MG_LRB \r"))
                bytesToRead = self.ser.inWaiting()
                LRB = self.ser.read(bytesToRead)
            self.ser.write(str("DPA 0\r"))
            self.ser.write(str("DPB 0\r"))
            return 1
        except:
            print("*** could not initialize motors ***")
            return 0




def main():
    rospy.loginfo('Welcome to the Smart Template controller\n')

    controller = Controller()
    time.sleep(3)

    controller.OpenConnection()

    if controller.CheckController()==0:
        rospy.loginfo('Check Galil controller setup\nClosing the software')
        controller.TransferData.data = "Wrong Galil Config"
        controller.pub.publish(TransferData)


    while controller.state != INIT:
        time.sleep(3)
        print("waiting...")

    if controller.InitiUSmotors():
        controller.MotorsReady = 1






    ser.close()
    print(ser.is_open)# close port

if __name__ == "__main__":
    main()