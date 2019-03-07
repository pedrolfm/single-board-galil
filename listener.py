#!/usr/bin/env python

import rospy
import gclib
from std_msgs.msg import Int16MultiArray
from ros_igtl_bridge.msg import igtlpoint, igtltransform, igtlstring


class galilCommand:
    def __init__(self):
        rospy.Subscriber('chatter', Int16MultiArray, self.callback)
        #rospy.Subscriber('IGTL_POINT_OUT', igtlpoint, self.callback)
        rospy.Subscriber('IGTL_STRING_IN', igtlstring, self.callback2)

        self.MotorCmd = []
        self.g = gclib.py()
        self.flag = False
        print('gclib version:', self.g.GVersion())
        try:
            self.g.GOpen('192.168.0.99 --direct -s ALL')
        except gclib.GclibError as e:
            print('Unexpected GclibError:', e) 

    def callback(self, data):
        print("aqui")
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        self.MotorCmd = data.data
        #self.flag = True

    def callback2(self, data):
        print("tste")
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        #self.flag = True

    def WriteMotorRP(self):
        try: 
            self.g.GCommand(('PR %d, %d')%(self.MotorCmd[0],self.MotorCmd[1]))
            print(self.g.GCommand('PR ?,?'))
            self.flag = False
        except gclib.GclibError as e:
            print('Unexpected GclibError:', e)

def listener():


    rospy.init_node('listener', anonymous=True)
    gc = galilCommand()
    print("perdendo tempo com Transform")
    #while not rospy.is_shutdown():
        #if gc.flag:
        #    gc.WriteMotorRP()

    rospy.spin()

if __name__ == '__main__':
    listener()
