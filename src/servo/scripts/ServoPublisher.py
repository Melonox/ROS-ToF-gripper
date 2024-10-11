#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayDimension


#this is out of date
def servos():
    pub = rospy.Publisher('/servo/command', Int32MultiArray, queue_size=10)
    rospy.init_node('servo_control', anonymous=True)
    #rate = rospy.Rate(10)
    
    arrayMSG = Int32MultiArray()
    arrayMSG.layout.dim = [MultiArrayDimension(label='MutliOutput', size = 8, stride = 8)]
    arrayMSG.layout.data_offset = 0
    #only running once
    ##arrayMSG.data = [0, 0, 1, 0, 2, 0, 3, 0] 
    arrayMSG.data = [0, 0, 1, 0, 2, 0, 3, 0] #could change to a 2D array, but no need
    rospy.loginfo(arrayMSG)
    for i in range (0, len(arrayMSG.data), 2): ##divide by 2 because each servo pin is set at even indexes
        rospy.loginfo("Publishing servo msg: Servo "+str(arrayMSG.data[i])+"; Angle: "+str(arrayMSG.data[i+1]))
    pub.publish(arrayMSG)
    rospy.sleep(1)

    rospy.loginfo("Finishing setting angle: "+str(arrayMSG.data[1]))

if __name__ == '__main__':
    try:
        servos()
    except rospy.ROSInterruptException:
        pass