#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Int32MultiArray
from adafruit_servokit import ServoKit

#make a const variable for int servos = 4
myKit = ServoKit(channels=16) ##there are 16 channels on the board

# outdated
# Setting the given servo number (based on the channels) to a given angle
# Assumes all given angles are the same from publisher, and all servos are same model
def set_all_servo_angle(tupleOfSerAngle, previousPosition): 
    intSerAngle = int(tupleOfSerAngle)
    intPrevPos = int(previousPosition)
    if (tupleOfSerAngle > previousPosition):
        for i in range(intPrevPos, intSerAngle):
            for j in range(0, 4):
                myKit.servo[j].angle = i
                time.sleep(0.01)
    else:
        for i in range(intPrevPos, intSerAngle, -1):
            for j in range(0, 4):
                myKit.servo[j].angle = i
                time.sleep(0.01)

# Reading the message given
def servo_callback(msg):
    checks = True
    maxLength = len(msg.data)-1
    for i in range (0, 4): # 4 servos
        servoId = i
        servoAngle = msg.data[i]
        #rospy.loginfo("Received servo number: "+str(servoId)+". Received angle: "+str(servoAngle))
        if ((servoAngle > 85) or (servoAngle < 0)): #subject to change based on servo
            rospy.loginfo("Cannot turn servo over "+str(servoAngle)+" degrees")
            checks = False
    if (checks):
        arrayInput = list(msg.data)
        for i in range(0, int(len(arrayInput)/2)): # 4 servos
            rospy.loginfo("Servo"+str(i)+"Setting angle: "+str(msg.data[i])+"; Previous angle: "+str(msg.data[i+4]))
        set_angles(arrayInput)

#testing
def set_angles(arr):
    for i in range(4):
        myKit.servo[i].angle = arr[i]


# sets servo at whatever angles it is given.
# this doesnt work, make sure do redo this, because it went over max range
# we might even want to do scaling
def smooth_set_angles(arr):
    scaling = []
    closest = 85 #not possible
    maxAngle = 0 #stores index of max angle
    for i in range (0,4):
        if closest-arr[i+4] != 0 and abs(arr[i] - arr[i+4]) > abs(closest - arr[i+4]): 
            closest = arr[i]
        if arr[i] > maxAngle:
            maxAngle = i
    if closest <= 0 or closest > 85:
        closest = 1
    for i in range (0,4):
        if int((arr[i]-arr[i+4])/abs(closest-arr[i+4])) <= 1:
            scaling.append(1)
        else:
            scaling.append(int((arr[i]-arr[i+4])/abs(closest-arr[i+4])))
    while arr[maxAngle] > arr[i+4]:
        for j in range(0, 4):
                arr[i] = arr[i]+scaling[i]
                myKit.servo[j].angle = arr[i]
                time.sleep(0.01)
        

# Basic ROS subscriber
def servo_subscriber():
    rospy.init_node('servo_control', anonymous=True)
    rospy.Subscriber('/servo/command', Int32MultiArray, servo_callback)
    rospy.spin() #yields when no callbacks (makes sure python doesn't quit)


# Running Main method which creates the ROS node subscriber
if __name__ == '__main__':
    servo_subscriber()