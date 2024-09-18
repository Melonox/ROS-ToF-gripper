#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
from adafruit_servokit import ServoKit

# This servo class turns the physical servos connected to a PWM driver board
class Servo:
    maxAngle = 85 #tested for safety or else the metal rod pops out
    myKit = ServoKit(channels=16) #16 channels on the board

    # creating a servo object and setting the amount of servos connected to the PWM driver
    def __init__(self, numServos):
        self.numServos = numServos

    # String representation of the object
    def __str__(self):
        return f"There are {self.numServos} servos, with maximum angle turning of {self.maxAngle}"

    # Turns all servos on the PWM driver given an array of each angle
    def set_angles(self, arrOfAngles):
        for i in range(self.numServos):
            self.myKit.servo[i].angle = arrOfAngles[i]

    #this also depends on what I want the published message to be
    #for now it will only contain the angles we want to set it at
    def servo_callback(self, msg):
        checks = True
        for i in range (0, len(msg.data)):
            servoAngle = msg.data[i]
            ##rospy.loginfo(list(msg.data))
            if ((servoAngle > self.maxAngle) or (servoAngle < 0)):
                rospy.loginfo("Cannot turn servo to "+str(servoAngle)+" degrees")
                checks = False
        if checks:
            arrayInput = list(msg.data)
            for i in range(0, len(arrayInput)):
                rospy.loginfo("Servo "+str(i)+" Setting angle: "+str(msg.data[i]))
            self.set_angles(arrayInput)

    # ROS subscriber node to get the data from our filter
    def turn(self):
        rospy.init_node('servo_control', anonymous=True)
        rospy.Subscriber('/servo/command', Int32MultiArray, self.servo_callback)
        rospy.spin() #yields when no callbacks (makes sure python doesn't quit)


# running the main method
if __name__ == '__main__':
    servo = Servo(4)
    servo.turn()