#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
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

    #converting array of distance to array of angles corresponding to each servo
    def convertDistancetoAngle(self, arrayInput):
        distance = []
        for i in range(len(arrayInput)): #converting distance to degree
            if (arrayInput[i] < 0.5): #minimum before the data is funky
                distance.append(0)
            elif (arrayInput[i] > 6.75): #maximum range of fingers
                distance.append(0)  
            else:
                distance.append(int(arrayInput[i] * (85/6.75)))
        return distance

    #this also depends on what I want the published message to be
    #for now it will only contain the angles we want to set it at
    def servo_callback(self, msg):
        arrayInput = list(msg.data)
        distance = self.convertDistancetoAngle(arrayInput)
        for i in range(0, len(distance)):
            rospy.loginfo("Servo "+str(i)+" Setting angle: "+str(distance[i]))
        self.set_angles(distance)

    # ROS subscriber node to get the data from our filter
    def turn(self):
        rospy.init_node('servo_control', anonymous=True)
        rospy.Subscriber('/servo/command', Float32MultiArray, self.servo_callback)
        rospy.spin() #yields when no callbacks (makes sure python doesn't quit)


# running the main method
if __name__ == '__main__':
    servo = Servo(4)
    servo.turn()