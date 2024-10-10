#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, UInt16MultiArray
from adafruit_servokit import ServoKit
from simple_pid import PID
import numpy as np
import time

import matplotlib.pyplot as plt

CM_PER_DEGREE = 0.375 # was 0.375

# This servo class turns the physical servos connected to a PWM driver board
class Servo:
    # creating a servo object and setting the amount of servos connected to the PWM driver
    def __init__(self, numServos, mode="nervous", target_distance=2):
        self.numServos = numServos
        self.mode = mode # nervous or follow
        self.maxAngle = 85 #tested for safety or else the metal rod pops out
        self.myKit = ServoKit(channels=16) #16 channels on the board
        self.target_distance = target_distance
        self.servos = [0] * numServos
        for i in range(self.numServos):
            self.myKit.servo[i].angle = self.servos[i]

        self.angle_error = np.zeros(self.numServos)
        self.servo_positions = np.zeros(self.numServos)

        time.sleep(2)
        
        # PID variables
        self.kp = 0.11
        self.ki = 0
        self.kd = 0.01
        self.integral = np.zeros(self.numServos)
        self.derivative = np.zeros(self.numServos)
        self.previous_error = [0]*4
        self.previous_time = time.time()
        rospy.Subscriber('/servo/command', UInt16MultiArray, self.servo_callback2, queue_size=1, buff_size=2**24)
        self.pub_position = rospy.Publisher('/servo/position', Float32MultiArray, queue_size=1)
        
    # String representation of the object
    def __str__(self):
        return f"There are {self.numServos} servos, with maximum angle turning of {self.maxAngle}"

    # Turns all servos on the PWM driver given an array of each angle
    def set_angles(self, arrOfAngles):
        for i in range(self.numServos):
            self.myKit.servo[i].angle = arrOfAngles[i]
    
    # ROS callback
    def servo_callback2(self, msg):
        self.current_time = time.time()
        self.elapsed_time = self.current_time - self.previous_time #I think its because the previous time doesn't change
        self.previous_time = self.current_time #added
        amount_of_servos = len(msg.data) #should equal self.numServos
        for i in range(amount_of_servos):
            error = msg.data[i] / 10 - self.target_distance 
            self.angle_error[i] = error / CM_PER_DEGREE 
        for i in range(amount_of_servos):
            # Calculate Integral #if you leave it on for a while reading the ceiling and then input an object, it decrements too slow
            self.integral[i] += self.angle_error[i] * self.elapsed_time
            # Calculate Derivative
            self.derivative[i] = (self.angle_error[i] - self.previous_error[i]) / self.elapsed_time if self.elapsed_time > 0 else 0
            self.previous_error[i] = self.angle_error[i] #added
            # PID
            adjustment = self.kp * self.angle_error[i] + self.ki * self.integral[i] + self.kd * self.derivative[i]
            current_servo_position = self.myKit.servo[i].angle
            #self.pub_position.publish(Float32MultiArray(data=[current_servo_position]))
            new_servo_position = current_servo_position + adjustment
            self.servo_positions[i] = max(0, min(85, new_servo_position))
        for i in range(amount_of_servos):
            self.myKit.servo[i].angle = self.servo_positions[i]
        time.sleep(0.01) 

# running the main method
if __name__ == '__main__':
    rospy.init_node('servo_control', anonymous=True)
    num_servos = rospy.get_param('num_servos', 4)
    mode = rospy.get_param('mode', "nervous")
    target_distance = rospy.get_param('target_distance', 2)
    servo = Servo(num_servos, mode, target_distance)
    rospy.spin()