#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, UInt16MultiArray
from adafruit_servokit import ServoKit
from simple_pid import PID
import numpy as np
import time

# This servo class turns the physical servos connected to a PWM driver board
class Servo:

    # creating a servo object and setting the amount of servos connected to the PWM driver
    def __init__(self, numServos, mode="nervous", target_distance=4):
        self.numServos = numServos
        self.mode = mode # nervous or follow
        self.maxAngle = 85 #tested for safety or else the metal rod pops out
        self.myKit = ServoKit(channels=16) #16 channels on the board
        
        # self.test = np.array([[0, 100],
        #                       [5, 97],
        #                       [10, 93],
        #                       [15, 89],
        #                       [20, 84],
        #                       [25, 89],
        #                       [30, 75],
        #                       [35, 72],
        #                       [40, 68],
        #                       [45, 64],
        #                       [50, 60],
        #                       [55, 56],
        #                       [60, 51],
        #                       [65, 47],
        #                       [70, 43],
        #                       [75, 39],
        #                       [80, 34],
        #                       [85, 30]])
        self.test = np.array([[0, 109],
                              [5, 106],
                              [10, 102],
                              [15, 98],
                              [20, 94],
                              [25, 90],
                              [30, 86],
                              [35, 81],
                              [40, 78],
                              [45, 73],
                              [50, 69],
                              [55, 66],
                              [60, 61],
                              [65, 58],
                              [70, 54],
                              [75, 50],
                              [80, 46],
                              [85, 43]])
        
        self.target_distance = target_distance
        self.target_angle = self.test[(np.abs(self.test[:,1] - self.target_distance * 10)).argmin(), 0]
        self.pid = []
        self.servos = [0] * numServos
        for i in range(self.numServos):
            self.myKit.servo[i].angle = self.servos[i]
            self.pid.append(PID(Kp=1.0, Ki=0., Kd=0., setpoint=self.target_distance))
            self.pid[i].output_limits = (0, 85)

        time.sleep(2)

        rospy.Subscriber('/servo/command', UInt16MultiArray, self.servo_callback2, queue_size=1, buff_size=2**24)
        self.pub = rospy.Publisher('/servo/input', UInt16MultiArray, queue_size=1)
        

    # String representation of the object
    def __str__(self):
        return f"There are {self.numServos} servos, with maximum angle turning of {self.maxAngle}"

    # Turns all servos on the PWM driver given an array of each angle
    def set_angles(self, arrOfAngles):
        for i in range(self.numServos):
            self.myKit.servo[i].angle = arrOfAngles[i]

    #converting array of distance to array of angles corresponding to each servo
    def convertDistancetoAngle(self, arrayInput):
        arrayInput = [element/10 for element in arrayInput]
        # print(f'arrayInput {arrayInput}')
        # import pdb; pdb.set_trace()
        distance = np.zeros(4)
        for i in range(len(arrayInput)): #converting distance to degree
            if (arrayInput[i] < 0.5): #minimum before the data is funky
                distance[i] = 0
            # elif (arrayInput[i] > 6.75) and self.mode == "nervous": #maximum range of fingers
            #     distance[i] = 0
            elif (arrayInput[i] > 6.75): #  and self.mode == "follow" maximum range of fingers
                distance[i] = self.maxAngle  
            else:
                distance[i] = int(arrayInput[i] * (self.maxAngle/6.75))
        #print(f'distance {distance} arrayInput {arrayInput}')
        return distance

    #this also depends on what I want the published message to be
    #for now it will only contain the angles we want to set it at
    def servo_callback(self, msg):

        arrayInput = msg.data
        # time.sleep(0.1)
        for i in range(self.numServos):
            index = (np.abs(self.test[:,1] - arrayInput[i])).argmin()
            # index = (np.abs(self.test[:,1] - self.target_distance * 10)).argmin()
            # import pdb; pdb.set_trace()
            # self.servos[i] = self.test[index, 0]
            # control = self.pid[i](msg.data[i]/10) # cm
            control = self.pid[i](arrayInput[i])
            self.myKit.servo[i].angle = max(0 , min(85, control))
            
            # self.servos[i] = self.servos[i] - control
            # self.servos[i] = self.convertDistancetoAngle(arrayInput)[i] #- control
        # import pdb; pdb.set_trace()
            # self.myKit.servo[i].angle = max(0 , min(85, self.servos[i] - control))
            # self.myKit.servo[i].angle = max(0 , min(85, self.test[index, 0]))
            # self.myKit.servo[i].angle = self.convertDistancetoAngle(arrayInput)[i]
        # print(self.servos, msg.data[i]/10, control, max(0 , min(85, self.servos[i] - control)), self.test[index, 0])
        print(control)
            # print(self.convertDistancetoAngle(arrayInput)[i])

        # index = self.test[(np.abs(self.test[1] - arrayInput[0])).argmin()][0]
        # self.servo_0 = self.convertDistancetoAngle(arrayInput)[0] - control
        # self.servo_0 = self.servo_0 - control #this is wrong and is what is causing the delay

        # self.pub.publish(UInt16MultiArray(data=[int(self.servo_0), int(control), int(msg.data[0]/10)]))


        # print(max(0 , min(85, arrayInput[0]/10*85/6.75)), arrayInput[0]/10*85/6.75)
        # print(control, arrayInput[0]/10, self.servo_0, max(0 , min(85, self.servo_0)))
        # import pdb; pdb.set_trace()
        # distance = self.convertDistancetoAngle(arrayInput)
        # print(f'distance {distance} arrayInput {arrayInput}')
        # for i in range(0, len(distance)):
        #     rospy.loginfo("Servo "+str(i)+" Setting angle: "+str(distance[i]))
        # self.set_angles(distance)

        # Old code after Josh left
        # start = time.time()
        # arrayInput = msg.data
        # control = self.pid(msg.data[0]/10) # cm
        # index = self.test[(np.abs(self.test[1] - arrayInput[0])).argmin()][0]
        # self.servo_0 = self.convertDistancetoAngle(arrayInput)[0] - control
        # # self.servo_0 = self.servo_0 - control #this is wrong and is what is causing the delay
        # self.myKit.servo[0].angle = max(0 , min(85, self.servo_0))
        # # self.myKit.servo[1].angle = max(0 , min(85, arrayInput[1]/10*85/6.75))
        # # self.myKit.servo[2].angle = max(0 , min(85, arrayInput[2]/10*85/6.75))
        # # self.myKit.servo[3].angle = max(0 , min(85, arrayInput[3]/10*85/6.75))
        # # index = self.test[(np.abs(self.test[1] - arrayInput[0])).argmin()][0]
        # # import pdb; pdb.set_trace()
        # self.pub.publish(UInt16MultiArray(data=[int(self.servo_0), int(control), int(msg.data[0]/10)]))
        # end = time.time() - start

        # print(self.servo_0, control, arrayInput, end)

    def servo_callback2(self, msg):
        current_distance = msg.data[0] / 10
        print(current_distance)
        if current_distance < self.target_distance - 0.1:
            self.servos[0] = 0
            self.myKit.servo[0].angle = self.servos[0]
        elif current_distance > self.target_distance + 0.1:
            self.servos[0] = 85
            self.myKit.servo[0].angle = self.servos[0]

        time.sleep(0.1)
        



# running the main method
if __name__ == '__main__':
    rospy.init_node('servo_control', anonymous=True)
    num_servos = rospy.get_param('num_servos', 1)
    mode = rospy.get_param('mode', "nervous")
    target_distance = rospy.get_param('target_distance', 4)
    servo = Servo(num_servos, mode, target_distance)
    rospy.spin()