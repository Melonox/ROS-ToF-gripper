#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
import statistics
import numpy as np

#both subscriber and publisher
class FilteringToF:
    numOfServo = 4
    arrayOfDistAngle = [0,0,0,0] #this is first an array of distances that changes to be angles
    sensorData = [] # this is data from sensor but will be modified to filtered array

    #both a subscriber and a publisher node
    def __init__(self):
        rospy.init_node('servo_control', anonymous=True)
        rospy.Subscriber('/sensor/data', Int32MultiArray, self.filter)
        self.pub = rospy.Publisher('/servo/command', Int32MultiArray, queue_size=10)
    
    #filter based off previous point
    # def feedbackFilter(self, x, a0, b1):
    #     y = np.zeros(x.size)
    #     for n in range(0, x.size):
    #         y[n] = a0 * x[n] - b1 * y[n-1]
    #     return y

    #divides into 4, take median distance and puts it into arrayOfDistAngle.
    #also assumes servos are in anitclockwise positioning only uses sensor data from sensor 1
    def matchPlane(self):
        sensor1 = self.sensorData[0]
        #top left
        medianArray = []
        for row in range(int(len(sensor1)/2)):
            for col in range(int(len(sensor1[row])/2)):
                medianArray.append(sensor1[row][col])
        self.arrayOfDistAngle[0] = int(statistics.median(medianArray))
        medianArray.clear
        #top right
        for row in range(int(len(sensor1)/2)):
            for col in range(int(len(sensor1[row])/2), int(len(sensor1[row]))):
                medianArray.append(sensor1[row][col])
        self.arrayOfDistAngle[1] = int(statistics.median(medianArray))
        medianArray.clear
        #bottom right
        for row in range(int(len(sensor1)/2), int(len(sensor1))):
            for col in range(int(len(sensor1[row])/2), int(len(sensor1[row]))):
                medianArray.append(sensor1[row][col])
        self.arrayOfDistAngle[2] = int(statistics.median(medianArray))
        medianArray.clear
        #bottom left
        for row in range(int(len(sensor1)/2), int(len(sensor1))):
            for col in range(int(len(sensor1[row])/2)):
                medianArray.append(sensor1[row][col])
        self.arrayOfDistAngle[3] = int(statistics.median(medianArray))
        medianArray.clear

    #converting array of distance to array of angles corresponding to each servo
    def convertDistancetoAngle(self):
        for i in range(len(self.arrayOfDistAngle)): #converting distance to degree
            if (self.arrayOfDistAngle[i] < 5): #minimum before the data is funky
                self.arrayOfDistAngle[i] = 0
            elif (self.arrayOfDistAngle[i] > 67.5): #maximum range of fingers
                self.arrayOfDistAngle[i] = 85  
            else:
                self.arrayOfDistAngle[i] = int(self.arrayOfDistAngle[i] * (85/67.5))

    #describes the process of filtering data and turning it into an angle
    def filter(self, msg):
        if msg.data:
            save = np.array(msg.data)
            arrayFiltered = save
            # arrayFiltered = self.feedbackFilter(save, 1.0, 0.25) #filtering
            self.sensorData = arrayFiltered.reshape(1,8,8).tolist() ####this depends on how many sensors you put in ToF_Reading
            self.matchPlane() # calculates individual finger distance
            rospy.loginfo(self.sensorData)
            rospy.loginfo(self.arrayOfDistAngle)
            self.convertDistancetoAngle() #calculates angle
        self.publish() #publishes data

    # publishes the array of angles for the servos to be set at
    def publish(self):
        if(len(self.arrayOfDistAngle) == self.numOfServo):
            self.pub.publish(Int32MultiArray(data=self.arrayOfDistAngle))
            rospy.loginfo(self.arrayOfDistAngle)

# runs main method of the program which creates the filter object and then publishes the angles to turn the servos
if __name__ == "__main__":
    FilteringToF()
    rospy.spin()