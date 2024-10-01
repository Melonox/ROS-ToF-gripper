#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import statistics
import numpy as np
from scipy import signal

#both subscriber and publisher
class FilteringToF:
    numOfServo = 4
    arrDistance = [0,0,0,0] #this is first an array of distances that changes to be angles
    sensorData = [] # this is data from sensor but will be modified to filtered array
    weight = 0.9 # subject to change, but this is based of some trials and estimations
    numberOfSensors = 4 # number of sensors on the platform
    queueData = [] # each sensor has a queue
    queueSize = 10

    #both a subscriber and a publisher node
    def __init__(self):
        rospy.init_node('servo_control', anonymous=True)
        rospy.Subscriber('/sensor/data', Int32MultiArray, self.filter)
        self.pub = rospy.Publisher('/servo/command', Float32MultiArray, queue_size=10)
        self.queueData = np.zeros((self.numberOfSensors, 8, 8)).tolist()
        for i in range(self.numberOfSensors):
            for j in range(8):
                for k in range(8):
                    self.queueData[i][j][k] = []
    
    # storing data into the queue, pushes until queue reaches queueSize, then pops to keep queueSize
    def pushAndPopQueue(self):
        for i in range(self.numberOfSensors):
            for j in range (8):
                for k in range(8):
                    if (len(self.queueData[i][j][k]) < self.queueSize):
                        self.queueData[i][j][k].append(self.sensorData[i][j][k])
                    elif (len(self.queueData[i][j][k]) > self.queueSize):
                        while (len(self.queueData[i][j][k]) > self.queueSize):
                            self.queueData[i][j][k].pop(0)
                    else:
                        self.queueData[i][j][k].append(self.sensorData[i][j][k])
                        self.queueData[i][j][k].pop(0)
    
    #filter based off previous point, changes the sensor data values through filtering
    def feedbackFilter(self, x, a0, b1):
        y = np.zeros(x.size)
        for n in range(0, x.size):
            y[n] = a0 * x[n] - b1 * y[n-1] # something here about + or - ??
        return y.tolist()
    
    #gating sensor feedback, changes the sensor data values through gating
    def gating(self, x):
        var = np.var(x)
        mean = np.mean(x)
        x[x > mean + 3*np.sqrt(var)] = mean
        return x.tolist()
    
    # process of filtering data for each individual sensor, changes sensor data
    def filterProcess(self):
        for i in range(self.numberOfSensors):
            for j in range (8):
                for k in range(8):
                    self.queueData[i][j][k] = self.gating(np.array(self.queueData[i][j][k])) #gating
                    self.queueData[i][j][k] = self.feedbackFilter(np.array(self.queueData[i][j][k]), 1.0-self.weight, self.weight) #filtering
                    self.sensorData[i][j][k] = self.queueData[i][j][k][-1]

    # matches a plane given that each sensor is responsible for their own sensors.
    def simplePlane(self):
        median = []
        for i in range(self.numberOfSensors):
            for j in range (8):
                for k in range(8):
                    median.append(self.sensorData[i][j][k])
            self.arrDistance[i] = statistics.median(median)
            median.clear()

    #describes the process of filtering data and turning it into an angle
    def filter(self, msg):
        if msg.data:
            ########################################################################
            save = np.array(msg.data)
            self.sensorData = save.reshape(self.numberOfSensors,8,8).tolist()
            self.pushAndPopQueue()
            if (len(self.queueData[0][0][0]) == self.queueSize):
                self.filterProcess()
                rospy.loginfo(self.sensorData)
                for i in range(len(self.sensorData)):
                    self.sensorData[i] = signal.medfilt2d(np.array(self.sensorData[i])).tolist()
                rospy.loginfo(self.sensorData)
                ########################################################################
                self.simplePlane() # calculates individual finger distance
                rospy.loginfo(self.arrDistance) #distance
                self.publish() #publishes data

    # publishes the array of angles for the servos to be set at
    def publish(self):
        for i in range(len(self.arrDistance)):
            self.arrDistance[i] = int(self.arrDistance[i])
        if(len(self.arrDistance) == self.numOfServo):
            self.pub.publish(Float32MultiArray(data=self.arrDistance))
            #rospy.loginfo(self.arrDistance)

# runs main method of the program which creates the filter object and then publishes the angles to turn the servos
if __name__ == "__main__":
    FilteringToF()
    rospy.spin()