#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, Int32MultiArray, UInt16MultiArray
import statistics
import numpy as np
from scipy import signal
import sys

np.set_printoptions(threshold=sys.maxsize)
#both subscriber and publisher
class FilteringToF:

    #both a subscriber and a publisher node
    def __init__(self, num_sensors, sensor_readings, filter_history):
        self.numOfServo = 4
        self.arrDistance = [0,0,0,0] #this is first an array of distances that changes to be angles
        self.sensorData = [] # this is data from sensor but will be modified to filtered array
        self.weight = 0.9 # subject to change, but this is based of some trials and estimations
        self.numberOfSensors = num_sensors # number of sensors on the platform
        self.sensor_readings = sensor_readings
        self.queueData = [] # each sensor has a queue
        self.queueSize = 10
        # rospy.Subscriber('/sensor/data', Int32MultiArray, self.filter)
        rospy.Subscriber('/sensor/data', Int32MultiArray, self.vectorized_filter)
        self.pub_filtered = rospy.Publisher('/sensor/filtered_data', UInt16MultiArray, queue_size=10)
        self.pub_avr_readings = rospy.Publisher('/servo/command', UInt16MultiArray, queue_size=10)
        self.queueData = np.zeros((self.numberOfSensors, 8, 8)).tolist()
        for i in range(self.numberOfSensors):
            for j in range(8):
                for k in range(8):
                    self.queueData[i][j][k] = []

        self.filter_history = 100 #filter_history
        self.filterData = np.zeros((self.filter_history, self.sensor_readings * self.numberOfSensors))
        self.filteredData = self.filterData
        # self.counter = 0
    
    def vectorized_filter(self, msg):
        # import pdb; pdb.set_trace()
        data = np.array(msg.data)
        self.filterData[:-1,:] = self.filterData[1:,:]
        self.filterData[-1,:] = data
        # print(self.filterData[0])
            
        self.filteredData = self.feedbackFilter(self.filterData, 0.2, 0.8)

        # self.counter += 1
        # if self.counter % 20 == 0:
        #     import pdb; pdb.set_trace()
            #self.visualize(self.filterData)

    def visualize(self):
        import matplotlib.pyplot as plt
        plt.plot(np.arange(self.filter_history), self.filterData[:,0])
        plt.plot(np.arange(self.filter_history), self.filteredData[:,0])
        plt.show()

    #filter based off previous point, changes the sensor data values through filtering
    def feedbackFilter(self, x, a0, b1):
        y = np.zeros_like(x)
        for n in range(0, x.shape[0]):
            y[n,:] = a0 * x[n,:] + b1 * y[n-1,:] # something here about + or - ??
        return y
    
    def gating(self, x):
        var = np.var(x)
        mean = np.mean(x)
        x[x > mean + np.sqrt(var)] = mean
        return x

    def avg_sensor_reading(self, readings):
        # import pdb; pdb.set_trace()
        data = readings[-1,:].reshape((self.numberOfSensors, self.sensor_readings))
        return data.mean(axis=1)

    
    # # storing data into the queue, pushes until queue reaches queueSize, then pops to keep queueSize
    # def pushAndPopQueue(self):
    #     for i in range(self.numberOfSensors):
    #         for j in range (8):
    #             for k in range(8):
    #                 if (len(self.queueData[i][j][k]) < self.queueSize):
    #                     self.queueData[i][j][k].append(self.sensorData[i][j][k])
    #                 elif (len(self.queueData[i][j][k]) > self.queueSize):
    #                     while (len(self.queueData[i][j][k]) > self.queueSize):
    #                         self.queueData[i][j][k].pop(0)
    #                 else:
    #                     self.queueData[i][j][k].append(self.sensorData[i][j][k])
    #                     self.queueData[i][j][k].pop(0)
    
    #filter based off previous point, changes the sensor data values through filtering
    # def feedbackFilter(self, x, a0, b1):
    #     y = np.zeros(x.size)
    #     for n in range(0, x.size):
    #         y[n] = a0 * x[n] - b1 * y[n-1] # something here about + or - ??
    #     return y.tolist()
    
    #gating sensor feedback, changes the sensor data values through gating
    # def gating(self, x):
    #     var = np.var(x)
    #     mean = np.mean(x)
    #     x[x > mean + 3*np.sqrt(var)] = mean
    #     return x.tolist()
    
    # # process of filtering data for each individual sensor, changes sensor data
    # def filterProcess(self):
    #     for i in range(self.numberOfSensors):
    #         for j in range (8):
    #             for k in range(8):
    #                 self.queueData[i][j][k] = self.gating(np.array(self.queueData[i][j][k])) #gating
    #                 self.queueData[i][j][k] = self.feedbackFilter(np.array(self.queueData[i][j][k]), 1.0-self.weight, self.weight) #filtering
    #                 self.sensorData[i][j][k] = self.queueData[i][j][k][-1]

    # # matches a plane given that each sensor is responsible for their own sensors.
    # def simplePlane(self):
    #     median = []
    #     for i in range(self.numberOfSensors):
    #         for j in range (8):
    #             for k in range(8):
    #                 median.append(self.sensorData[i][j][k])
    #         self.arrDistance[i] = statistics.median(median)
    #         median.clear()

    #describes the process of filtering data and turning it into an angle
    # def filter(self, msg):
    #     if msg.data:
    #         ########################################################################
    #         save = np.array(msg.data)
    #         self.sensorData = save.reshape(self.numberOfSensors,8,8).tolist()
    #         self.pushAndPopQueue()
    #         if (len(self.queueData[0][0][0]) == self.queueSize):
    #             self.filterProcess()
    #             rospy.loginfo(self.sensorData)
    #             for i in range(len(self.sensorData)):
    #                 self.sensorData[i] = signal.medfilt2d(np.array(self.sensorData[i])).tolist()
    #             rospy.loginfo(self.sensorData)
    #             ########################################################################
    #             self.simplePlane() # calculates individual finger distance
    #             rospy.loginfo(self.arrDistance) #distance
    #             self.publish() #publishes data

    # publishes the array of angles for the servos to be set at
    # def publish(self):
        # for i in range(len(self.arrDistance)):
        #     self.arrDistance[i] = int(self.arrDistance[i])
        # if(len(self.arrDistance) == self.numOfServo):
        #     self.pub.publish(Float32MultiArray(data=self.arrDistance))
        #     #rospy.loginfo(self.arrDistance)


# runs main method of the program which creates the filter object and then publishes the angles to turn the servos
if __name__ == "__main__":
    rospy.init_node('tof_filtering', anonymous=True)
    num_sensors = rospy.get_param('num_sensors', 4)
    sensor_readings = rospy.get_param('sensor_readings', 64)
    filter_history = rospy.get_param('filter_history', 10)
    filter = FilteringToF(num_sensors, sensor_readings, filter_history)
    
    while not rospy.is_shutdown():
        filter.pub_filtered.publish(UInt16MultiArray(data=filter.filteredData[-1,:].astype(np.uint16)))
        filter.pub_avr_readings.publish(UInt16MultiArray(data=filter.avg_sensor_reading(filter.filteredData).astype(np.uint16)))