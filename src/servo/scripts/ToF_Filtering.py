#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, Int32MultiArray, UInt16MultiArray
import statistics
import numpy as np
from scipy import signal
import sys

import matplotlib.pyplot as plt
# np.set_printoptions(threshold=sys.maxsize)
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
        # rospy.Subscriber('/sensor/data', Int32MultiArray, self.filter)
        rospy.Subscriber('/sensor/data', Int32MultiArray, self.vectorized_filter)
        self.pub_filtered = rospy.Publisher('/sensor/filtered_data', UInt16MultiArray, queue_size=1)
        self.pub_avr_readings = rospy.Publisher('/servo/command', UInt16MultiArray, queue_size=1)

        self.filter_history = filter_history
        self.filterData = np.zeros((self.filter_history, self.sensor_readings * self.numberOfSensors))
        self.filteredData = self.filterData
        # self.counter = 0
    
    def vectorized_filter(self, msg):
        data = np.array(msg.data)
        self.filterData[:-1,:] = self.filterData[1:,:]
        self.filterData[-1,:] = data
        # apply spatial filter here
        # self.filterData = self.isInCompartment(self.filterData)
        # self.filterData = self.gating(self.filterData)
        # self.filteredData = self.feedbackFilter(self.filterData, 0.5, 0.5) #bad readings if close

    def visualize(self):
        plt.plot(np.arange(self.filter_history), self.filterData[:,0])
        plt.plot(np.arange(self.filter_history), self.filteredData[:,0])
        plt.show()

    # check
    def isInCompartment(self, data):
        # self.bin -> convert us to 3d ### or convert in coming to a distance -> what inputs?
        # set outside points = 0, we do want the positioning of the array
        # this also means we have to code other parts so that it doesn't filter the 0 as actual points.
        return data

    #filter based off previous point, changes the sensor data values through filtering
    def feedbackFilter(self, x, a0, b1):
        y = np.zeros_like(x)
        for n in range(0, x.shape[0]):
            # apply spatial here
            # #if x != 0 (bad reading or outside of compartment):
            y[n,:] = a0 * x[n,:] + b1 * y[n-1,:] 
        return y
    
    #check if gating applies to individual filters or across
    def gating(self, x):
        var = np.var(x)
        mean = np.mean(x)
        upper_threshold = mean + 2*np.sqrt(var)
        lower_threshold = mean - 2*np.sqrt(var)
        # apply spatial here
        #if x != 0 (bad reading or outside of compartment):
        x= np.where(x > upper_threshold, mean, x)
        x= np.where(x < lower_threshold, mean, x)
        return x

    def avg_sensor_reading(self, readings):
        data = readings[-1,:].reshape((self.numberOfSensors, self.sensor_readings))
        return data.mean(axis=1)
    
    def avg_center_readings(self, readings):
        data = readings[-1,:].reshape((self.numberOfSensors, self.sensor_readings))
        return data[:,30:36].mean(axis=1)

    def median_center_readings(self, readings):
        # print(readings)
        data = readings[-1,:].reshape((self.numberOfSensors, self.sensor_readings))
        accuracy_range = np.concatenate((data[:, 41:46], data[:, 49:54]), axis=1)
        return np.median(accuracy_range, axis=1)

# runs main method of the program which creates the filter object and then publishes the angles to turn the servos
if __name__ == "__main__":
    rospy.init_node('tof_filtering', anonymous=True)
    num_sensors = rospy.get_param('num_sensors', 4)
    sensor_readings = rospy.get_param('sensor_readings', 64)
    filter_history = rospy.get_param('filter_history', 10)
    filter = FilteringToF(num_sensors, sensor_readings, filter_history)
    rospy.loginfo("Initialized Filtering")
    while not rospy.is_shutdown():
        filter.pub_filtered.publish(UInt16MultiArray(data=filter.filteredData[-1,:].astype(np.uint16)))
        filter.pub_avr_readings.publish(UInt16MultiArray(data=filter.median_center_readings(filter.filteredData).astype(np.uint16)))