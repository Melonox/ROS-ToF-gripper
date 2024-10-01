#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayDimension
import serial
import numpy as np

# this object is used to read the serial output of the time of flight sensors
class ReadingSerial:
    data = []
    lengthOfReadings = 8

    # reading board with serial
    ser = serial.Serial (
        port='/dev/ttyACM0',
        baudrate=115200
    )

    # creates an array for the amount of ToF sensors there are stored in an array
    def __init__(self, numToFSensor):
        self.numToFSensor = numToFSensor
        for i in range(numToFSensor):
            self.data.append([])
    
    # string representation of the ReadingSerial object
    def __str__(self):
        return f"There are {self.numToFSensor} ToF sensors \n data: {self.data}"
    
    # interprets the serial output into an array of strings
    def readingSerial(self):
        val = self.ser.readline()
        valueInString = str(val, 'UTF-8') #sometimes utf-8 code can't decode?
        input = valueInString.strip().split()
        return input
    
    # read serial data for each numToFSensor it has in ascending order
    # stored in data field, so that the data field changes and 
    # clears previous data based on current reading
    # no return, just call this and access the data field
    def reading(self):
        self.data.clear()
        for i in range(self.numToFSensor):
            self.data.append([])
        while(len(self.data[0]) < self.lengthOfReadings): # checks first sensor
            preValue = self.readingSerial()
            if (not preValue): # preValue will always be blank
                currValue = self.readingSerial()
                if(currValue[0][:2] == "1:"):
                    currValue = self.changeInt(currValue)
                    self.data[0].append(currValue)
                    for i in range(self.lengthOfReadings-1):
                        currValue = self.readingSerial()
                        currValue = self.changeInt(currValue)
                        self.data[0].append(currValue)
        # enumarate rest since we are now in order
        for i in range(1, self.numToFSensor):
            blank = self.readingSerial()
            for j in range(self.lengthOfReadings):
                newReading = self.readingSerial()
                newReading = self.changeInt(newReading)
                self.data[i].append(newReading)
        #delete empty arrays in case there are any
        self.data = list(filter(None, self.data))

    # makes the serial data strings into integers
    def changeInt(self, arrayOfStrings):
        for i in range(len(arrayOfStrings)):
            arrayOfStrings[i] = int(arrayOfStrings[i][2:])
        return arrayOfStrings

# method for running the ROS publisher node and publishes the ToF sensors 
# data with an single array comprising 6x8x8 points
def publishSerialReading():
    pub = rospy.Publisher('/sensor/data', Int32MultiArray, queue_size=0)
    rospy.init_node('servo_control', anonymous=True)
    rosSensor = ReadingSerial(4) #IF YOU CHANGE THIS, YOU MUST CHANGE THE ToF_Filtering as well
    while not rospy.is_shutdown():
        rosSensor.reading()
        msg = np.array(rosSensor.data)
        msg = np.ravel(msg)
        arrayMSG = msg.tolist()
        # rospy.loginfo(arrayMSG) # if you really wanted to be overwhelmed with the messages
        pub.publish(Int32MultiArray(data=arrayMSG))
    
# standard ROS main method
if __name__ == '__main__':
    publishSerialReading()