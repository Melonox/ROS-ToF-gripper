#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayDimension
import serial
import numpy as np

# this object is used to read the serial output of the time of flight sensors
class ReadingSerial:
    # creates an array for the amount of ToF sensors there are stored in an array
    def __init__(self, numToFSensor, port, baudrate):
        self.numToFSensor = numToFSensor
        self.data = [[]] * self.numToFSensor
        # reading board with serial
        self.ser = serial.Serial (
            port="/dev/ttyACM0",
            baudrate=baudrate
        )
        self.lengthOfReadings = 8
        self.numberOfDataPoints = self.lengthOfReadings * self.lengthOfReadings * numToFSensor
    
    # string representation of the ReadingSerial object
    def __str__(self):
        return f"There are {self.numToFSensor} ToF sensors \n data: {self.data}"
    
    #still some errors when parsing, this will probably need a try and catch
    def string_sensor_data(self):
        all_readings = ""
        val = self.ser.readline()
        valueInString = str(val, 'UTF-8')
        if valueInString[1:3] == "1:":
            all_readings = all_readings + valueInString
        else:
            while valueInString[1:3] != "1:":
                val = self.ser.readline()
                valueInString = str(val, 'UTF-8')
                if valueInString[1:3] == "1:":
                    all_readings = all_readings + valueInString
        numberOfSensorReadingsLeft = self.numToFSensor * 8 - 1 + (self.numToFSensor)
        for i in range(numberOfSensorReadingsLeft):
            val = self.ser.readline()
            valueInString = str(val, 'UTF-8')
            all_readings = all_readings + valueInString
        
        return all_readings
    
    def parse_sensor_data(self, data):
        lines = data.split('\r\n')
        values = []
        for line in lines:
            readings = line.split('\t')
            for reading in range(1, len(readings)):
                sensor_id, value = readings[reading].split(':')
                values.append(int(value))

        return np.array(values)

# method for running the ROS publisher node and publishes the ToF sensors 
# data with an single array comprising 6x8x8 points
def publishSerialReading():
    pub = rospy.Publisher('/sensor/data', Int32MultiArray, queue_size=1)
    rospy.init_node('sensor_data', anonymous=True)
    num_sensors = rospy.get_param('num_sensors', 4) # Default is 4 sensors
    port = rospy.get_param('port', "/dev/ttyACM0") # Default is 4 sensors
    baudrate = rospy.get_param('baudrate', 115200) # Default is 4 sensors
    rosSensor = ReadingSerial(num_sensors, port, baudrate) 
    while True:
            char = rosSensor.ser.read()
            if char == b'\n':
                rospy.loginfo('Flushed buffer')
                break 
    while not rospy.is_shutdown():
        data = rosSensor.string_sensor_data()
        arr = rosSensor.parse_sensor_data(data)
        #rosSensor.reading()
        msg = arr
        msg = np.ravel(msg)
        arrayMSG = msg.tolist()
        # rospy.loginfo(arrayMSG) # if you really wanted to be overwhelmed with the messages
        if (len(msg) == rosSensor.numberOfDataPoints): #checks to make sure we pass 256 points
            pub.publish(Int32MultiArray(data=arrayMSG))
    
# standard ROS main method
if __name__ == '__main__':
    publishSerialReading()

    
##############sensor testing
# 3.286 -> board 3.3V supply
# 3.15 -> sensor 1
# 3.04 -> senor 4
# After seperate power
# 3.27 -> sensor 1
# 3.19 -> sensor 4
#
#
#
#
