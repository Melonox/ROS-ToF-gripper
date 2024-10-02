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
    
    # string representation of the ReadingSerial object
    def __str__(self):
        return f"There are {self.numToFSensor} ToF sensors \n data: {self.data}"
    
    def string_sensor_data(self):
        all_readings = ""
        
        while True:
            val = self.ser.readline()
            valueInString = str(val, 'UTF-8')
            
            if valueInString[1:3] == "1:":
                all_readings = all_readings + valueInString
                break
        numberOfSensorReadingsLeft = self.numToFSensor * 8 - 1 + (self.numToFSensor)
        #could be faster
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
                #print(readings[reading])
                sensor_id, value = readings[reading].split(':')
                values.append(int(value))

        return np.array(values)

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

        self.data = [[]] * self.numToFSensor
        print(len(self.data), len(self.data[0]))
        while(len(self.data[0]) < self.lengthOfReadings): # checks first sensor
            preValue = self.readingSerial()
            if (not preValue): # preValue will always be blank
                currValue = self.readingSerial()
                if(currValue[0][:2] == "1:"):
                    currValue = self.changeInt(currValue)
                    self.data[0].append(currValue.copy())
                    import pdb; pdb.set_trace()
                    for i in range(self.lengthOfReadings-1):
                        currValue = self.readingSerial()
                        currValue = self.changeInt(currValue)
                        self.data[0].append(currValue.copy())
                        import pdb; pdb.set_trace()
        print(len(self.data), len(self.data[0]), len(self.data[0][0]))
        #print(self.data)
        import pdb; pdb.set_trace()
        # enumarate rest since we are now in order
        for i in range(1, self.numToFSensor):
            blank = self.readingSerial()
            self.data[i].clear()
            for j in range(self.lengthOfReadings):
                newReading = self.readingSerial()
                newReading = self.changeInt(newReading)
                print(len(self.data), len(self.data[0]), len(self.data[0][0]), i, len(newReading))
                self.data[i].append(newReading.copy())
            
            #print(self.data[i], str(i), np.array(self.data[i]).shape)
        #delete empty arrays in case there are any
        #self.data = list(filter(None, self.data))
        # print(self.data)
        # print(f'self.numToFSensor {self.numToFSensor}')

    # makes the serial data strings into integers
    def changeInt(self, arrayOfStrings):
        for i in range(len(arrayOfStrings)):
            arrayOfStrings[i] = int(arrayOfStrings[i][2:])
        return arrayOfStrings


# method for running the ROS publisher node and publishes the ToF sensors 
# data with an single array comprising 6x8x8 points
def publishSerialReading():
    pub = rospy.Publisher('/sensor/data', Int32MultiArray, queue_size=0)
    rospy.init_node('sensor_data', anonymous=True)
    num_sensors = rospy.get_param('num_sensors', 4) # Default is 4 sensors
    port = rospy.get_param('port', "/dev/ttyACM0") # Default is 4 sensors
    baudrate = rospy.get_param('baudrate', 115200) # Default is 4 sensors
    rosSensor = ReadingSerial(num_sensors, port, baudrate) #IF YOU CHANGE THIS, YOU MUST CHANGE THE ToF_Filtering as well
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
        pub.publish(Int32MultiArray(data=arrayMSG))
    
# standard ROS main method
if __name__ == '__main__':
    publishSerialReading()
