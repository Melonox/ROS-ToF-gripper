#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayDimension
import serial
import statistics

ser = serial.Serial (
    port='/dev/ttyACM0',
    baudrate=115200
)

# 3D array creation from serial
def readSerial():
    FilterArray = []
    for i in range(6):
        tempOneSensorArray = []
        if(i != 0): #for blank ln from serial monitor, I don't think we need this
            value = ser.readline()
        while (len(tempOneSensorArray) < 8):
            value = ser.readline()
            valueInString=str(value, 'UTF-8')
            input = valueInString.strip().split()
            if input:
                allValues = []
                for k in range(len(input)):
                    allValues.append(int(float(input[k][2:])))
                tempOneSensorArray.append(allValues)
        FilterArray.append(tempOneSensorArray)
    return FilterArray

#only reads sensor 1, still 3D array
#
def testReadSerial():
    FilterArray = []
    for i in range(6):
        tempOneSensorArray = []
        while (len(tempOneSensorArray) < 8):
            value = ser.readline()
            valueInString=str(value, 'UTF-8')
            input = valueInString.strip().split()
            if input:
                if input[0][0:2] == "1:":
                    allValues = []
                    for k in range(len(input)):
                        allValues.append(int(float(input[k][2:])))
                    tempOneSensorArray.append(allValues)
        FilterArray.append(tempOneSensorArray)
    return FilterArray


#median filtering with median approach, or I could have just used one line from openCV
def readToFmedianFilter(): #don't need median
    MedianFilterArray = testReadSerial()
    for i in range(len(MedianFilterArray)):
        for j in range(len(MedianFilterArray[i])):
            for k in range(len(MedianFilterArray[i][j])):
                tempMedian = []
                if(j > 0): #access above
                    tempMedian.append(MedianFilterArray[i][j-1][k])
                    if(k > 0): #access top left
                        tempMedian.append(MedianFilterArray[i][j-1][k-1])
                    if(k < (len(MedianFilterArray[i][j])-1) ): #access top right
                        tempMedian.append(MedianFilterArray[i][j-1][k+1])
                if(j < (len(MedianFilterArray[i])-1) ): #access below
                    tempMedian.append(MedianFilterArray[i][j+1][k])
                    if(k > 0): #access bottom left
                        tempMedian.append(MedianFilterArray[i][j+1][k-1])
                    if(k < (len(MedianFilterArray[i][j])-1) ): #access bottom right
                        tempMedian.append(MedianFilterArray[i][j+1][k+1])
                if(k > 0): #access right
                    tempMedian.append(MedianFilterArray[i][j][k-1])
                if(k < (len(MedianFilterArray[i][j])-1) ): #access left
                    tempMedian.append(MedianFilterArray[i][j][k+1])
                tempMedian.append(MedianFilterArray[i][j][k])
                tempMedian.sort()
                MedianFilterArray[i][j][k]=tempMedian[int((len(tempMedian)-1)/2)]
    return MedianFilterArray

# average/ gaussian approach
def readTofSensorAvg(average):
    for i in range(6): 
        if(i != 0): #for blank ln from serial monitor
            value = ser.readline()
        for j in range(8): #summing all sensor data
            value = ser.readline()
            valueInString=str(value, 'UTF-8')
            input = valueInString.strip().split()
            for element in input:
                num = int(float(element[2:]))
                average[i] += num
        average[i] = average[i]/64
    return average

# median approach (use this)
def readTofSensorMedian(median):
    for i in range(6):
        allValues = []
        while(len(allValues) < 64): #summing all sensor data
            value = ser.readline()
            valueInString=str(value, 'UTF-8')
            input = valueInString.strip().split()
            if input:
                for k in range(len(input)):
                    allValues.append(int(float(input[k][2:])))
        allValues.sort()
        median[i] = int(0.9*(allValues[31]+allValues[32])/2) ## actually pretty good
    return median

#maybe also change this to median
def degree(arrayOfSensorData):
    average = 0
    for i in range(len(arrayOfSensorData)):
        average += arrayOfSensorData[i]
    average = average/len(arrayOfSensorData)
    if (average > 67.5 or average < 5): #max range
        return 0
    else:
        return int( average * (85/67.5))

def MedianRegFilt(FilteredArray): #this is slower now. Is this even right?
    degrees = [0,0,0,0] # 4 servos
    for i in range(len(FilteredArray)):
        upperbound = int(len(FilteredArray[i])/2)
        leftboud = int(len(FilteredArray[i][0])/2)
        lowerbound = len(FilteredArray[i])
        rightbound = len(FilteredArray[i][0])
        topLeft = []
        for j in range(upperbound):
            for k in range(leftboud):
                topLeft.append(FilteredArray[i][j][k])
        topRight=[]
        for j in range(upperbound):
            for k in range(leftboud, rightbound):
                topRight.append(FilteredArray[i][j][k])
        bottomLeft = []
        for j in range(upperbound, lowerbound):
            for k in range(leftboud):
                bottomLeft.append(FilteredArray[i][j][k])
        bottomRight = []
        for j in range(upperbound, lowerbound):
            for k in range(leftboud, rightbound):
                bottomRight.append(FilteredArray[i][j][k])
    degrees[0] = statistics.median(topRight) #index also corresponds to which servo
    degrees[1] = statistics.median(topLeft)
    degrees[2] = statistics.median(bottomLeft)
    degrees[3] = statistics.median(bottomRight) #this also might be flipped
    for i in range(len(degrees)): #converting distance to degree
        if (degrees[i] > 67.5 or degrees[i] < 5): #max range
            degrees[i] = 0
        else:
            degrees[i] = int( degrees[i] * (85/67.5))
    return degrees

# defines a structure for this node
def process():
    FilteredArray = readToFmedianFilter()
    regionedDegrees = MedianRegFilt(FilteredArray)
    return regionedDegrees

def servos():
    pub = rospy.Publisher('/servo/command', Int32MultiArray, queue_size=10)
    rospy.init_node('servo_control', anonymous=True)
    #rate = rospy.Rate(10)
    arrayMSG = Int32MultiArray()
    arrayMSG.layout.dim = [MultiArrayDimension(label='MutliOutput', size = 8, stride = 8)]
    arrayMSG.layout.data_offset = 0
    previousAngle = [0,0,0,0]
    while not rospy.is_shutdown():
        turnAngles = process()
        arrayMSG.data = [turnAngles[0], turnAngles[1], turnAngles[2], turnAngles[3], 
                            previousAngle[0], previousAngle[1], previousAngle[2], previousAngle[3]]
        # rospy.loginfo(arrayMSG)
        for i in range (0, 4): 
            rospy.loginfo("Servo"+str(i)+"Publishing servo angle: "+str(arrayMSG.data[i]))
        pub.publish(arrayMSG)
        previousAngle = turnAngles
        # rospy.loginfo("Finishing setting angle: "+str(arrayMSG.data[1]))
        for i in range (4, 8): 
            rospy.loginfo("Servo"+str(i)+"Previous servo angle: "+str(arrayMSG.data[i]))
        #rate.sleep()

if __name__ == '__main__':
    try:
        servos()
    except rospy.ROSInterruptException:
        pass