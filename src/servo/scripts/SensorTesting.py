import serial
import time

ser = serial.Serial (
    port='/dev/ttyACM0',
    baudrate=115200
)

# def readingSerial():
#     val = ser.readline()
#     valueInString=str(val, 'UTF-8')
#     input = valueInString.strip().split()
#     return input

oneSec = time.time() + 0.3
while time.time() < oneSec:
    value = ser.readline()
    print(value)

# while True:
#     value = ser.readline()
#     print(value)


## this only gets data from the 1: sensor and in order too!
# filterArray = []
# while time.time() < oneSec:
#     preValue = readingSerial()
#     currValue = readingSerial()
#     if (not preValue and currValue): #preValue will always be blank
#         if(currValue[0][:2] == "1:"):
#             filterArray.append(currValue)
#             for i in range(7):
#                 currValue = readingSerial()
#                 filterArray.append(currValue)
#             break
# print(filterArray)
# print(len(filterArray))
