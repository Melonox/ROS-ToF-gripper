#!/usr/bin/env python
from adafruit_servokit import ServoKit
import time
myKit = ServoKit(channels=16) ##there are 16 channels on the board

# for i in range(0, 85):
#     for j in range (0, 4):
#         myKit.servo[j].angle=i
#         time.sleep(0.01)
# for i in range(85,0, -1):
#     for j in range (0, 4):
#         myKit.servo[j].angle=i
#         time.sleep(0.01)

# myKit.servo[0].angle=int(46*85/67.5)
# myKit.servo[1].angle=int(46*85/67.5)
# myKit.servo[2].angle=int(46*85/67.5)
# myKit.servo[3].angle=int(46*85/67.5)
# print(int(46*85/65))

myKit.servo[0].angle=0
myKit.servo[1].angle=0
myKit.servo[2].angle=0
myKit.servo[3].angle=0
myKit.servo[4].angle=0
myKit.servo[5].angle=0
myKit.servo[6].angle=0
myKit.servo[7].angle=0 #55 #110 #20
myKit.servo[8].angle=0
myKit.servo[9].angle=0
myKit.servo[10].angle=0
myKit.servo[11].angle=0
myKit.servo[12].angle=0
myKit.servo[13].angle=0
myKit.servo[14].angle=0
myKit.servo[15].angle=10.25

time.sleep(1)
# import pdb; pdb.set_trace()
# myKit.servo[7].angle

# we will say for all servos, max is 85.
# for i in range(0,4,1):
#     myKit.servo[i].angle=0 #might need to manually calibrate each servo to 0 before assembly
#     maxAngle = 87
#     if i == 0: #different servo
#         maxAngle = 85
#     for j in range(0,maxAngle+1,1): 
#         myKit.servo[i].angle=j      
#         print("Servo: "+str(i)+" Angle: "+str(j))
#         time.sleep(.05)
#     for j in range(maxAngle,-1,-1):
#         myKit.servo[i].angle=j      
#         print("Servo: "+str(i)+" Angle: "+str(j))
#         time.sleep(.05)
#     myKit.servo[i].angle=maxAngle
#     time.sleep(3)
#     myKit.servo[i].angle=0
#     time.sleep(3)