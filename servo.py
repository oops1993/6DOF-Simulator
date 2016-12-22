from __future__ import division
from math import *
import time
import time
I2C = False
try:
    import Adafruit_PCA9685
    I2C = True
    print("Succeeded to import Adafruit_PCA9685.\n")
except Exception as ex1:
    print("Failed to import Adafruit_PCA9685.\n")

pwm = 0
servoFrequency = 50
servoMinDutyCycle = 2.5
servoMaxDutyCycle = 12.5
servoPinNumber = range(4,10)
crank2servo = (60.,-60.,60.,-60.,60.,-60.)
servoInitialAngle = (90.,90.,90.,90.,90.,90.)

def servoInitialize():
    global servoInitialAngle, pwm, servoFrequency
    if I2C:
        pwm = Adafruit_PCA9685.PCA9685()
        pwm.set_pwm_freq(servoFrequency)
        changeServoAngle(servoInitialAngle)

def changeServoAngle(angleList):
    global servoMinDutyCycle, servoMinDutyCycle, servoPinNumber, pwm
    def pl(angle):
        if angle>=0. and angle<=180.:
            return int((angle/180.*(servoMaxDutyCycle-servoMinDutyCycle)+servoMinDutyCycle)*40.96+0.5)
        elif angle<0.:
            return int((servoMinDutyCycle)*40.96+0.5)
        elif angle>180.:
            return int((servoMaxDutyCycle)*40.96+0.5)
    for i in (0,1,2,3,4,5):
        if I2C:
            pwm.set_pwm(servoPinNumber[i], 0 , pl(angleList[i]))
        if I2C==False:
            print("Servo Pin Number: {0}, Pulse length = {1}".format(servoPinNumber[i],pl(angleList[i])))
def changeCrankAngle(angleList):
    global crank2servo
    changeServoAngle(map(lambda (a,b):a+b, zip(angleList,crank2servo)))
if __name__ =="__main__":
    servoInitialize()
    time.sleep(1)
    for i in range(0,3):
        changeServoAngle([45., 135., 45., 135., 45., 135.])
        time.sleep(1)
        changeServoAngle([135., 45., 135., 45., 135., 45.])
        time.sleep(1)

    
