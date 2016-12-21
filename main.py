from __future__ import division
import sixDof
reload(sixDof)
from math import *
import time
import numpy.matlib as ml
import time
# Import the PCA9685 module.
I2C = False
try:
    import Adafruit_PCA9685
    I2C= True
except Exception as ex1:
    print("Failed to import Adafruit_PCA9685")

if I2C:
    servoFrequency = 50
    servoMinDutyCycle = 2.5
    servoMaxDutyCycle = 12.5
    servoPinNumber = (0,1,2,3,4,5)
    crank2servo = (60.,-60.,60.,-60.,60.,-60.)
    servoInitialAngle = (90.,90.,90.,90.,90.,90.)

    def servoInitialize():
        global servoInitialAngle, pwm, servoFrequency
        pwm = Adafruit_PCA9685.PCA9685()
        pwm.set_pwm_freq(servoFrequency)
        changeServoAngle(servoInitialAngle)

    def changeServoAngle(angleList):
        global servoMinDutyCycle, servoMinDutyCycle, servoPinNumber, pwm
        def pl(angle):
            if angle>=0. and angle<=180.:
                return floor((angle/(servoMinDutyCycle-servoMinDutyCycle)+servoMinDutyCycle)/40.96+0.5)
            elif angle<0.:
                return floor((servoMinDutyCycle)/40.96+0.5)
            elif angle>180.:
                return floor((servoMaxDutyCycle)/40.96+0.5)
        for i in (0,1,2,3,4,5):
            pwm.set_pwm(servoPinNumber[i], 0 , pl(angleList[i]))
    def changeCrankAngle(angleList):
        global crank2servo
        changeServoAngle(map(lambda (a,b):a+b, zip(angleList,crank2servo)))

    servoInitialize()
    time.sleep(1)

a = sixDof.sixDofSimPltfm()
Rot1 = sixDof.calcTransformMatrix(\
    20.,0.,0.,3.,3.,3.,degree=True)
a.transformByMatrixGlobal(Rot1)
a.printCoords('pivots')
if False:
    for i in range(0,6):
        print(a.pltfmPivotCoordSystemLocal[i])
        print(a.pltfmPivotCoordSystem[i])
a.printAngles('crank')
print a
a.calcBearingAngle()
print(a.checkBearingAngle())
if I2C:
    changeCrankAngle([45., 135., 45., 135., 45., 135.])
