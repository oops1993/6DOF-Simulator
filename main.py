from __future__ import division
import sixDof
reload(sixDof)
from math import *
import time
import numpy.matlib as ml
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
def changeCrankAngle(angleList):
    global crank2servo
    if I2C:
        changeServoAngle(map(lambda (a,b):a+b, zip(angleList,crank2servo)))
if I2C:
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
print a.checkBearingAngle(),'\n'
a.pltfmPosition.printAngles()
if I2C:
    while True:
        changeServoAngle([45., 135., 45., 135., 45., 135.])
        time.sleep(1)
        changeServoAngle([135., 45., 135., 45., 135., 45.])
        time.sleep(1)
if False:

    # Initialise the PCA9685 using the default address (0x40).
    pwm = Adafruit_PCA9685.PCA9685()

    # Alternatively specify a different address and/or bus:
    #pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

    # Configure min and max servo pulse lengths
    servo_min = 150  # Min pulse length out of 4096
    servo_max = 600  # Max pulse length out of 4096

    # Helper function to make setting a servo pulse width simpler.
    def set_servo_pulse(channel, pulse):
        pulse_length = 1000000    # 1,000,000 us per second
        pulse_length //= 60       # 60 Hz
        print('{0}us per period'.format(pulse_length))
        pulse_length //= 4096     # 12 bits of resolution
        print('{0}us per bit'.format(pulse_length))
        pulse *= 1000
        pulse //= pulse_length
        pwm.set_pwm(channel, 0, pulse)

    # Set frequency to 60hz, good for servos.
    pwm.set_pwm_freq(60)

    print('Moving servo on channel 0, press Ctrl-C to quit...')
    while True:
        # Move servo on channel O between extremes.
        pwm.set_pwm(0, 0, servo_min)
        time.sleep(1)
        pwm.set_pwm(0, 0, servo_max)
        time.sleep(1)