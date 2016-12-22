import sixDof
reload(sixDof)
from math import *
import time
import numpy.matlib as ml
import servo
#added sth
a = sixDof.sixDofSimPltfm()
Rot1 = sixDof.calcTransformMatrix(\
    0.,0.,0.,0.,0.,0.,degree=True)
a.transform(Rot1,'Global')
a.printCoords('pivots')
if False:
    for i in range(0,6):
        print(a.pltfmPivotCoordSystemLocal[i])
        print(a.pltfmPivotCoordSystem[i])
a.printAngles('crank')
print a
a.calcBearingAngle()
print a.checkBearingAngle(),'\n'
servo.servoInitialize()
servo.changeCrankAngle(a.crankAnglesNow)
time.sleep(1)
Rot2 = sixDof.calcTransformMatrix(\
    1.,0.,0.,0.,0.,0.,degree=True)
for i in range(0,40):
    a.transform(Rot2,'Global')
    print 'HaHa'
    servo.changeCrankAngle(a.crankAnglesNow)
    #time.sleep(0.1)

while False:
    servo.changeServoAngle([45., 135., 45., 135., 45., 135.])
    time.sleep(1)
    servo.changeServoAngle([135., 45., 135., 45., 135., 45.])
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
