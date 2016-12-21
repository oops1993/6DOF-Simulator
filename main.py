from __future__ import division
import sixDof
reload(sixDof)
from math import *
import time
import numpy.matlib as ml
import time
# Import the PCA9685 module.
try:
    import Adafruit_PCA9685
except Exception as ex1:
    print("Failed to import Adafruit_PCA9685")
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
    20.,0.,0.,0.,0.,0.,degree=True)
a.transformByMatrixGlobal(Rot1)
a.printCoords('pivots')
if False:
    for i in range(0,6):
        print(a.pltfmPivotCoordSystemLocal[i])
        print(a.pltfmPivotCoordSystem[i])
a.printAngles('crank')
a.calcBearingAngle()
print(a.checkBearingAngle())
changeCrankAngle([45., 135., 45., 135., 45., 135.])
#a.printCrankAngles()
#a.printAnglesRodToCrankPlane()
#a.printCoords('pivots')
#a.printCoords('pivots0')
#a.printCoords('crankPivot')
#a.printAngles('rod2CrankPlane')
#a.printAngles('crankAngle0')

#a.transformByMatrixGlobal(Rot2)
#b.transform(Rot1,changeType='Global')
#b.printCrankAngles()
#a.printCrankAngles()
#print(b.pltfmPosition)
#a.transformByMatrixAbs(Rot2)
#print(a.pltfmPosition)
#b.transformByMatrixLocal(Rot2)
#print(b.pltfmPosition)

'''
def calcCrankAngleSeq(startAngle=0.,stopAngle=0.,division=1,axis='Z',loop = False):
    temp1 = linspace(startAngle,stopAngle,division+1).tolist()
    if loop == True:
        temp2 = linspace(stopAngle,startAngle,division+1).tolist()
        temp1.pop(-1)
        temp2.pop(-1)
        angleListInCycle = temp1 + temp2
        del temp1
        del temp2
    elif loop == False:
        angleListInCycle = temp1
        del temp1
    for angle in angleListInCycle:
        print('='*50,'\nRotation Angle is:',angle,'degrees\n',)
        if axis == 'x' or axis == 'X':
            A = sixDof.createCoordSystem6(0.,0.,80.,angle,0.,0.)
        elif axis == 'y' or axis == 'Y':
            A = sixDof.createCoordSystem6(0.,0.,80.,0.,angle,0.)
        elif axis == 'z' or axis == 'Z':
            A = sixDof.createCoordSystem6(0.,0.,80.,0.,0.,angle)
        else:
            raise TypeError
        a.calcPltfmPivotCoords(A)
        a.calcCrankAngle()
        for i in range(0,6):
            print 'No.'+str(i+1)+' Servo Angle is '+str("%.2f" % a.crankAngleNow[i])
        if False:
            time.sleep(0.01)
calcCrankAngleSeq(startAngle=-18.,stopAngle=18.,division=2,axis='X',loop = True)
'''
