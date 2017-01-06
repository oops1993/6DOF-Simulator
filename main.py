import sixDof
reload(sixDof)
from math import *
import time
import pickle_learning
import numpy.matlib as ml
import servo

a = sixDof.sixDofSimPltfm()
print a

Rot1 = sixDof.calcTransformMatrix(\
    0.,0.,0.,0.,0.,0.,degree=True)
Rot2 = sixDof.calcTransformMatrix(\
    1.,0.,0.,0.,0.,0.,degree=True)
Rot3 = sixDof.calcTransformMatrix(\
    -1.,0.,0.,0.,0.,0.,degree=True)

a.transform(Rot1,'Global')

if 0:
    a.printCoords('pivots')
    a.printAngles('crank')
    a.calcBearingAngle()
    print a.checkBearingAngle(),'\n'
    print a.pltfmPosition

servo.servoInitialize()
servo.changeCrankAngle(a.crankAnglesNow)
#time.sleep(1)

if 1:
    for _ in range(2):
        for _ in range(20):
            a.transform(Rot2,'Global')
            servo.changeCrankAngle(a.crankAnglesNow)
            try:
                pickle_learning.sendData("<ffffff",a.crankAnglesNow)
                print("({0:>7,.2f},{1:>7,.2f},{2:>7,.2f},{3:>7,.2f},{4:>7,.2f},{5:>7,.2f})"\
                      .format(a.crankAnglesNow[0],a.crankAnglesNow[1],\
                              a.crankAnglesNow[2],a.crankAnglesNow[3],\
                              a.crankAnglesNow[4],a.crankAnglesNow[5]))
            except Exception,e:
                print str(e)
            #time.sleep(0.02)
        for _ in range(40):
            a.transform(Rot3,'Local')
            servo.changeCrankAngle(a.crankAnglesNow)
            try:
                pickle_learning.sendData("<ffffff",a.crankAnglesNow)
                print("({0:>7,.2f},{1:>7,.2f},{2:>7,.2f},{3:>7,.2f},{4:>7,.2f},{5:>7,.2f})"\
                      .format(a.crankAnglesNow[0],a.crankAnglesNow[1],\
                              a.crankAnglesNow[2],a.crankAnglesNow[3],\
                              a.crankAnglesNow[4],a.crankAnglesNow[5]))
            except Exception,e:
                print str(e)
            #time.sleep(0.02)
    print 'Done...'

lock =  threading.Lock()
def receiveJob(i):
    temp = []
    for j in np.linspace(0,20,21):
        for k in np.linspace(108,128,21):
            pltfmPosition=sixDof.createCoordSystem(x=i,y=j,z=k,angleX=0.,angleY=0.,angleZ=0.)
            a.calcPltfmPivotCoords(pltfmPosition)
            if a.calcCrankAngle()!=False:
                #trueNum=trueNum+1
                #print 'True NO.',trueNum,"{0}/{1}/{2}".format(i,j,k)
                #a.printAngles('crank')
                temp.append([i,j,k])
                pass
            elif a.calcCrankAngle()==False:
                #answer.append([i,j,k-1])
                #falseNum=falseNum+1
                #print 'False NO.',falseNum
                pass
    with lock:    
        print threading.current_thread().name,i
        print 'x =',i,' \t',datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return temp
def threader():
    global answer
    while True:
        i = q.get()
        answer.extend(exampleJob(i))
        q.task_done()
    
q = Queue()

for _ in range(4):
    t = threading.Thread(target = threader)
    t.daemon = True
    t.start()
start = time.time()
for i in np.linspace(0,40,41):
    q.put(i)
q.join()

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

    print('Moving servo on channel 0, press Ctrl+C to quit...')
    while True:
        # Move servo on channel O between extremes.
        pwm.set_pwm(0, 0, servo_min)
        time.sleep(1)
        pwm.set_pwm(0, 0, servo_max)
        time.sleep(1)
