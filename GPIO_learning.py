import servo
import time
a=servo.servo(pinNumber=17,initialAngle=90.)
time.sleep(3)
a.startServo()
time.sleep(3)
a.changeAngle(120.)
time.sleep(3)
a.changeAngle(60.)
time.sleep(3)
a.stop()