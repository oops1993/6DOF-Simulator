import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
#GPIO.PWM: ChangeDutyCycle,ChangeFrequency, Start, Stop
class servo(GPIO.PWM):
	def __init__(self,pinNumber=int,initialAngle=float):
		self.angleLimit=[0.,180.]
		self.initialAngle=initialAngle
		self.servoFrequency=50.
		self.pinNumber=pinNumber
		GPIO.setup(self.pinNumber, GPIO.OUT, initial=GPIO.LOW)
		super(servo, self).__init__(self.pinNumber,self.servoFrequency)
		self.dutyCycleLimit=[2.5,12.5]
	def calcDutyCycle(self,angle):
		angleBand = self.angleLimit[1]-self.angleLimit[0]
		dutyCycleBand = self.dutyCycleLimit[1]-self.dutyCycleLimit[0]
		dutyCycle = (angle-self.angleLimit[0])/angleBand*dutyCycleBand+self.dutyCycleLimit[0]
		return dutyCycle
	def startServo(self):
		dutyCycle = self.calcDutyCycle(self.initialAngle)
		self.start(dutyCycle)
	def stopServo(self):
		self.stop()
	def changeAngle(self,angle):
		dutyCycle=self.calcDutyCycle(angle)
		self.ChangeDutyCycle(dutyCycle)