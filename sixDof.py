##################################################
# sixDof by Ray Dutchman
# 
# Version - v1.0.0
# 
# Credits:
#
# None of the code below is to be redistributed
# or reused without the permission of the
# author(s).
##################################################
import numpy.matlib as ml
from math import *
#add sth.
'''x axis faces toward front, y axis faces toward left, z axis faces upward.'''
class sixDofSimPltfm(object):
	def __init__(self):
		'''Simulator platform definition'''
		self.pltfmPivotDistance = 15.
		self.pltfmPivotRadius = 87.
		self.shaftRadius = 90.0
		self.crankLength = 40.0
		self.rodLength = 100.0
		self.pltfmHeight =108.
		self.maxBearingMisalignment = ml.radians(23.57)
		self.pltfmRefPointCoords = ml.matrix([0.,0.,108,1.]).T
		self.crankAngles0 = []
		self.crankAnglesNow = [.0,.0,.0,.0,.0,.0]
		self.pltfmPosition = coordSystem(originCoords = self.pltfmRefPointCoords,directionMatrix=ml.eye(4))
		self.shaftCoordSystem = []
		self.pivotCoordSystem = []
		self.pltfmPivotCoords0 = []
		self.pltfmPivotCoordSystem = [coordSystem()]*6
		self.pltfmPivotCoordSystemLocal = [coordSystem()]*6
		self.pltfmPivotDirection = [0.,ml.radians(120.),ml.radians(120.),ml.radians(240.),ml.radians(240.),0.]
		self.pltfmPivotCoordsLocal = []
		self.pltfmPivotCoordsNow = []
		self.anglesRodToCrankPlane = [0.]*6
		self.anglesRodToPivotPlane = [0.]*6
		self.crankPivotCoords = [ml.matrix([0.,0.,0.,1.]).T]*6
		self.rodBearingTwist=[[0.,0.,0.]]*6
		#[rodCrankPivotTwist, rodPltfmPivotTwist, rodRotationBetweenPivots]*6

		'''
		self.accG = ml.zeros(6)						#acceleration caused by gravity in local coordinate system.
		self.accMovement = ml.zeros(6)				#acceleration caused by movement of the platform.
		self.accTotal = ml.zeros(6)
		'''
		'''
		Caculate: self.shaftCoordSystem
		'''
		No1BaseShaftRadian = acos(self.pltfmPivotRadius/self.shaftRadius)
		co = lambda x,y:ml.matrix([cos(x)*y,sin(x)*y,0.,1.]).T
		di = lambda z:calcTransformMatrix(radZ=z)
		shaftCoords = [\
			co( No1BaseShaftRadian,self.shaftRadius),\
			co(-No1BaseShaftRadian+120.0/180.0*pi,self.shaftRadius),\
			co( No1BaseShaftRadian+120.0/180.0*pi,self.shaftRadius),\
			co(-No1BaseShaftRadian-120.0/180.0*pi,self.shaftRadius),\
			co( No1BaseShaftRadian-120.0/180.0*pi,self.shaftRadius),\
			co(-No1BaseShaftRadian,self.shaftRadius)]
		shaftDirection = [\
			di(ml.radians(0.)),
			di(ml.radians(120.)),
			di(ml.radians(120.)),
			di(ml.radians(240.)),
			di(ml.radians(240.)),
			di(ml.radians(0.))]
		del co
		del di
		for i in range(0,6):
			self.shaftCoordSystem.append(coordSystem(originCoords=shaftCoords[i],\
		directionMatrix=shaftDirection[i]))
		'''
		==================================================
		Caculate:
			self.pltfmPivotCoords0
			self.pltfmPivotCoordsLocal
			self.pltfmPivotCoordSystemLocal
		Initialize: 
			self.pltfmPivotCoordsNow
		'''
		No1PltfmPivotRadian = atan(self.pltfmPivotDistance/2/self.pltfmPivotRadius)
		g = lambda alpha,h:ml.matrix([\
			cos(alpha)*sqrt(self.pltfmPivotRadius**2+self.pltfmPivotDistance**2/4),\
			sin(alpha)*sqrt(self.pltfmPivotRadius**2+self.pltfmPivotDistance**2/4),\
			h,1.0]).T
		self.pltfmPivotCoords0 = [\
		g( No1PltfmPivotRadian,self.pltfmHeight),\
		g(-No1PltfmPivotRadian+120.0/180.0*pi,self.pltfmHeight),\
		g( No1PltfmPivotRadian+120.0/180.0*pi,self.pltfmHeight),\
		g(-No1PltfmPivotRadian-120.0/180.0*pi,self.pltfmHeight),\
		g( No1PltfmPivotRadian-120.0/180.0*pi,self.pltfmHeight),\
		g(-No1PltfmPivotRadian,self.pltfmHeight)]
		self.pltfmPivotCoordsNow = [\
		g( No1PltfmPivotRadian,self.pltfmHeight),\
		g(-No1PltfmPivotRadian+120.0/180.0*pi,self.pltfmHeight),\
		g( No1PltfmPivotRadian+120.0/180.0*pi,self.pltfmHeight),\
		g(-No1PltfmPivotRadian-120.0/180.0*pi,self.pltfmHeight),\
		g( No1PltfmPivotRadian-120.0/180.0*pi,self.pltfmHeight),\
		g(-No1PltfmPivotRadian,self.pltfmHeight)]
		del g
		for i in self.pltfmPivotCoords0:
			self.pltfmPivotCoordsLocal.append(i-self.pltfmRefPointCoords + ml.matrix([0.,0.,0.,1.]).T)
		for i in range(0,6):
			self.pltfmPivotCoordSystemLocal[i]=\
				coordSystem(originCoords=self.pltfmPivotCoordsLocal[i],\
							directionMatrix=calcTransformMatrix(radZ=self.pltfmPivotDirection[i]))
			self.pltfmPivotCoordSystem[i]=self.pltfmPivotCoordSystemLocal[i]
		'''
		==================================================
		Caculate:
			self.crankAngles0
		Initialize:
			self.crankAnglesNow
		'''
		temp = self.calcCrankAngle()
		for i in temp:
			self.crankAngles0.append(i)
		del temp
	
	def __repr__(self):
		info = \
			'#'*42+\
			'\n# This a 6 DOF Racing Simulator Platfom. #\n'+'#'*42+\
			'\n\nIts Parameters are as follows:\n'\
			'\n    '+"{:<20}".format('pltfmPivotDistance')  +' = '+"{:>6,.1f}".format(self.pltfmPivotDistance)+\
			'\n    '+"{:<20}".format('pltfmPivotRadius')    +' = '+"{:>6,.1f}".format(self.pltfmPivotRadius)+\
			'\n    '+"{:<20}".format('shaftRadius')         +' = '+"{:>6,.1f}".format(self.shaftRadius)+\
			'\n    '+"{:<20}".format('crankLength')         +' = '+"{:>6,.1f}".format(self.crankLength)+\
			'\n    '+"{:<20}".format('rodLength')           +' = '+"{:>6,.1f}".format(self.rodLength)+\
			'\n    '+"{:<20}".format('pltfmHeight')         +' = '+"{:>6,.1f}".format(self.pltfmHeight)+\
			'\n    '+"{:<20}".format('pltfmRefPointCoords') +' = '+\
			"({:>5,.1f},{:>5,.1f},{:>6,.1f})".format(self.pltfmRefPointCoords[0,0],self.pltfmRefPointCoords[1,0],self.pltfmRefPointCoords[2,0])+\
			'\n\n    crankAngles0 =\n'
		for i in range(0,6):
			info += ' '*8+'SERVO Number '+str(i+1)+':'+"{:>7,.2f}".format(self.crankAngles0[i])+' degrees\n'
		info += '    crankAnglesNow =\n'
		for i in range(0,6):
			info += ' '*8+'SERVO Number '+str(i+1)+':'+"{:>7,.2f}".format(self.crankAnglesNow[i])+' degrees\n'
		info += '    pltfmPivotCoords0 =\n'
		for i in range(0,6):
			info = info+' '*8+'Pivot Number '+str(i+1)+': ('+\
			"{:>7,.2f},".format(self.pltfmPivotCoords0[i][0,0])+\
			"{:>7,.2f},".format(self.pltfmPivotCoords0[i][1,0])+\
			"{:>7,.2f})\n".format(self.pltfmPivotCoords0[i][2,0])
		info += '    pltfmPivotCoordsNow =\n'
		for i in range(0,6):
			info += ' '*8+'Pivot Number '+str(i+1)+': ('+\
			"{:>7,.2f},".format(self.pltfmPivotCoordsNow[i][0,0])+\
			"{:>7,.2f},".format(self.pltfmPivotCoordsNow[i][1,0])+\
			"{:>7,.2f})\n".format(self.pltfmPivotCoordsNow[i][2,0])
		info = info + '#'*49 + '\n'
		return info
	def printCoords(self,printObjectName):
		if printObjectName in ('pltfmPivot','pivot','pltfmPivotNow','pivotNow',\
							   'pltfmPivots','pivots','pltfmPivotsNow','pivotsNow'):
			printObjectName = 'pltfmPivotCoordsNow'
			printObject = self.pltfmPivotCoordsNow
		elif printObjectName in ('pltfmPivot0','pivot0','pltfmPivots0','pivots0'):
			printObjectName = 'pltfmPivotCoords0'
			printObject = self.pltfmPivotCoords0
		elif printObjectName in ('pltfmPivotLocal','pivotLocal','pltfmPivotsLocal','pivotsLocal'):
			printObjectName = 'pltfmPivotCoordsLocal'
			printObject = self.pltfmPivotCoordsLocal
		elif printObjectName in ('crankPivot','crankPivotNow','crankPivots','crankPivotsNow'):
			printObjectName = 'crankPivotCoords'
			printObject = self.crankPivotCoords
		else:
			pass
		info = '='*45+'\n'+printObjectName+' =\n'
		for i in range(0,6):
			coords = printObject[i].T.tolist()[0]
			strX="{:>7,.2f}".format(coords[0])
			strY="{:>7,.2f}".format(coords[1])
			strZ="{:>7,.2f}".format(coords[2])
			info += '    Point Number '+str(i+1)+': ('+\
			strX+','+strY+','+strZ+')\n'
		info = info + '='*45 + '\n'
		print(info)
	def printAngles(self,printObjectName):
		if printObjectName in ('crankAnglesNow','crankAngles','crankAngleNow','crankAngle','crank'):
			printObjectName = 'crankAnglesNow'
			printObject = self.crankAnglesNow
		elif printObjectName in ('crankAngles0','crankAngle0','crank0'):
			printObjectName = 'crankAngles0'
			printObject = self.crankAngles0
		elif printObjectName in ('anglesRodToCrankPlane','anglesRod2CrankPlane','rodToCrankPlane','rod2CrankPlane'):
			printObjectName = 'anglesRodToCrankPlane'
			printObject = self.anglesRodToCrankPlane
		else:
			raise TypeError
		info = '='*34 + '\n'+printObjectName+' =\n'
		for i in range(0,6):
			info += "    Angle Number "+str(i+1)+':'+"{:>7,.2f}".format(printObject[i])+' degrees\n'
		info = info + '='*34 + '\n'
		print(info)
	'''
	==================================================
	Input: 
	Modify: 
	Output: 
	'''
	def transformByMatrixGlobal(self, transformMatrix):
		if type(transformMatrix) == ml.matrix and ml.shape(transformMatrix) == (4,4):
			self.pltfmPosition = self.pltfmPosition.transformByMatrixGlobal(transformMatrix)
			#for i in range(0,6):#list contains 6 coordinates
			#	self.pltfmPivotCoordsNow[i] = self.pltfmPosition.calcAbsCoords(self.pltfmPivotCoordsLocal[i])
			self.calcPltfmPivotCoords()
			self.calcCrankAngle()
		else:
			raise TypeError
	def transformByMatrixLocal(self, transformMatrix):
		if type(transformMatrix) == ml.matrix and ml.shape(transformMatrix) == (4,4):
			self.pltfmPosition = self.pltfmPosition.transformByMatrixLocal(transformMatrix)
			self.calcPltfmPivotCoords()
			self.calcCrankAngle()
		else:
			raise TypeError
	def transformByCoordSystemGlobal(self, transformCoordSystem):
		if type(transformMatrix) == coordSystem:
			self.pltfmPosition = self.pltfmPosition+transformCoordSystem
			self.calcPltfmPivotCoords()
			self.calcCrankAngle()
		else:
			raise TypeError
	def transformByCoordSystemLocal(self, transformMatrix):
		if type(transformMatrix) == coordSystem:
			self.pltfmPosition = self.pltfmPosition*transformCoordSystem
			self.calcPltfmPivotCoords()
			self.calcCrankAngle()
		else:
			raise TypeError
	'''
	==================================================
	'''
	def transform(self,change,changeType='Global'):
		if type(change)==ml.matrix and ml.shape(change)==(4,4):
			if changeType=='Global':
				self.pltfmPosition.transformByMatrixGlobal(change)
			elif changeType=='Local':
				self.pltfmPosition.transformByMatrixLocal(change)
			else:
				raise TypeError
		elif type(change)==coordSystem:
			if changeType=='Global':
				self.pltfmPosition = self.pltfmPosition + change
			elif changeType=='Local':
				self.pltfmPosition = self.pltfmPosition * change
			else:
				raise TypeError
		else:
			raise TypeError
		self.calcPltfmPivotCoords()
		self.calcCrankAngle()
	'''
	==================================================
	Input: pltfmPosition as a coordSystem class or 4*4 matrix,
		will change self.pltfmPosition if input a coordSystem
	Modify: self.pltfmPivotCoordsNow
	Output: None
	'''
	def calcPltfmPivotCoords(self, pltfmPosition = None):
		if type(pltfmPosition) == ml.matrix and ml.shape(pltfmPosition) == (4,4):
			for i in range(0,6):#list contains 6 coordinates
				self.pltfmPivotCoordsNow[i]=pltfmPosition*self.pltfmPivotCoordsLocal[i]
		elif type(pltfmPosition) == coordSystem:
			self.pltfmPosition = pltfmPosition
			for i in range(0,6):#list contains 6 coordinates
				self.pltfmPivotCoordsNow[i]=pltfmPosition.calcAbsCoords(coords=self.pltfmPivotCoordsLocal[i])
				self.pltfmPivotCoordSystem[i]=pltfmPosition*self.pltfmPivotCoordSystemLocal[i]
		else:
			for i in range(0,6):#list contains 6 coordinates
				self.pltfmPivotCoordsNow[i]=self.pltfmPosition.calcAbsCoords(coords=self.pltfmPivotCoordsLocal[i])
				self.pltfmPivotCoordSystem[i]=self.pltfmPosition*self.pltfmPivotCoordSystemLocal[i]
	'''
	==================================================
	'''
	
	'''
	==================================================
	Input: None, 
	Read: self.shaftCoordSystem and self.pltfmPivotCoordsNow
	Modify: self.crankAnglesNow
	Output: self.crankAnglesNow
	'''
	def calcCrankAngle(self):
		mathDomainError = False
		pPivotCoordsInShaftCoordSystem = []
		for i in range(0,6):
			pPivotCoordsInShaftCoordSystem.append(\
				self.shaftCoordSystem[i].calcRltCoords(self.pltfmPivotCoordsNow[i]))
		if False:
			for i in pPivotCoordsInShaftCoordSystem:
				print('='*50,'\npPivotCoordsInShaftCoordSystem: ','\n',i,'\n')
		for i in range(0,6):
			rodDistanceToYZPlane = pPivotCoordsInShaftCoordSystem[i][(0,0)]
			pivotShaftProjSquare = (\
				(pPivotCoordsInShaftCoordSystem[i][(1,0)])**2+
				(pPivotCoordsInShaftCoordSystem[i][(2,0)])**2)
			rodLengthProjSquare = self.rodLength**2-rodDistanceToYZPlane**2
			cosBeta = (self.crankLength**2+pivotShaftProjSquare-rodLengthProjSquare)\
					  /(2*self.crankLength*sqrt(pivotShaftProjSquare))
			#print(i,'\ncos(beta) = ',cosBeta,'\n','='*50)
			if cosBeta>=-1. and cosBeta<=1.: 
				beta = acos(cosBeta)
				alpha = acos(pPivotCoordsInShaftCoordSystem[i][(1,0)]/sqrt(pivotShaftProjSquare))
				if i==0 or i==2 or i==4:
					singleCrankAngle = ml.degrees(alpha - beta)
				elif i==1 or i==3 or i==5:
					singleCrankAngle = ml.degrees(alpha + beta)
				self.crankAnglesNow[i] = singleCrankAngle
				self.anglesRodToCrankPlane[i]=ml.degrees(asin(rodDistanceToYZPlane/self.rodLength))
				singleCrankPivotCoordsLocal=ml.matrix([\
					[0.],\
					[self.crankLength*cos(ml.radians(singleCrankAngle))],\
					[self.crankLength*sin(ml.radians(singleCrankAngle))],\
					[1.]])
				self.crankPivotCoords[i]=self.shaftCoordSystem[i].calcAbsCoords(singleCrankPivotCoordsLocal)
			else:
				mathDomainError = True
				if False:
					print('No.'+str(i+1)+' Actuator Error','\n')
				#alpha = 0
				#beta = 0
		if mathDomainError == False:	
			answer=[]
			for i in self.crankAnglesNow:
				answer.append(i)
			return answer
		else:
			return False
	'''
	==================================================
	'''
	def calcBearingAngle(self):
		for i in range(0,6):
			self.rodBearingTwist[i] = \
				calcRodBearingAngle(shaftCoordSystem=self.shaftCoordSystem[i],\
									pltfmPivotCoordSystem=self.pltfmPivotCoordSystem[i],\
									crankPivotCoords=self.crankPivotCoords[i],\
									pltfmPivotCoordsNow=self.pltfmPivotCoordSystem[i].originCoords)
	def checkBearingAngle(self):
		checkResult=[True]*6
		for i in range(0,6):	
			twist1 = abs(self.rodBearingTwist[i][0])
			twist2 = abs(self.rodBearingTwist[i][1])
			if twist1 > self.maxBearingMisalignment or twist2 > self.maxBearingMisalignment:
				checkResult[i]=False
			else:
				rotaion = self.rodBearingTwist[i][2]
				allowedRotation1=acos(cos(self.maxBearingMisalignment)/cos(twist1))
				allowedRotation2=acos(cos(self.maxBearingMisalignment)/cos(twist2))
				if allowedRotation1+allowedRotation2<rotaion:
					checkResult[i]=False
		return checkResult

class coordSystem(object):
	def __init__(self,originCoords=ml.matrix([[0.],[0.],[0.],[1.]]),\
		directionMatrix=ml.matrix([[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]])):
		self.originCoords = ml.matrix([[0.],[0.],[0.],[1.]])#1*4 Matrix
		self.directionMatrix = ml.matrix([[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]])#4*4 Matrix
		if ((type(originCoords)==list or type(originCoords)==tuple)) \
			and (ml.shape(originCoords)==(4,) or ml.shape(originCoords)==(3,)):
			originCoords=ml.matrix(originCoords).T
		if type(originCoords)==ml.matrix:
			size = ml.shape(originCoords)
			if size==(1,4):
				self.originCoords=originCoords.T
			elif size==(4,1):
				self.originCoords=originCoords
			elif size==(1,3):
				self.originCoords=ml.vstack(originCoords.T,ml.eye(1))
			elif size==(3,1):
				self.originCoords=ml.vstack(originCoords,ml.eye(1))
		else:
			raise TypeError
		if ((type(directionMatrix)==list or type(directionMatrix)==tuple)) \
			and (ml.shape(directionMatrix)==(4,4) or ml.shape(directionMatrix)==(3,3)):
			directionMatrix = ml.matrix(directionMatrix)
		if type(directionMatrix)==ml.matrix:
			size = ml.shape(directionMatrix)
			if size==(4,4):
				self.directionMatrix=directionMatrix
			elif size==(3,3):
				self.directionMatrix=ml.eye(4)
				self.directionMatrix[0:3,0:3]=directionMatrix
		else:
			raise TypeError
	def __repr__(self):
		if False:
			return '\nCoordinates are:\n'+str(self.originCoords[0:3,0])+\
				   '\nDirection Matrix is:\n'+str(self.directionMatrix[0:3,0:3])
		else:
			return '\nCoordinates are:\n'+str(self.originCoords)+\
				   '\nDirection Matrix is:\n'+str(self.directionMatrix)
	def __add__(self,other):
		#Generate a new coordSystem, which is:
		#coordSystem A transformed by a global CoordSystem B.
		newOriginCoords = self.originCoords + other.originCoords + ml.matrix([[0.],[0.],[0.],[1.]])
		newDirectionMatrix = other.directionMatrix*self.directionMatrix
		return coordSystem(originCoords=newOriginCoords,\
						   directionMatrix=newDirectionMatrix)
	def __sub__(self,other):
		newOriginCoords = self.originCoords - other.originCoords + ml.matrix([[0.],[0.],[0.],[-1.]])
		newDirectionMatrix = other.directionMatrix.I*self.directionMatrix
		return coordSystem(originCoords=newOriginCoords,\
						   directionMatrix=newDirectionMatrix)
	def __mul__(self,other):
		#Generate a new coordSystem, which is:
		#coordSystem A transformed by a A's local coordSystem B
		newOriginCoords = self.calcAbsCoords(other.originCoords)
		newDirectionMatrix = self.directionMatrix*other.directionMatrix
		return coordSystem(originCoords=newOriginCoords,\
						   directionMatrix=newDirectionMatrix)
	def __div__(self,other):
		newOriginCoords = self.calcAbsCoords(-other.originCoords)
		newDirectionMatrix = self.directionMatrix*other.directionMatrix.I
		return coordSystem(originCoords=newOriginCoords,\
						   directionMatrix=newDirectionMatrix)
	def __setitem__(self,key=None,var=None):
		pass
	def __getitem__(self,key):
		pass
	def printAngles(self):
		print('='*42)
		print('Angles bewteen YZ plane is'+"{:>7,.2f}".format(ml.degrees(acos(self.directionMatrix[0,0])))+' degrees;')
		print('Angles bewteen ZX plane is'+"{:>7,.2f}".format(ml.degrees(acos(self.directionMatrix[1,1])))+' degrees;')
		print('Angles bewteen XY plane is'+"{:>7,.2f}".format(ml.degrees(acos(self.directionMatrix[2,2])))+' degrees.')
		print('='*42+'\n')
	def transformByMatrixGlobal(self,transformMatrix):
		#CoordSystem transform by a transform Matrix in global coord system.
		self.originCoords = self.originCoords + ml.vstack([transformMatrix[0:3,3],ml.zeros((1,1))])
		directionChange = ml.eye(4)
		directionChange[0:3,0:3] = transformMatrix[0:3,0:3]
		self.directionMatrix = directionChange*self.directionMatrix
		return self
	def transformByMatrixLocal(self,transformMatrix):
		#CoordSystem transform by a transform matrix in its local coord system.
		self.originCoords = self.originCoords + self.directionMatrix*ml.vstack([transformMatrix[0:3,3],ml.zeros((1,1))])
		directionChange = ml.eye(4)
		directionChange[0:3,0:3] = transformMatrix[0:3,0:3]
		self.directionMatrix = self.directionMatrix*directionChange
		return self
	def calcAbsCoords(self,coords):
		#Coordinates in local coordSystem is known, calculate global coordinates.
		T = ml.hstack([ml.vstack([ml.eye(3),ml.zeros([1,3])]),self.originCoords])
		ans = T*self.directionMatrix*coords
		return ans
	def calcRltCoords(self,coords):
		#Global coordinates is known, calculate coordinates in local coordSystem.
		T = ml.hstack([ml.vstack([ml.eye(3),ml.zeros([1,3])]),-self.originCoords])
		T[(3,3)]=1
		ans = self.directionMatrix.I*T*coords
		return ans
def createCoordSystem(x=0.0,y=0.0,z=0.0,angleX=0.,angleY=0.,angleZ=0.):
	a = calcTransformMatrix(radX=angleX,radY=angleY,radZ=angleZ,degree=True)
	return coordSystem(ml.matrix([x,y,z,1.]).T,a)
def createCoordSystemByMatrix(matrix=ml.eye(4)):
	originCoords = matrix[0:4,3]
	directionMatrix = matrix
	directionMatrix[0:4,3]=ml.matrix([[0.],[0.],[0.],[1.]])
	return coordSystem(originCoords,directionMatrix)
'''Rotaion Sequence: First: X Axis, Second: Y Axis, Third: Z Z axis.'''
def calcTransformMatrix(x=0.0,y=0.0,z=0.0,radX=0.,radY=0.,radZ=0.,degree = False):
	if degree == True:
		radX = ml.radians(radX)
		radY = ml.radians(radY)
		radZ = ml.radians(radZ)
	T = ml.matrix([\
		[1., 0., 0., x ],\
		[0., 1., 0., y ],\
		[0., 0., 1., z ],\
		[0., 0., 0., 1.]])
	RX = ml.matrix([\
		[1.,       0.,        0.,0.],\
		[0.,cos(radX),-sin(radX),0.],\
		[0.,sin(radX), cos(radX),0.],\
		[0.,       0.,        0.,1.]])
	RY = ml.matrix([\
		[ cos(radY),0.,sin(radY),0.],\
		[0.        ,1.,       0.,0.],\
		[-sin(radY),0.,cos(radY),0.],\
		[0.,       0.,        0.,1.]])
	RZ = ml.matrix([\
		[cos(radZ),-sin(radZ),0.,0.],\
		[sin(radZ), cos(radZ),0.,0.],\
		[0.       , 0.       ,1.,0.],\
		[0.       , 0.       ,0.,1.]])
	M = T*RZ*RY*RX
	return M
def calcTransform(pointIn=ml.matrix([[0.], [0.], [0.], [1.]]),\
				  x=0.0,y=0.0,z=0.0,radX=0.,radY=0.,radZ=0.,degree = False):
	if degree == True:
		radX = ml.radians(radX)
		radY = ml.radians(radY)
		radZ = ml.radians(radZ)
	T = ml.matrix([\
		[1., 0., 0., x ],\
		[0., 1., 0., y ],\
		[0., 0., 1., z ],\
		[0., 0., 0., 1.]])
	RX = ml.matrix([\
		[1.,       0.,        0.,0.],\
		[0.,cos(radX),-sin(radX),0.],\
		[0.,sin(radX), cos(radX),0.],\
		[0.,       0.,        0.,1.]])
	RY = ml.matrix([\
		[ cos(radY),0.,sin(radY),0.],\
		[0.        ,1.,       0.,0.],\
		[-sin(radY),0.,cos(radY),0.],\
		[0.,       0.,        0.,1.]])
	RZ = ml.matrix([\
		[cos(radZ),-sin(radZ),0.,0.],\
		[sin(radZ), cos(radZ),0.,0.],\
		[0.       , 0.       ,1.,0.],\
		[0.       , 0.       ,0.,1.]])
	M = T*RZ*RY*RX
	return M*pointIn
def calcInverseMatrix(x=0.0,y=0.0,z=0.0,\
					  radX=0.,radY=0.,radZ=0.,degree = False):
	if degree == True:
		radX = ml.radians(radX)
		radY = ml.radians(radY)
		radZ = ml.radians(radZ)
	T = ml.matrix([\
		[1., 0., 0.,-x ],\
		[0., 1., 0.,-y ],\
		[0., 0., 1.,-z ],\
		[0., 0., 0., 1.]])
	RX = ml.matrix([\
		[1.,       0.,         0.,0.],\
		[0., cos(radX), sin(radX),0.],\
		[0.,-sin(radX), cos(radX),0.],\
		[0.,        0.,        0.,1.]])
	RY = ml.matrix([\
		[cos(radY),0.,-sin(radY),0.],\
		[0.       ,1.,        0.,0.],\
		[sin(radY),0., cos(radY),0.],\
		[0.,      0.,         0.,1.]])
	RZ = ml.matrix([\
		[ cos(radZ),sin(radZ),0.,0.],\
		[-sin(radZ),cos(radZ),0.,0.],\
		[0.        ,0.       ,1.,0.],\
		[0.        ,0.       ,0.,1.]])
	M = RX*RY*RZ*T
	return M
'''Rotaion Sequence: First: X Axis, Second: Y Axis, Third: Z Z axis.'''
def calcInverse(pointIn=ml.matrix([[0.], [0.], [0.], [1.]]),\
				x=0.0,y=0.0,z=0.0,radX=0.,radY=0.,radZ=0.,degree = False):
	if degree == True:
		radX = ml.radians(radX)
		radY = ml.radians(radY)
		radZ = ml.radians(radZ)
	T = ml.matrix([\
		[1., 0., 0.,-x ],\
		[0., 1., 0.,-y ],\
		[0., 0., 1.,-z ],\
		[0., 0., 0., 1.]])
	RX = ml.matrix([\
		[1.,       0.,         0.,0.],\
		[0., cos(radX), sin(radX),0.],\
		[0.,-sin(radX), cos(radX),0.],\
		[0.,        0.,        0.,1.]])
	RY = ml.matrix([\
		[cos(radY),0.,-sin(radY),0.],\
		[0.       ,1.,        0.,0.],\
		[sin(radY),0., cos(radY),0.],\
		[0.,      0.,         0.,1.]])
	RZ = ml.matrix([\
		[ cos(radZ),sin(radZ),0.,0.],\
		[-sin(radZ),cos(radZ),0.,0.],\
		[0.        ,0.       ,1.,0.],\
		[0.        ,0.       ,0.,1.]])
	M = RX*RY*RZ*T
	return M*pointIn

def calcCoordSystemByTwoPoints(\
	pointA = ml.matrix([[0.],[0.],[0.],[1.]]),\
	pointB = ml.matrix([[0.],[0.],[1.],[1.]])):
	originCoords=pointA
	pointA=pointA[0:3,0]
	pointB=pointB[0:3,0]
	unit = lambda x:x/ml.linalg.norm(x)
	cross = lambda x,y:ml.cross(x.T,y.T).T
	vector3=pointB-pointA
	vector1=cross(ml.matrix([[0.],[1.],[0.]]),vector3)
	vector2=cross(vector3,vector1)
	vector1=unit(vector1)
	vector2=unit(vector2)
	vector3=unit(vector3)
	directionMatrix=ml.eye(4)
	directionMatrix[0:3,0:3]=ml.hstack([vector1,vector2,vector3])
	return coordSystem(originCoords,directionMatrix)
def calcRodBearingAngle(shaftCoordSystem,pltfmPivotCoordSystem,crankPivotCoords,pltfmPivotCoordsNow):
	pointA=crankPivotCoords
	pointB=pltfmPivotCoordsNow
	rodCoordSystem=calcCoordSystemByTwoPoints(ml.matrix([[0.],[0.],[0.],[1.]]),pointB-pointA)
	shaftDirection=shaftCoordSystem.directionMatrix[0:4,0]
	pltfmPivotDirection=pltfmPivotCoordSystem.directionMatrix[0:4,0]
	shaftDirectionLocal=rodCoordSystem.calcRltCoords(shaftDirection)
	pltfmPivotDirectionLocal=rodCoordSystem.calcRltCoords(pltfmPivotDirection)
	rodCrankPivotTwist=asin(shaftDirectionLocal[2,0])
	rodPltfmPivotTwist=asin(pltfmPivotDirectionLocal[2,0])
	def rad(x,y):
		cosAngle = (x.T*y)[0,0]/(ml.linalg.norm(x)*ml.linalg.norm(y))
		gap = 1e-13
		if cosAngle>1.0-gap and cosAngle<1.0+gap:
			return 0.
		elif cosAngle<-1.0+gap and cosAngle>-1.0-gap:
			return 180.
		else:
			return acos(cosAngle)
	rodRotationBetweenPivots=rad(shaftDirectionLocal[0:2,0],pltfmPivotDirectionLocal[0:2,0])
	return [rodCrankPivotTwist,rodPltfmPivotTwist,rodRotationBetweenPivots]