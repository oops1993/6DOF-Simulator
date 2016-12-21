import numpy as np
from math import *
import os
'''x axis faces toward front, y axis faces toward left, z axis faces upward.'''
#def angle2radian(angle=0.0):
	#return angle/180*pi()
class six_dof_pltfm(object):
	def __init__(self):
		'''Base definition'''
		self.pltfmPivotDistance = 150.
		self.pltfmPivotRadius = 780.
		self.baseShaftCenterRadius = 1000.0
		self.baseCrankLength = 300.0
		self.pltfmHeight =800.0
		self.pltfmPivotDistance = 150.0
		self.pltfmPivotRadius = 780.0
		self.basePivotsCoords = np.zeros((6,6))			#6 points, every one has 6 coords, x, y, z, phi x, phi y, phi z
		self.pltfmRefPointCoords = np.zeros(6)			#6 point, 6coords, x, y, z, phi x, phi y, phi z
		self.pltfmPivotsCoords = np.zeros((6,3))
		self.accG =  np.zeros(6)				#acceleration caused by gravity in local coordinate system.
		self.accMovement = np.zeros(6)		#acceleration caused by movement of the platform.
		self.accTotal = np.zeros(6)
		

	def calcParameters(self):#This refers to the distance between platform center and crossline of two nearby pivots.
		No1BasePivotRadian = 60.0/180.0*pi-acos(self.pltfmPivotRadius/self.baseShaftCenterRadius)
		f = lambda x,y,z:np.array([cos(x)*y,sin(x)*y,0.0,0.0,0.0,z])
		No1BasePivotCoods = f( No1BasePivotRadian,self.baseShaftCenterRadius,60.0/180.0*pi)
		No2BasePivotCoods = f(-No1BasePivotRadian+120.0/180.0*pi,self.baseShaftCenterRadius,60.0/180.0*pi)
		No3BasePivotCoods = f( No1BasePivotRadian+120.0/180.0*pi,self.baseShaftCenterRadius,pi)
		No4BasePivotCoods = f(-No1BasePivotRadian-120.0/180.0*pi,self.baseShaftCenterRadius,pi)
		No5BasePivotCoods = f( No1BasePivotRadian-120.0/180.0*pi,self.baseShaftCenterRadius,300/180.0*pi)
		No6BasePivotCoods = f(-No1BasePivotRadian,self.baseShaftCenterRadius,300/180.0*pi)
		self.basePivotsCoords = np.vstack((\
			No1BasePivotCoods,\
			No2BasePivotCoods,\
			No3BasePivotCoods,\
			No4BasePivotCoods,\
			No5BasePivotCoods,\
			No6BasePivotCoods))
		self.pltfmRefPointCoords = np.array([0.,0.,pltfmHeight,0.,0.,0.])
		
		self.pltfmRefPointCoords = np.array([100.,1000.,800.,0.,0.,0.,30.0/180*pi])
		
		No1PltfmPivotRadian = asin(self.pltfmPivotDistance/2/self.pltfmPivotRadius)
		g = lambda alpha,r,h:np.array([cos(alpha)*r,sin(alpha)*r,h])
		No1PltfmPivotCoods = g( No1PltfmPivotRadian,self.pltfmPivotRadius,60.0/180.0*pi)
		No2PltfmPivotCoods = g(-No1PltfmPivotRadian+120.0/180.0*pi,self.pltfmPivotRadius,60.0/180.0*pi)
		No3PltfmPivotCoods = g( No1PltfmPivotRadian+120.0/180.0*pi,self.pltfmPivotRadius,pi)
		No4PltfmPivotCoods = g(-No1PltfmPivotRadian-120.0/180.0*pi,self.pltfmPivotRadius,pi)
		No5PltfmPivotCoods = g( No1PltfmPivtRadian-120.0/180.0*pi,self.pltfmPivotRadius,300/180.0*pi)
		No6PltfmPivotCoods = g(-No1PltfmPivotRadian,self.pltfmPivotRadius,300/180.0*pi)
		self.pltfmPivotsCoords = np.vstack((\
			No1PltfmPivotCoods,\
			No2PltfmPivotCoods,\
			No3PltfmPivotCoods,\
			No4PltfmPivotCoods,\
			No5PltfmPivotCoods,\
			No6PltfmPivotCoods))