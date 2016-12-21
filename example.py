import sixDof
import numpy.matlib as ml
A=sixDof.calcTransformMatrix(0,0,0,ml.radians(0),ml.radians(0),ml.radians(45))
B=sixDof.calcTransformMatrix(0,0,0,ml.radians(45),ml.radians(0),ml.radians(0))
C=sixDof.calcTransformMatrix(0,0,0,ml.radians(0),ml.radians(0),ml.radians(-20))
D=sixDof.calcTransformMatrix(0,0,0,ml.radians(45),ml.radians(0),ml.radians(20))
a=sixDof.coordSystem(originCoords=ml.matrix([[0.],[0.],[0.],[1.]]),\
              directionMatrix=A)
#print a
a=sixDof.createCoordSystem(0,0,0,0,0,45)
#print a
b=sixDof.coordSystem(originCoords=ml.matrix([[10.],[0.],[0.],[1.]]),\
              directionMatrix=B)
c=sixDof.coordSystem(originCoords=ml.matrix([[5.],[5.],[5.],[1.]]),\
              directionMatrix=C)
#print '='*50,'\na+b=\n',a+b
#print '='*50,'\na-b=\n',a-b
#print '='*50,'\na*b=\n',a*b
#print D
#print '='*50,'\na/b=\n',a/b
pointA=ml.matrix([0,0,0,1.]).T
pointB=ml.matrix([10,10,100.,1.]).T
a=sixDof.calcCoordSystemByTwoPoints(pointA,pointB)
print a
