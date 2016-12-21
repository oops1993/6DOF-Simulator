import sixDof
import numpy.matlib as ml
import numpy as np
import xlwt
from datetime import datetime
from math import *
reload(sixDof)
answer=[]
#trueNum=0
#falseNum=0
a = sixDof.sixDofSimPltfm()
for i in np.linspace(0,40,41):
    print 'x =',i,' \t',datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    for j in np.linspace(0,40,41):
        for k in np.linspace(108,148,41):
            pltfmPosition=sixDof.createCoordSystem(x=i,y=j,z=k,angleX=0.,angleY=0.,angleZ=0.)
            a.calcPltfmPivotCoords(pltfmPosition)
            if a.calcCrankAngle()!=False:
                #trueNum=trueNum+1
                #print 'True NO.',trueNum,"{0}/{1}/{2}".format(i,j,k)
                #a.printAngles('crank')
                answer.append([i,j,k])
                pass
            elif a.calcCrankAngle()==False:
                #answer.append([i,j,k-1])
                #falseNum=falseNum+1
                #print 'False NO.',falseNum
                pass
answer2=[]
for i in range(0,len(answer)-1):
    if answer[i][0]!=answer[i+1][0] or answer[i][1]!=answer[i+1][1]:
        answer2.append(answer[i])
answer2.append(answer[-1])
style0 = xlwt.easyxf('font: name Times New Roman,\
    color-index black, bold off',num_format_str='#,##0.00')
style1 = xlwt.easyxf('font: name Times New Roman, color-index black, bold off',
    num_format_str='#,##0')
style2 = xlwt.easyxf(num_format_str='D-MMM-YY')
wb = xlwt.Workbook()
ws = wb.add_sheet('Answer')
ws.write(0, 0, 'x(mm)' , style0)
ws.write(0, 1, 'y(mm)' , style0)
ws.write(0, 2, 'z(mm)' , style0)
for i in range(0,len(answer2)):
    ws.write(i+1, 0, answer2[i][0], style1)
    ws.write(i+1, 1, answer2[i][1], style1)
    ws.write(i+1, 2, answer2[i][2], style1)
wb.save('Answer2.xls')
