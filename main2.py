import threading
from Queue import Queue
import sixDof
import numpy.matlib as ml
import numpy as np
import xlwt
import time
#import pickle_learning
from datetime import datetime
from math import *
reload(sixDof)
answer=[]
#trueNum=0
#falseNum=0
a = sixDof.sixDofSimPltfm()

lock =  threading.Lock()
def exampleJob(i):
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
print 'Using Thread, Entire job took:',time.time()-start,'sec(s).'
'''
answer = []
start = time.time()
for i in np.linspace(0,20,5):
    print 'x =',i,' \t',datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    #pickle_learning.sendData("<f",(i,))
    for j in np.linspace(0,20,5):
        for k in np.linspace(108,128,5):
            pltfmPosition=sixDof.createCoordSystem(x=i,y=j,z=k,angleX=0.,angleY=0.,angleZ=0.)
            a.calcPltfmPivotCoords(pltfmPosition)
            time.sleep(0.005)
            if a.calcCrankAngle()!=False:
                #trueNum=trueNum+1
                #print 'True NO.',trueNum,"{0}/{1}/{2}".format(i,j,k)
                #a.printAngles('crank')
                answer.append([i,j,k])
                #print a.crankAnglesNow 
                try:
                    pickle_learning.sendData("<ffffff",a.crankAnglesNow)
                except Exception,e:
                    print str(e)
            else:
                #answer.append([i,j,k-1])
                #falseNum=falseNum+1
                #print 'False NO.',falseNum
                 pass
'''
answer2=[]
for i in range(0,len(answer)-1):
    if answer[i][0]!=answer[i+1][0] or answer[i][1]!=answer[i+1][1]:
        answer2.append(answer[i])
answer2.append(answer[-1])
style0 = xlwt.easyxf('font: name Times New Roman,\
    color-index black, bold off',num_format_str='#,##0.00')
style1 = xlwt.easyxf('font: name Times New Roman, color-index black, bold off',
    num_format_str='#,##0')
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
'''
import threading
from queue import Queue
import time

print_lock =  threading.Lock()

def exampleJob(worker):
    time.sleep(0.5)

    with print_lock:
        print(threading.current_thread().name,worker)

def threader():
    while True:
        worker = q.get()
        exampleJob(worker)
        q.task_done()
    
q = Queue()

for _ in range(10):
    t = threading.Thread(target = threader)
    t.daemon = True
    t.start()
start = time.time()

for worker in range(20):
    q.put(worker)

q.join()
print('Entire job took:',time.time()-start)
'''
