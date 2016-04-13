#!/usr/bin/python
#will plot the distance, speed and acceleration of robot

import matplotlib.pyplot as plt
from os import listdir
leftDistance=[]
rightDistance=[]
distance=[]
difference=[]
time=[]
leftPower=[]
rightPower=[]

b=listdir(".")
i=0
for d in b:
    print str(i) + ':' + d
    i +=1
fileName = b[int(raw_input("file containing data:"))]

output=open(fileName,'r')

i=0;
imod=int(raw_input("imod:"))
counter=0
for line in output:
    numStr=""
    numLine=False
    for char in line:
        if char.isdigit() or char=='-':
            numStr+=char 
            numLine=True
    if numLine:
        print str(counter) + ',' + str(i) + ':' + numStr
        if i==0:
            leftDistance.append(int(numStr))
            distance.append(int(numStr))
        if i==1:
            rightDistance.append(int(numStr))
            difference.append((distance[counter]-int(numStr))/2.0)
            distance[counter]=(distance[counter]+int(numStr))/4.0
            if(counter>0 and distance[counter]-distance[counter-1]<0):
                print "distance decreasing"
                print distance[counter]
                print distance[counter-1]
                exit()
        elif i==2:
            time.append((float(numStr)/1000000))
        elif i==3:
            leftPower.append(int(numStr))
        elif i==4:
            rightPower.append(int(numStr))
        if i==imod-1:
            counter+=1
        i=(i+1)%imod

speed=[]
speedTimes=[]
for i in range(1,len(distance)):
    speed.append((distance[i]-distance[i-1])/(time[i]-time[i-1]))
    speedTimes.append((time[i]+time[i-1])/2)

acceleration=[]
accelerationTimes=[]
for i in range(1,len(speed)):
    acceleration.append((speed[i]-speed[i-1])/(speedTimes[i]-speedTimes[i-1]))
    accelerationTimes.append((speedTimes[i]+speedTimes[i-1])/2)

plt.plot(time,distance)
plt.ylabel('distance(degrees)')
plt.xlabel('time')
plt.show()

plt.plot(time,difference)
plt.ylabel('difference(degrees)')
plt.xlabel('time')
plt.show()

plt.plot(speedTimes,speed)
plt.ylabel('speed(degrees/s)')
plt.xlabel('time')
plt.show()

plt.plot(accelerationTimes,acceleration)
plt.ylabel('acceleration(degrees/s/s)')
plt.xlabel('time')
plt.show()

plt.plot(time,leftPower)
plt.ylabel('leftPower')
plt.xlabel('time')
plt.show()

plt.plot(time,rightPower)
plt.ylabel('rightPower')
plt.xlabel('time')
plt.show()

plt.plot(time,leftPower, rightPower)
plt.ylabel('leftPower')
plt.xlabel('time')
plt.show()
