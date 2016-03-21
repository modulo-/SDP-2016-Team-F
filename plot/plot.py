#!/usr/bin/python
#will plot the distance, speed and acceleration of robot

import matplotlib.pyplot as plt
distance=[]
difference=[]
time=[]

fileName=raw_input("name of file containing data:")

output=open(fileName,'r')

i=0;
counter=0
for line in output:
    numStr=""
    numLine=False
    for char in line:
        if char.isdigit() or char=='-':
           numStr+=char 
           numLine=True
        elif char==',':
            if(i<2):
                i+=1
            else:
                i=0
            break;
    if numLine:
        print str(counter) + ',' + str(i) + ':' + numStr
        if i==0:
            distance.append((-int(numStr)))
        if i==1:
            difference.append(distance[counter]-(-int(numStr)))
            distance[counter]=(distance[counter]+(-int(numStr)))/2
            if(counter>0 and distance[counter]-distance[counter-1]<=0):
                exit()
        elif i==2:
            time.append((float(numStr)/1000000))
            counter+=1

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
plt.ylabel('distance')
plt.xlabel('time')
plt.show()

plt.plot(time,difference)
plt.ylabel('difference')
plt.xlabel('time')
plt.show()

plt.plot(speedTimes,speed)
plt.ylabel('speed')
plt.xlabel('time')
plt.show()

plt.plot(accelerationTimes,acceleration)
plt.ylabel('acceleration')
plt.xlabel('time')
plt.show()
