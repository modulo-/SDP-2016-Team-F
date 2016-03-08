#!/usr/bin/python2
from planning.comms import RFCommsManager
import os


print "serial devices available"

i=0
b=[]
for d in os.listdir("/dev"):
    if d[0:6]=="ttyACM":
            print str(i) + "\t" + d
            b.append("/dev/"+d)
robot = RFCommsManager(0, b[int(raw_input("serial device:"))])
print("press h for help")

while True:
    command=raw_input("command (t/k/m/g/r):")
    if command == "t":
        angle=int(raw_input("angle:"))
        robot.turn(angle)
        print("robot.turn("+str(angle)+")")
    elif command == "k":
        distance=int(raw_input("distance in cm:"))
        robot.kick(distance)
        print("robot.kick("+str(distance)+")")
    elif command == "m":
        distance=int(raw_input("distance in mm")) 
        robot.move(distance)
        print("robot.move("+str(distance)+")")
    elif command=="g":
        robot.close_grabbers()
        print("robot.grab()")
    elif command=="r":
        robot.release_grabbers()
        print("robot.release()")
    else:
        print("commands: help h, kick k, move m, grab g, release r")
    
