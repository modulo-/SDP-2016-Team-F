import math

def move(distance):
    print("Moving distance {0}".format(distance))

def move_to(x, y):
    heading = math.degrees(math.atan2(y, x))
    distance = math.sqrt(x**2+y**2)
    print("Turning to head {0} and moving distance {1}".format(heading, distance))

def turn(angle):
    print("Turning angle {0}".format(angle))

def kick(distance):
    print("Kicking distance {0}".format(distance))
