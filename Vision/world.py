import time
from numpy import pi


class Coordinate(object):

    def __init__(self, x, y):
        if x == None or y == None:
            raise ValueError('Can not initialize to attributes to None')
        else:
            self._x = x
            self._y = y

    @property
    def centre(self):
        return (int(self._x), int(self._y))

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @x.setter
    def x(self, new_x):
        if new_x == None:
            raise ValueError('Can not set attributes of Coordinate to None')
        else:
            self._x = new_x

    @y.setter
    def y(self, new_y):
        if new_y == None:
            raise ValueError('Can not set attributes of Coordinate to None')
        else:
            self._y = new_y

    def __repr__(self):
        return 'x: %s, y: %s\n' % (self._x, self._y)

class Vector(Coordinate):

    def __init__(self, x, y, angle, velocity):
        super(Vector, self).__init__(x, y)
        if angle == None or velocity == None:
            raise ValueError('Can not initialise attributes of Vector to None')
        elif angle < 0 or angle >= (2*pi):
            raise ValueError('Angle out of bounds')
        else:
            self._angle = angle
            self._velocity = velocity

    @property
    def angle(self):
        return self._angle

    @property
    def velocity(self):
        return self._velocity

    @angle.setter
    def angle(self, new_angle):
        if new_angle == None or new_angle < 0 or new_angle >= (2*pi):
            raise ValueError('Angle can not be None, also must be between 0 and 2pi')
        self._angle = new_angle

    @velocity.setter
    def velocity(self, new_velocity):
        if new_velocity == None or new_velocity < 0:
            raise ValueError('Velocity can not be None or negative')
        self._velocity = new_velocity

    def __eq__(self, other):
        return (isinstance(other, self.__class__) and (self.__dict__ == other.__dict__))

    def __repr__(self):
        return ('x: %s, y: %s, angle: %s, velocity: %s\n' %
                (self.x, self.y,
                 self._angle, self._velocity))


class World(object):
    def __init__(self, world = None):
        self.time = time.time()
        if world is None:
            self.ball = None
            self.robot_blue_pink = None
            self.robot_blue_green = None
            self.robot_yellow_pink = None
            self.robot_yellow_green = None
        else:
            self.ball = world.ball
            self.robot_blue_pink = world.robot_blue_pink
            self.robot_blue_green = world.robot_blue_green
            self.robot_yellow_pink = world.robot_yellow_pink
            self.robot_yellow_green = world.robot_yellow_green
