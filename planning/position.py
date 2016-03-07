import math
import logging


class Coordinate(object):

    def __init__(self, x, y):
        if x is None or y is None:
            raise ValueError('Can not initialize to attributes to None')
        else:
            self._x = x
            self._y = y

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @x.setter
    def x(self, new_x):
        if new_x is None:
            raise ValueError('Can not set attributes of Coordinate to None')
        else:
            self._x = new_x

    @y.setter
    def y(self, new_y):
        if new_y is None:
            raise ValueError('Can not set attributes of Coordinate to None')
        else:
            self._y = new_y

    def __repr__(self):
        return 'x: %s, y: %s\n' % (self._x, self._y)


class Vector(Coordinate):

    def __init__(self, x, y, angle, velocity):
        self._angle = 0
        self._velocity = 0
        super(Vector, self).__init__(x, y)
        if angle is None or velocity is None:
            raise ValueError('Can not initialise attributes of Vector to None')
        else:
            self.velocity = velocity
            self.angle = angle

    @property
    def angle(self):
        return self._angle

    @property
    def velocity(self):
        return self._velocity

    @angle.setter
    def angle(self, new_angle):
        if new_angle is None:
            raise ValueError('Can not initialise attributes of Vector to None')
        elif new_angle < 0 or new_angle >= (2 * math.pi):
            new_angle_converted = new_angle
            while (new_angle_converted < 0):
                new_angle_converted += 2 * math.pi

            while (new_angle_converted >= 2 * math.pi):
                new_angle_converted -= 2 * math.pi

            self._angle = new_angle_converted
        else:
            self._angle = new_angle

    @velocity.setter
    def velocity(self, new_velocity):
        if new_velocity is None or new_velocity < 0:
            raise ValueError('Velocity can not be None or negative')
        self._velocity = new_velocity

    def __eq__(self, other):
        return (isinstance(other, self.__class__) and (self.__dict__ == other.__dict__))

    def __repr__(self):
        return ('x: %s, y: %s, angle: %s, velocity: %s\n' %
                (self.x, self.y,
                 self._angle, self._velocity))
