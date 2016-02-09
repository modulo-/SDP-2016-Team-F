from Polygon.cPolygon import Polygon
from math import hypot, pi, atan2
# from vision import tools
from position import Vector

# Width measures the front and back of an object
# Length measures along the sides of an object

ROBOT_WIDTH = 64
ROBOT_LENGTH = 64
ROBOT_HEIGHT = 10

BALL_WIDTH = 5
BALL_LENGTH = 5
BALL_HEIGHT = 5

GOAL_WIDTH = 140
GOAL_LENGTH = 1
GOAL_HEIGHT = 10


class PitchObject(object):
    '''
    A class that describes an abstract pitch object
    Width measures the front and back of an object
    Length measures along the sides of an object
    '''

    def __init__(self, x, y, angle, velocity, width, length, height, angle_offset=0):
        if width < 0 or length < 0 or height < 0:
            raise ValueError('Object dimensions must be positive')
        else:
            self._width = width
            self._length = length
            self._height = height
            self._angle_offset = angle_offset
            self._vector = Vector(x, y, angle, velocity)

    @property
    def width(self):
        return self._width

    @property
    def length(self):
        return self._length

    @property
    def height(self):
        return self._height

    @property
    def angle_offset(self):
        return self._angle_offset

    @property
    def angle(self):
        return self._vector.angle

    @property
    def velocity(self):
        return self._vector.velocity

    @property
    def x(self):
        return self._vector.x

    @property
    def y(self):
        return self._vector.y

    @property
    def vector(self):
        return self._vector

    @vector.setter
    def vector(self, new_vector):
        if new_vector is None or not isinstance(new_vector, Vector):
            raise ValueError('The new vector can not be None and must be an instance of a Vector')
        else:
            self._vector = Vector(
                new_vector.x, new_vector.y, new_vector.angle - self._angle_offset, new_vector.velocity)

    def get_generic_polygon(self, width, length):
        '''
        Get polygon drawn around the current object, but with some
        custom width and length:
        '''
        front_left = (self.x + length / 2, self.y + width / 2)
        front_right = (self.x + length / 2, self.y - width / 2)
        back_left = (self.x - length / 2, self.y + width / 2)
        back_right = (self.x - length / 2, self.y - width / 2)
        poly = Polygon((front_left, front_right, back_left, back_right))
        poly.rotate(self.angle, self.x, self.y)
        return poly[0]

    def get_polygon(self):
        '''
        Returns 4 edges of a rectangle bounding the current object in the
        following order: front left, front right, bottom left and bottom right.
        '''
        return self.get_generic_polygon(self.width, self.length)

    def __repr__(self):
        return ('x: %s\ny: %s\nangle: %s\nvelocity: %s\ndimensions: %s\n' %
                (self.x, self.y,
                 self.angle, self.velocity, (self.width, self.length, self.height)))


class Robot(PitchObject):

    def __init__(self, zone, x, y, angle, velocity, width=ROBOT_WIDTH, length=ROBOT_LENGTH, height=ROBOT_HEIGHT, angle_offset=0):
        super(Robot, self).__init__(x, y, angle, velocity, width, length, height, angle_offset)
        self._zone = zone
        self._catcher = 'open'

    @property
    def zone(self):
        return self._zone

    @property
    def catcher_area(self):
        front_left = (self.x + self._receiving_area['front_offset'] + self._receiving_area['height'], self.y + self._receiving_area['width'] / 2.0)
        front_right = (self.x + self._receiving_area['front_offset'] + self._receiving_area['height'], self.y - self._receiving_area['width'] / 2.0)
        back_left = (self.x + self._receiving_area['front_offset'], self.y + self._receiving_area['width'] / 2.0)
        back_right = (self.x + self._receiving_area['front_offset'], self.y - self._receiving_area['width'] / 2.0)
        area = Polygon((front_left, front_right, back_left, back_right))
        area.rotate(self.angle, self.x, self.y)
        return area

    @catcher_area.setter
    def catcher_area(self, area_dict):
        self._receiving_area = area_dict

    @property
    def catcher(self):
        return self._catcher

    @catcher.setter
    def catcher(self, new_position):
        assert new_position in ['OPEN', 'CLOSED']
        self._catcher = new_position

    def can_catch_ball(self, ball):
        '''
        Get if the ball is in the catcher zone but may not have possession
        '''
        return self.catcher_area.isInside(ball.x, ball.y)

    def has_ball(self, ball):
        '''
        Gets if the robot has possession of the ball
        '''
        return (self._catcher == 'closed') and self.can_catch_ball(ball)

    def get_rotation_to_point(self, x, y):
        '''
        This method returns an angle by which the robot needs to rotate to achieve alignment.
        It takes either an x, y coordinate of the object that we want to rotate to
        '''
        delta_x = x - self.x
        delta_y = y - self.y
        displacement = hypot(delta_x, delta_y)
        if displacement == 0:
            theta = 0
        else:
            theta = atan2(delta_y, delta_x) - self.angle  # atan2(sin(self.angle), cos(self.angle))
            if theta > pi:
                theta -= 2 * pi
            elif theta < -pi:
                theta += 2 * pi
        assert -pi <= theta <= pi
        return theta

    def get_displacement_to_point(self, x, y):
        '''
        This method returns the displacement between the robot and the (x, y) coordinate.
        '''
        delta_x = x - self.x
        delta_y = y - self.y
        displacement = hypot(delta_x, delta_y)
        return displacement

    def get_direction_to_point(self, x, y):
        '''
        This method returns the displacement and angle to coordinate x, y.
        '''
        return self.get_displacement_to_point(x, y), self.get_rotation_to_point(x, y)

    def get_pass_path(self, target):
        '''
        Gets a path represented by a Polygon for the area for passing ball between two robots
        '''
        robot_poly = self.get_polygon()
        target_poly = target.get_polygon()
        return Polygon(robot_poly[0], robot_poly[1], target_poly[0], target_poly[1])

    def __repr__(self):
        return ('zone: %s\nx: %s\ny: %s\nangle: %s\nvelocity: %s\ndimensions: %s\n' %
                (self._zone, self.x, self.y,
                 self.angle, self.velocity, (self.width, self.length, self.height)))


class Ball(PitchObject):

    def __init__(self, x, y, angle, velocity):
        super(Ball, self).__init__(x, y, angle, velocity, BALL_WIDTH, BALL_LENGTH, BALL_HEIGHT)


class Goal(PitchObject):

    def __init__(self, zone, x, y, angle):
        super(Goal, self).__init__(x, y, angle, 0, GOAL_WIDTH, GOAL_LENGTH, GOAL_HEIGHT)
        self._zone = zone

    @property
    def zone(self):
        return self._zone

    def __repr__(self):
        return ('zone: %s\nx: %s\ny: %s\nangle: %s\nvelocity: %s\ndimensions: %s\n' %
                (self._zone, self.x, self.y, self.angle, self.velocity, (self.width, self.length, self.height)))


class Pitch(object):
    '''
    Class that describes the pitch
    '''

    def __init__(self, pitch_num):
        self._width = 100
        self._height = 100

    def is_within_bounds(self, robot, x, y):
        '''
        Checks whether the position/point planned for the robot is reachable
        '''
        zone = self._zones[robot.zone]
        return zone.isInside(x, y)

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

    @property
    def zones(self):
        return self._zones

    def __repr__(self):
        return str(self._zones)


class World(object):
    '''
    This class describes the environment
    Creates our robot
    '''
    _ball = Ball(0, 0, 0, 0)
    _robots = []
    _robots.append(Robot(0, 0, 0, 0, 0))
    _our_robot_index = 0
    _goals = []

    def __init__(self, our_side, pitch_num):
        '''
        Sets the sides and goals
        '''
        assert our_side in ['left', 'right']
        self._pitch = Pitch(pitch_num)
        self._our_side = our_side
        self._their_side = 'left' if our_side == 'right' else 'right'
        self._goals.append(Goal(0, 0, self._pitch.height / 2.0, 0))
        self._goals.append(Goal(3, self._pitch.width, self._pitch.height / 2.0, pi))

    @property
    def our_robot(self):
        return self._robots[self._our_robot_index]

    @property
    def ball(self):
        return self._ball

    @property
    def our_goal(self):
        return self._goals[0] if self._our_side == 'left' else self._goals[1]

    @property
    def their_goal(self):
        return self._goals[1] if self._our_side == 'left' else self._goals[0]

    @property
    def pitch(self):
        return self._pitch

    def update_positions(self, pos_dict):
        '''
        This method will update the positions of the pitch objects
        that it gets passed by the vision system
        '''
        self.our_robot.vector = pos_dict['our_robot']
        self.ball.vector = pos_dict['ball']
