import math

from Polygon.cPolygon import Polygon
from position import Vector
from logging import debug, info, error

# Width measures the front and back of an object
# Length measures along the sides of an object

ROBOT_WIDTH = 64
ROBOT_LENGTH = 64

BALL_WIDTH = 5
BALL_LENGTH = 5

GOAL_WIDTH = 140
GOAL_LENGTH = 1

GOAL_LOWER = 170
GOAL_HIGHER = 300

MILESTONE_BALL_AWAY_FROM_HOUSEROBOT_THRESHOLD = 100

class PitchObject(object):
    '''
    A class that describes an abstract pitch object
    Width measures the front and back of an object
    Length measures along the sides of an object
    '''

    def __init__(self, x, y, angle, velocity, width, length, angle_offset=0):
        if width < 0 or length < 0:
            raise ValueError('Object dimensions must be positive')
        else:
            self._width = width
            self._length = length
            self._angle_offset = angle_offset
            self._vector = Vector(x, y, angle, velocity)
            self._is_missing = True

    @property
    def width(self):
        return self._width

    @property
    def length(self):
        return self._length

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
        if new_vector is None:  # or not isinstance(new_vector, Vector):
            raise ValueError('The new vector can not be None and must be an instance of a Vector')
        else:
            self._vector = Vector(
                new_vector.x, new_vector.y, new_vector.angle - self._angle_offset, new_vector.velocity)
            self._is_missing = False

    def is_missing(self):
        return self._is_missing

    def set_missing(self):
        self._is_missing = True

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
                 self.angle, self.velocity, (self.width, self.length)))


class Robot(PitchObject):

    def __init__(self, x, y, angle, velocity, is_our_team, is_house_robot, width=ROBOT_WIDTH, length=ROBOT_LENGTH, angle_offset=0):
        super(Robot, self).__init__(x, y, angle, velocity, width, length, angle_offset)
        self._catch_distance = 32
        self._catcher = 'OPEN'
        self._is_our_team = is_our_team
        self._is_house_robot = is_house_robot

    @property
    def catcher_area(self):
        front_left = (self.x + self._receiving_area['front_offset'] + self._receiving_area['height'], self.y + self._receiving_area['width'] / 2.0)
        front_right = (self.x + self._receiving_area['front_offset'] + self._receiving_area['height'], self.y - self._receiving_area['width'] / 2.0)
        back_left = (self.x + self._receiving_area['front_offset'], self.y + self._receiving_area['width'] / 2.0)
        back_right = (self.x + self._receiving_area['front_offset'], self.y - self._receiving_area['width'] / 2.0)
        area = Polygon((front_left, front_right, back_left, back_right))
        area.rotate(math.pi / 2 - self.angle, self.x, self.y)
        return area

    @catcher_area.setter
    def catcher_area(self, area_dict):
        self._receiving_area = area_dict

    @property
    def catcher(self):
        return self._catcher

    @property
    def catch_distance(self):
        return self._catch_distance

    @catcher.setter
    def catcher(self, new_position):
        assert new_position in ['OPEN', 'CLOSED']
        self._catcher = new_position

    def can_catch_ball(self, ball):
        '''
        Get if the ball is in the catcher zone but may not have possession
        '''
        if self.catcher_area.isInside(ball.x, ball.y):
            debug("Can catch ball")
        else:
            debug("Can't catch ball")
        return self.catcher_area.isInside(ball.x, ball.y)

    def has_ball(self, ball):
        '''
        Gets if the robot has possession of the ball
        '''
        # TODO Make this work for opponents properly
        if self.is_our_team and not self.is_house_robot:
            return (self._catcher == 'CLOSED') and self.can_catch_ball(ball)
        else:
            return (math.hypot(self.x - ball.x, self.y - ball.y)
                    < MILESTONE_BALL_AWAY_FROM_HOUSEROBOT_THRESHOLD)

    def get_displacement_to_point(self, x, y):
        '''
        This method returns the displacement between the robot and the (x, y) coordinate.
        '''
        delta_x = x - self.x
        delta_y = y - self.y
        displacement = math.hypot(delta_x, delta_y)
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

    def on_goal_arc(self, goal):
        raise NotImplementedError

    def aligned_on_goal_arc(self, attacker):
        raise NotImplementedError

    def __repr__(self):
        return ('x: %s\ny: %s\nangle: %s\nvelocity: %s\ndimensions: %s\n' %
                (self.x, self.y,
                 self.angle, self.velocity, (self.width, self.length)))

    @property
    def is_our_team(self):
        return self._is_our_team

    @property
    def is_house_robot(self):
        return self._is_house_robot

class Defender(Robot):
    @property
    def get_tactical_position(self, world):
        # Calculate tactical position
        raise NotImplementedError


class Attacker(Robot):
    def get_blocking_position(self, world):
        # Calculate blocking position
        possession = world.robot_in_possession
        if not possession:
            error("Attempting to find blocking position while no robot in possession")
            return Vector(0, 0, 0, 0)
        assert(not possession.is_our_team)
        target = None
        if world.in_our_half(possession):
            info("Blocking our goal which is at ({0}, {1})".format(world.our_goal.x, world.our_goal.y))
            target = world.our_goal
        else:
            # Use their other robot as target
            info("Blocking opponent pass")
            target = ([r for r in world.their_robots if r != possession])[0]
        if (possession.x - target.x) == 0:
            error("Robot in possession in line on x!")
            return Vector(0, 0, 0, 0)
        #m =  (possession.y - target.y) / (possession.x - target.x)
        #return Vector(self.x, possession.y + m * (self.x - possession.x), 0, 0)
        return Vector((target.x + possession.x) / 2, (target.y + possession.y) / 2, 0, 0)
        """dx = target.x - possession.x
        dy = target.y - possession.y
        l = math.hypot(dx, dy)
        print dx,dy, l
        dx /= l
        dx *= 75
        dy /= l
        dy *= 75

        l = math.hypot(dx, dy)
        print dx,dy, l
	print possession.x+dx, possession.y+dy

        return Vector(possession.x+dx, possession.y+dy,0,0)"""
        


class Ball(PitchObject):

    def __init__(self, x, y, angle, velocity):
        super(Ball, self).__init__(x, y, angle, velocity, BALL_WIDTH, BALL_LENGTH)


class Goal(PitchObject):

    def __init__(self, x, y, angle, lower_post, higher_post):
        self._lower_post = lower_post
        self._higher_post = higher_post
        super(Goal, self).__init__(x, y, angle, 0, GOAL_WIDTH, GOAL_LENGTH)

    @property
    def lower_post(self):
        return self._lower_post

    @property
    def higher_post(self):
        return self._higher_post

    def __repr__(self):
        return ('x: %s\ny: %s\nangle: %s\nvelocity: %s\ndimensions: %s\n' %
                (self.x, self.y, self.angle, self.velocity, (self.width, self.length)))


class Pitch(object):
    '''
    Class that describes the pitch
    '''

    def __init__(self, pitch_num):
        # TODO Get real pitch size
        self._width = 600
        self._height = 400

    def is_within_bounds(self, robot, x, y):
        '''
        Checks whether the position/point planned for the robot is reachable
        '''
        # TODO Add goal boxes
        return x > 0 and x < self._width and y > 0 and y < self._height

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

    def __repr__(self):
        return str((self._width, self._height))


class World(object):
    '''
    This class describes the environment
    Creates our robot
    '''
    _ball = Ball(0, 0, 0, 0)
    _our_defender = Defender(0, 0, 0, 0, 0, True, True)
    _our_attacker = Attacker(0, 0, 0, 0, 0, True, False)
    _their_robots = []
    _their_robots.append(Robot(0, 0, 0, 0, 0, False, True))
    _their_robots.append(Robot(0, 0, 0, 0, 0, False, True))
    _our_defender_index = 0
    _our_attacker_index = 1
    _their_defender_index = 2
    _their_attacker_index = 3

    _goals = []

    def __init__(self, our_side, pitch_num):
        '''
        Sets the sides and goals
        '''
        assert our_side in ['left', 'right']
        self._pitch = Pitch(pitch_num)
        self._our_side = our_side
        self._their_side = 'left' if our_side == 'right' else 'right'
        self._goals.append(Goal(0, self._pitch.height / 2.0, 0, GOAL_LOWER, GOAL_HIGHER))
        self._goals.append(Goal(self._pitch.width, self._pitch.height / 2.0, math.pi, GOAL_LOWER, GOAL_HIGHER))

    @property
    def our_defender(self):
        return self._our_defender

    @property
    def our_attacker(self):
        return self._our_attacker

    @property
    def their_robots(self):
        return self._their_robots

    @property
    def their_defenders(self):
        return [r for r in self._their_robots if self.in_their_half(r)]

    @property
    def their_attackers(self):
        return [r for r in self._their_robots if self.in_our_half(r)]

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

    def in_our_half(self, robot):
        halfway = self._pitch.width / 2
        if self._our_side == 'left':
            return robot.x < halfway
        else:
            return robot.x >= halfway

    def in_their_half(self, robot):
        return not self.in_our_half(robot)

    @property
    def robot_in_possession(self):
        robots = [self.our_defender,
                  self.our_attacker] + self.their_robots
        for r in robots:
            if r.has_ball(self.ball):
                return r
        return None

    def update_positions(self, **kwargs):
        '''
        This method will update the positions of the pitch objects
        that it gets passed by the vision system
        '''
        pitch_objects = [('our_defender', self.our_defender),
                         ('our_attacker', self.our_attacker),
                         ('their_robot_0', self.their_robots[0]),
                         ('their_robot_1', self.their_robots[1]),
                         ('ball', self.ball)]
        for (name, obj) in pitch_objects:
            if name not in kwargs or kwargs[name] is None:
                obj.set_missing()
            else:
                obj.vector = kwargs[name]

    @property
    def score_zone(self):
        # TODO Temporarily use location for milestone 3 passing
        return self.our_attacker.vector
        if not self._score_zone:
            # Calculate score zone
            raise NotImplementedError
        return self._score_zone
