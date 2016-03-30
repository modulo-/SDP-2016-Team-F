from __future__ import division
import math
import utils

from Polygon.cPolygon import Polygon
from position import Vector
from logging import debug, info, error

# Width measures the front and back of an object
# Length measures along the sides of an object

ROBOT_WIDTH = 64
ROBOT_LENGTH = 64
# Unintuative, but the height is in cm and not in imaginary vision units.
ROBOT_HEIGHT = 23

BALL_WIDTH = 5
BALL_LENGTH = 5

GOAL_WIDTH = 140
GOAL_LENGTH = 1

GOAL_LOWER = 170
GOAL_HIGHER = 300

MILESTONE_BALL_AWAY_FROM_HOUSEROBOT_THRESHOLD = 30


class PitchObject(object):
    '''
    A class that describes an abstract pitch object
    Width measures the front and back of an object
    Length measures along the sides of an object
    '''

    def __init__(self, x, y, angle, velocity, width, length, height, angle_offset=0):
        if width < 0 or length < 0:
            raise ValueError('Object dimensions must be positive')
        else:
            self._width = width
            self._length = length
            self._height = height
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

    def __init__(self, x, y, angle, velocity, is_our_team, is_house_robot, width=ROBOT_WIDTH, length=ROBOT_LENGTH, height=ROBOT_HEIGHT, angle_offset=0):
        super(Robot, self).__init__(x, y, angle, velocity, width, length, height, angle_offset)
        self._catch_distance = 40
        self._catched_ball = False
        self._catcher = 'OPEN'
        self.penalty = False
        self._has_grabbed = False
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

    @property
    def catched_ball(self):
        return self._catched_ball

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
            return (self.catcher == 'CLOSED') and self.can_catch_ball(ball)
        else:
            return (math.hypot(self.x - ball.x, self.y - ball.y) < MILESTONE_BALL_AWAY_FROM_HOUSEROBOT_THRESHOLD)

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

    def __init__(self, x, y, angle, velocity, is_our_team, is_house_robot, width=ROBOT_WIDTH, length=ROBOT_LENGTH, height=ROBOT_HEIGHT, angle_offset=0):
        self._is_ball_in_grabbers = False
        super(Attacker, self).__init__(x, y, angle, velocity, is_our_team, is_house_robot, width, length, height, angle_offset)

    @property
    def is_ball_in_grabbers(self):
        return self._is_ball_in_grabbers

    @is_ball_in_grabbers.setter
    def is_ball_in_grabbers(self, value):
        self._is_ball_in_grabbers = value

    def has_ball(self, ball):
        return self.is_ball_in_grabbers

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
        # m =  (possession.y - target.y) / (possession.x - target.x)
        # return Vector(self.x, possession.y + m * (self.x - possession.x), 0, 0)
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
        super(Ball, self).__init__(x, y, angle, velocity, BALL_WIDTH, BALL_LENGTH, 0)


class Goal(PitchObject):

    def __init__(self, x, y, angle, lower_post, higher_post):
        self._lower_post = lower_post
        self._higher_post = higher_post
        super(Goal, self).__init__(x, y, angle, 0, GOAL_WIDTH, GOAL_LENGTH, 0)

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
        self._height = 450
        if pitch_num == 0:
            self._goal_box_x = {'left':179, 'right':484}
            self._goal_line_x = {'left':36, 'right':618}
            self._centre_line_x = 276
        else:
            # TODO add values for other pitch room
            assert(False)
            self._goal_box_x = {'left':100, 'right':200}
            self._goal_line_x = {'left':0, 'right':200}
            self._centre_line_x = 400

    def is_within_bounds(self, robot, x, y):
        '''
        Checks whether the position/point planned for the robot is reachable
        '''
        return x > 0 and x < self._width and y > 0 and y < self._height

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

    @property
    def goal_box_x(self):
        return self._goal_box_x

    @property
    def goal_line_x(self):
        return self._goal_line_x

    @property
    def centre_line_x(self):
        return self._centre_line_x

    def __repr__(self):
        return str((self._width, self._height))


class World(object):
    '''
    This class describes the environment
    Creates our robot
    '''
    _ball = Ball(0, 0, 0, 0)
    _our_defender = Defender(0, 0, 0, 0, True, False)
    _our_attacker = Attacker(0, 0, 0, 0, True, False)
    _their_robots = []
    _their_robots.append(Robot(0, 0, 0, 0, False, True))
    _their_robots.append(Robot(0, 0, 0, 0, False, True))
    _our_defender_index = 0
    _our_attacker_index = 1
    _their_defender_index = 2
    _their_attacker_index = 3

    _goals = []

    def __init__(self, our_side, pitch_num):
        '''
        Sets the sides and goals
        '''
        self.our_side = our_side
        self._game_state = None
        self._pitch = Pitch(pitch_num)
        self._their_side = 'left' if our_side == 'right' else 'right'
        self._goals.append(Goal(self._pitch.goal_line_x['left'], self._pitch.height / 2.0, 0, GOAL_LOWER, GOAL_HIGHER))
        self._goals.append(Goal(self._pitch.goal_line_x['right'], self._pitch.height / 2.0, math.pi, GOAL_LOWER, GOAL_HIGHER))
        self._camera_height = 255

    @property
    def game_state(self):
        return self._game_state

    @game_state.setter
    def game_state(self, state):
        assert state in [None, 'kickoff-them', 'kickoff-us', 'normal-play', 'penalty-defend', 'penalty-shoot']
        self._game_state = state

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
    def our_side(self):
        return self._our_side

    @our_side.setter
    def our_side(self, side):
        assert side in ['left', 'right']
        self._our_side = side

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
        halfway = self._pitch.centre_line_x
        if self._our_side == 'left':
            return robot.x < halfway
        else:
            return robot.x >= halfway

    def in_their_half(self, robot):
        return not self.in_our_half(robot)

    def is_possible_position(self, robot, x, y):
        if not self.pitch.is_within_bounds(robot, x, y):
            return False
        if robot == self.our_attacker:
            return x > self.pitch.goal_box_x['left'] and x < self.pitch.goal_box_x['right']
        # TODO
        return True
            

    @property
    def robot_in_possession(self):
        robots = [self.our_defender,
                  self.our_attacker] + self.their_robots
        for r in robots:
            if r.has_ball(self.ball):
                return r
        return None

    def translate_position(self, obj, v):
        deltax = v.x - (self._pitch.width / 2)
        deltay = v.y - (self._pitch.height / 2)
        factor = (self._camera_height - obj.height) / self._camera_height
        deltax *= factor
        deltay *= factor
        return Vector(deltax + self._pitch.width / 2, deltay + self._pitch.height / 2, v.angle, v.velocity)

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
                obj.vector = self.translate_position(obj, kwargs[name])

    def get_new_score_zone(self):
        halfway = self.pitch.centre_line_x
        x = halfway + (halfway - self.pitch.goal_box_x['left']) / 2 if self._our_side == 'left'\
            else halfway - (halfway - self.pitch.goal_box_x['right']) / 2
        y_step = self.pitch.width / 5
        centroids = [Vector(x, y_step * i, 0, 0) for i in range(1, 5)]
        filtered_centroids = filter(lambda v: utils.defender_can_pass_to_position(self, v) and
                                    utils.attacker_can_score_from_position(self, v), centroids)
        sorted_centroids = sorted(filtered_centroids, key=lambda v: (v.x)**2 + (v.y)**2, reverse=True)
        if not sorted_centroids:
            self._score_zone = None
        else:
            self._score_zone = sorted_centroids[0]
        info("Found score zone at {0}".format(self.score_zone))
        return self.score_zone

    @property
    def score_zone(self):
        try:
            if not self._score_zone:
                return self.our_attacker.vector
        except AttributeError:
            pass
        return self._score_zone
