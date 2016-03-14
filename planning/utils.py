import math
import logging
import random
import numpy as np
from position import Vector, Coordinate

from math import pi


def get_rotation_to_point(vec1, vec2):
    '''
    This method returns an angle by which vec1 needs to rotate to achieve alignment with vec2.
    It takes either an x, y coordinate of the object that we want to rotate to.
    positive angle - clockwise rotation
    negative angle - counter-clockwise rotation
    '''
    delta_x = vec2.x - vec1.x
    delta_y = vec2.y - vec1.y

    displacement = math.hypot(delta_x, delta_y)
    if displacement == 0:
        theta = 0
    else:
        theta = math.atan2(delta_x, delta_y) - vec1.angle  # atan2(sin(self.angle), cos(self.angle))
        if theta > math.pi:
            theta -= 2 * math.pi
        elif theta < -math.pi:
            theta += 2 * math.pi
    assert -math.pi <= theta <= math.pi

    return theta


def attacker_get_rotation_to_point(robot_vec, ball_vec):
    '''
    This method returns an angle by which the attacking robot needs to rotate to achieve alignment.
    It takes either an x, y coordinate of the object that we want to rotate to.
    positive angle - clockwise rotation
    negative angle - counter-clockwise rotation
    '''
    return get_rotation_to_point(robot_vec, ball_vec)


def is_ball_between_angles(front_angle, kicker_angle, relative_ball_angle, relative_catch_angle):
    '''
    Checks whether the ball is between the kicker angle and the facing angle.
    This method is important to make sure that the robot approaches the ball_angle
    from the side where the kicker is.
    '''
    ball_angle = front_angle + relative_ball_angle
    catch_point_angle_left = ball_angle - relative_catch_angle

    if is_angle_between_angles(kicker_angle, ball_angle, catch_point_angle_left):
        return relative_ball_angle - relative_catch_angle
    else:
        return relative_ball_angle + relative_catch_angle


def is_angle_between_angles(a, b, c):
    '''
    Returns true if angle b is between angles a anc c.
    Takes wrapping into account.
    '''
    if a > c:
        x = c
        c = a
        a = x

    # assuming c > a
    if c - a > math.pi:
        return b > c or b < a
    else:
        return b > a and b < c

def defender_get_alignment_offset(robot, obj, obj_angle, robot_angle_delta=0):
    obj0 = np.array([obj.x, obj.y])
    objv = np.array([math.sin(obj_angle), math.cos(obj_angle)])
    robot0 = np.array([robot.x, robot.y])
    angle = robot.angle + robot_angle_delta + pi/2
    robotv = np.array([math.sin(angle), math.cos(angle)])
    coefficients = np.array([objv, -robotv]).T
    constants = robot0 - obj0
    try:
        distances = np.linalg.solve(coefficients, constants)
    except np.linalg.LinAlgError:
        # Well shit.
        return 0
    if distances[0] < 0:
        # TODO: We are behind the ball/object! Panic!
        # Or rather, switch to a different plan.
        pass
    return distances[1]


def defender_get_rotation_to_catch_point(robot_vec, ball_vec, catch_distance):
    '''
    Finds a relative angle by which the defending robot needs to rotate to face the catching point.
    Catching point is the location where the robot is facing the ball and so can grab
    the ball easily.
    There are usually two grabbing point from each side of the ball, so the 'is_ball_between_angles'
    method is used to find the one that is suitable for taking the ball.
    Also, it tries to approach the ball from the left side and the righ side from the kicker and
    return the rotation that is shorter.
    positive angle - clockwise rotation
    negative angle - counter-clockwise rotation
    '''

    # Two vectors from the left and the right side of movement
    robot_vec_left = Vector(robot_vec.x, robot_vec.y, robot_vec.angle - math.pi / 2, 0)
    robot_vec_right = Vector(robot_vec.x, robot_vec.y, robot_vec.angle + math.pi / 2, 0)

    # calculates catching point angles for each side of movement
    first_front_angle, first_theta, first_alpha = defender_get_rotation_to_catch_point_helper(robot_vec_left, ball_vec, catch_distance, "left")
    second_front_angle, second_theta, second_alpha = defender_get_rotation_to_catch_point_helper(robot_vec_right, ball_vec, catch_distance, "right")

    # gets the best rotation angle for each side if movement
    first_relative_angle = is_ball_between_angles(first_front_angle, robot_vec.angle, first_theta, first_alpha)
    second_relative_angle = is_ball_between_angles(second_front_angle, robot_vec.angle, second_theta, second_alpha)

    # returns the rotation angle that is shorter
    if abs(first_relative_angle) < abs(second_relative_angle):
        logging.debug("Returning first:" + str(first_relative_angle) + " in degrees " + str(math.degrees(first_relative_angle)))
        return (first_relative_angle, "left")
    else:
        logging.debug("Returning second:" + str(second_relative_angle) + " in degrees " + str(math.degrees(second_relative_angle)))
        return (second_relative_angle, "right")


def defender_get_rotation_to_catch_point_helper(robot_vec, ball_vec, catch_distance, side="otherside"):
    '''
    Helping function for 'defender_get_rotation_to_catch_point_helper'.
    Computes the rotation to face the catching point given the side of movement.
    '''
    delta_x = ball_vec.x - robot_vec.x
    delta_y = ball_vec.y - robot_vec.y

    logging.debug("get_rotation_to_point from ({4} {5}) facing {6} to ({0} {1}) deltas ({2} {3})".format(ball_vec.x, ball_vec.y, delta_x, delta_y, robot_vec.x, robot_vec.y, robot_vec.angle))

    theta = get_rotation_to_point(robot_vec, ball_vec)
    logging.debug(side.upper() + " To BALL: " + str(theta) + " in degrees " + str(math.degrees(theta)))
    logging.debug("rotation to the ball = {0} in degrees {1}".format(theta, math.degrees(theta)))

    displacement = math.hypot(delta_x, delta_y)
    if displacement == 0:
        alpha = 0
    else:
        # TODO: has to handle this edge case better
        try:
            alpha = math.asin(catch_distance / displacement)
        except ValueError as er:
            print("Value Error!")
            print(er)
            return (robot_vec.angle, theta, 0)

    logging.debug("alpha angle = {0} in degrees {1}".format(alpha, math.degrees(alpha)))

    return (robot_vec.angle, theta, alpha)


def ball_heading_to_our_goal(world):
    '''
    Returns true if the ball is heading toward our goal (the side, not the
    object itself).
    '''
    if ball_is_static(world):
        return False
    if world.our_side == 'left':
        return world.ball.angle >= math.pi
    else:
        return world.ball.angle < math.pi

def ball_is_static(world):
    '''
    Returns true if the ball has lower velovity then defined threshold.
    '''

    static_threshold = 10
    return world.ball.velocity < static_threshold


def ball_can_reach_robot(ball, robot):
    '''
    Estimates the final location of moving ball and checks if the
    ball can reach a robot
    '''

    # FIXME this has to do stuff.
    # expected_distance = world.ball.velocity
    # dx = expected_distance * math.sin(math.radians(world.ball.angle))
    # dy = expected_distance * math.cos(math.radians(world.ball.angle))

    # expected_position = Coordinate(world.ball.x + dx, world.ball.y + dy)

    return True


def robot_can_reach_ball(ball, robot):
    '''
    Estimates the final location of moving ball and checks if the
    ball
    '''

    # threshold = 30
    return True


def defender_distance_to_line(axis, robot_vec, point):
    '''
    Calculates the moving distance to a point that is either on x
    or y axis.
    '''
    axis = 'y'
    distance = 0

    if axis == 'y':
        # Computer the intersection point
        x2 = 10 * math.sin(math.radians(robot_vec.angle))
        y2 = 10 * math.cos(math.radians(robot_vec.angle))
        print ("POINT: " + str(x2) + " - " + str(y2))
        robot_line = ((robot_vec.x, robot_vec.y), (x2, y2))
        target_line = ((0, point), (10, point))
        intersection_point = line_intersection(robot_line, target_line)

        # Compute the distnce to an intersection point
        vector_x = intersection_point[0] - robot_vec.x
        vector_y = intersection_point[1] - robot_vec.y
        distance = math.hypot(vector_x, vector_y)

        # Get movement direction
        # direction_vec = Vector(vector_x, vector_y, 0, 0)
        direction = get_movement_direction_from_vector(robot_vec, intersection_point)
        print("DISTANCE: " + str(distance * direction))

        return distance * direction

    return 0


def defender_distance_on_y(robot_vec, y_value):
    '''
    Calculates the moving distance on y axis.
    '''
    distance = abs(y_value - robot_vec.y)

    target_point = Vector(robot_vec.x, y_value, 0, 0)
    direction = get_movement_direction_from_vector(robot_vec, target_point)

    return (distance + 2) * direction


def get_movement_direction_from_vector(robot_vec, point_vec):
    '''
    Returns either
        1 - move right; or
        -1 - move left
    '''
    robot_vec_left = Vector(robot_vec.x, robot_vec.y, robot_vec.angle - math.pi / 2, 0)
    # point_vec = Vector(point[0], point[1], 0, 0)

    rotation_for_left = get_rotation_to_point(robot_vec_left, point_vec)
    # rotation_for_right = get_rotation_to_point(robot_vec_right, direction_vec)

    # print (math.degrees(robot_vec.angle))
    # print (point)
    # print ("ROTATION LEFT: " + str(math.degrees(rotation_for_left)))
    if abs(math.degrees(rotation_for_left)) < 90:
        return -1
    else:
        return 1


def line_intersection(line1, line2):
    '''
    Finds the interception point of the lines
    Each line as this structure ((x1, y1), (x2, y2))
    '''
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        logging.error('Lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

# Test if robot can score
# From 2015 Group 12 behaviour/utilities.py
def can_score(world, our_robot, their_goal, turn=0):
    predicted_y = predict_y_intersection(world, our_robot, their_goal.x)

    logging.info("Predicted goal intersection ({0}, {1})".format(their_goal.x, predicted_y))

    # return goal_posts[0][1] < predicted_y < goal_posts[1][1]
    return their_goal.lower_post < predicted_y < their_goal.higher_post

# Test if a robot at 'position' could be passed to
def defender_can_pass_to_position(world, position):
    raise NotImplementedError


# Test if a robot at 'position' could score
def attacker_can_score_from_position(world, position):
    raise NotImplementedError

# Predict y-intersection of robot's direction and goal line at predict_for_x
def predict_y_intersection(world, robot, predict_for_x):
    return robot.y + math.tan(math.pi / 2 - robot.angle) * (predict_for_x - robot.x)


def defender_angle_to_pass_upfield(world, defender_robot, enemy_zone_radius=40):
    """
    Calculates the angle for defender to pass to attacker or, if not possible,
    angle to kick upfield which is least contested by enemy robots.

    :param world: World object
    :param defender_robot: The robot who is trying to kick
    :param enemy_zone_radius: The radius of the circle around enemy robots in which the ball should not pass
    :return: The calculated angle
    """

    def can_pass_to_attacker(defender_vec, attacker_vec, their_robots_vecs):
        can_pass = True
        defender_attacker_line = ((defender_vec.x, defender_vec.y), (attacker_vec.x, attacker_vec.y))
        for t in their_robots_vecs:
            if line_intersects_circle(defender_attacker_line, ((t.x, t.y), enemy_zone_radius)):
                can_pass = False
                break
        return can_pass

    aim_vector = world.our_attacker.vector
    if world.our_attacker.is_missing:
        aim_vector = world.their_goal.vector

    their_vecs = [t.vector for t in world.their_robots if not t.is_missing]

    if can_pass_to_attacker(defender_robot.vector, aim_vector, their_vecs):
        return get_rotation_to_point(defender_robot.vector, aim_vector)
    else:
        # TODO possibly improve this to be less brute force
        y = aim_vector.y
        num_of_iterations = 100
        step = world.pitch.height / num_of_iterations
        # Upper range
        for x in range(world.pitch.height/2, world.pitch.height, step):
            new_vec = Vector(x, y, 0, 0)
            # Could skip by enemy_zone_radius but can't work out a way to update loop variable in python
            if can_pass_to_attacker(defender_robot.vector, new_vec, their_vecs):
                return get_rotation_to_point(defender_robot.vector, new_vec)
        # Lower range
        for x in range(world.pitch.height/2, 0, -step):
            new_vec = Vector(x, y, 0, 0)
            # Could skip by enemy_zone_radius but can't work out a way to update loop variable in python
            if can_pass_to_attacker(defender_robot.vector, new_vec, their_vecs):
                return get_rotation_to_point(defender_robot.vector, new_vec)


        # Could not find an angle, choose a random one within the cone between the defender and their corners
        top_corner = Vector(0, 0, 0, 0)
        bottom_corner = Vector(0, world.pitch.height, 0, 0)
        if world.our_side == 'left':
            top_corner = Vector(world.pitch.width, 0, 0, 0)
            bottom_corner = Vector(world.pitch.width, world.pitch.height, 0, 0)

        top_angle = get_rotation_to_point(defender_robot.vector, top_corner)
        bottom_angle = get_rotation_to_point(defender_robot.vector, bottom_corner)

        return random.uniform(min(top_angle, bottom_angle), max(top_angle, bottom_angle))



def line_intersects_circle(line, circle):
    """
    Does line intersect circle?

    Algorithm adapted from http://www.mathematik.tu-darmstadt.de/~ehartmann/cdgen0104.pdf page 17

    :param line: A line in format ((x1, y1), (x2, y2))
    :param circle: A circle in format ((x1, y1), r)
    :return: True if line intersects circle, false otherwise
    """

    xm = circle[0][0]
    ym = circle[0][1]
    r = circle[1]
    a = line[1][1] - line[0][1]
    b = line[1][0] - line[0][0]
    c = line[0][0]*line[1][1] - line[1][0]*line[0][1]

    # Check variables are in range
    if a == 0 or b == 0 or r < 0:
        return False

    c_prime = c - a * xm - b * ym

    return r**2 * (a**2 + b**2) - c_prime**2 >= 0
