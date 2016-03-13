import math
import logging
from position import Vector, Coordinate


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
    """# Offset the robot angle if need be
    robot_angle = our_robot.angle + turn
    goal_zone_poly = world.pitch.zones[their_goal.zone][0]

    reverse = True if their_goal.zone == 3 else False
    goal_posts = sorted(goal_zone_poly, key=lambda x: x[0], reverse=reverse)[:2]
    # Makes goal be sorted from smaller to bigger
    goal_posts = sorted(goal_posts, key=lambda x: x[1])

    goal_x = goal_posts[0][0]"""

    goal_x = their_goal.x

    predicted_y = predict_y_intersection(world, goal_x, our_robot, full_width=True)

    logging.info("Predicted goal intersection ({0}, {1})".format(goal_x, predicted_y))

    # return goal_posts[0][1] < predicted_y < goal_posts[1][1]
    return their_goal.lower_post < predicted_y < their_goal.higher_post

# Test if a robot at 'position' could be passed to
def defender_can_pass_to_position(world, position):
    # get defender position
    our_defender = world.our_defender

    # get opposing teams positions
    their_robots[0] = world.their_robots[0]
    their_robots[1] = world.their_robots[1]

    # work out if any opposing team robots are near the line
    # need to work out a good value for this
    threshold = 30

    # distance from their first robot to the line
    y2 = our_defender.y
    y1 = our_position.y
    y0 = their_robots[0].y
    x2 = our_defender.x
    x1 = our_position.x
    x0 = their_robots[0].x
    dist1 = math.abs(((y2 - y1)*x0) - ((x2 - x1)*y0) + x2*y1 - y2*x1) / math.sqrt(math.pow((y2 - y1),2) + math.pow((x2 - x1),2))

    # distance from their second robot to the line
    y3 = their_robots[1].y
    x3 = their_robots[1].x
    dist2 = math.abs(((y2 - y1)*x3) - ((x2 - x1)*y3) + x2*y1 - y2*x1) / math.sqrt(math.pow((y2 - y1),2) + math.pow((x2 - x1),2))

    # if one or both of the opposing robots are too close to the line then we cannot pass to the postion
    if dist1 or dist2 < threshold:
        return False
    # otherwise we can pass to the postion
    else:
        return True


# Test if a robot at 'position' could score
def attacker_can_score_from_position(world, position):
    our_position = position
    our_goal = world.our_goal
    their_attacker = world.their_attackers

    # need to work this out
    can_we_score = False
    # work out area between position and top two points of the goal
    # if their attacker is not in this area we can score from this position

    if (can_we_score == True):
        return True
    else:
        return False

def detect_object(world, vector, distance):
    our_attacker = world.our_attacker
    our_defender = world.our_defender
    their_robots = world.their_robots

    start_x = vector.x
    start_y = vector.y
    angle = vector.angle

    # calculate line
    end_x = start_x + (distance * math.cos(angle))
    end_y = start_y + (distance * math.sin(angle))

    # calculate distance from each object to the line
    distances = []
    # distance from our attacker to line
    dist_attacker = math.abs(((end_y - start_y)*our_attacker.x) - ((end_x - start_x)*our_attacker.y) + end_x*start_y - end_y*start_x) / math.sqrt(math.pow((end_y - start_y),2) + math.pow((end_x - start_x),2))
    distances.append(dist_attacker)

    # distance from our defender to the line
    dist_defender = math.abs(((end_y - start_y)*our_defender.x) - ((end_x - start_x)*our_defender.y) + end_x*start_y - end_y*start_x) / math.sqrt(math.pow((end_y - start_y),2) + math.pow((end_x - start_x),2))
    distances.append(dist_defender)

    # distance from opposing teams first robot to the line
    dist_theirs_0 = math.abs(((end_y - start_y)*their_robots[0].x) - ((end_x - start_x)*their_robots[0].y) + end_x*start_y - end_y*start_x) / math.sqrt(math.pow((end_y - start_y),2) + math.pow((end_x - start_x),2))
    distances.append(dist_theirs_0)

    # distance from opposing teams second robot to the line
    dist_theirs_1 = math.abs(((end_y - start_y)*their_robots[1].x) - ((end_x - start_x)*their_robots[1].y) + end_x*start_y - end_y*start_x) / math.sqrt(math.pow((end_y - start_y),2) + math.pow((end_x - start_x),2))
    distances.append(dist_theirs_1)

    # if within threshold then return object
    threshold = 50

    closest = min(distance for distance in distances if distance < threshold)

    if (closest == dist_attacker):
        return our_attacker
    elif (closest == dist_defender):
        return our_defender
    elif (closest == dist_theirs_0):
        return their_robots[0]
    elif (closest == dist_theirs_1):
        return their_robots[1]
    # else return none
    else:
        return None;

# From 2015 Group 12 behaviour/utilities.py
def predict_y_intersection(world, predict_for_x, robot, full_width=False, bounce=False):
        '''
        Predicts the (x, y) coordinates of the ball shot by the robot
        Corrects them if it's out of the bottom_y - top_y range.
        If bounce is set to True, predicts for a bounced shot
        Returns None if the robot is facing the wrong direction.
        '''
        x = robot.x
        y = robot.y
        top_y = world._pitch.height - 60 if full_width else world.our_goal.y + (world.our_goal.width / 2) - 30
        bottom_y = 60 if full_width else world.our_goal.y - (world.our_goal.width / 2) + 30
        angle = robot.angle
        if (robot.x < predict_for_x and not (math.pi / 2 < angle < 3 * math.pi / 2)) or (robot.x > predict_for_x and (3 * math.pi / 2 > angle > math.pi / 2)):
            if bounce:
                if not (0 <= (y + math.tan(angle) * (predict_for_x - x)) <= world._pitch.height):
                    bounce_pos = 'top' if (y + math.tan(angle) * (predict_for_x - x)) > world._pitch.height else 'bottom'
                    x += (world._pitch.height - y) / math.tan(angle) if bounce_pos == 'top' else (0 - y) / math.tan(angle)
                    y = world._pitch.height if bounce_pos == 'top' else 0
                    angle = (-angle) % (2 * math.pi)
            predicted_y = (y + math.tan(angle) * (predict_for_x - x))

            # Correcting the y coordinate to the closest y coordinate on the goal line:
            if predicted_y > top_y:
                return top_y
            elif predicted_y < bottom_y:
                return bottom_y
            return predicted_y
        else:
            return None
