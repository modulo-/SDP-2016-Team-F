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

    # TODO find real threshold value
    static_threshold = 1
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
