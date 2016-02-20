import math


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
        print(theta)
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


def defender_get_rotation_to_catch_point(robot_vec, ball_vec, catch_distance):
    '''
    This method returns an angle by which the defending robot needs to rotate to face the catching point.
    It takes either an x, y coordinate of the ball that we want to approach to and computes the catching point.
    positive angle - clockwise rotation
    negative angle - counter-clockwise rotation
    '''
    delta_x = ball_vec.x - robot_vec.x
    delta_y = ball_vec.y - robot_vec.y
    theta = get_rotation_to_point(robot_vec, ball_vec)
    displacement = math.hypot(delta_x, delta_y)
    if displacement == 0:
        alpha = 0
    else:
        alpha = math.sin(catch_distance / displacement)
    if theta > 0:
        return theta - alpha
    else:
        return theta + alpha


def ball_is_static(world):
    # TODO find real threshold value
    static_treshold = 0.1
    return world.ball.velocity < static_treshold


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
