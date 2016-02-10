from world import Robot
from math import pi, tan


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

    #return goal_posts[0][1] < predicted_y < goal_posts[1][1]
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
        if (robot.x < predict_for_x and not (pi / 2 < angle < 3 * pi / 2)) or (robot.x > predict_for_x and (3 * pi / 2 > angle > pi / 2)):
            if bounce:
                if not (0 <= (y + tan(angle) * (predict_for_x - x)) <= world._pitch.height):
                    bounce_pos = 'top' if (y + tan(angle) * (predict_for_x - x)) > world._pitch.height else 'bottom'
                    x += (world._pitch.height - y) / tan(angle) if bounce_pos == 'top' else (0 - y) / tan(angle)
                    y = world._pitch.height if bounce_pos == 'top' else 0
                    angle = (-angle) % (2 * pi)
            predicted_y = (y + tan(angle) * (predict_for_x - x))

            # Correcting the y coordinate to the closest y coordinate on the goal line:
            if predicted_y > top_y:
                return top_y
            elif predicted_y < bottom_y:
                return bottom_y
            return predicted_y
        else:
            return None
