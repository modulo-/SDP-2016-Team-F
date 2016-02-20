import cocos
import cocos.actions as ac
import math
from threading import Timer

from position import Vector
from world import World
from planner import DefencePlanner
from comms import CommsManager


def rotation_adjustment(angle):
    angle -= math.radians(270)

    if angle < 0:
        return angle + math.pi * 2
    elif angle > math.pi * 2:
        return angle - math.pi * 2
    else:
        return angle


class Test:

    def __init__(self, our_defender, ball):
        self.our_defender = our_defender
        self.ball = ball
        self.sequence = None
        self.sequence_pos = None
        self.p = DefencePlanner(comms=SimulatorComms(self.our_defender, self.ball, self.wait_and_next_step))
        self.w = World('left', 0)
        self.w.our_defender._receiving_area = {'width': 50, 'height': 50, 'front_offset': 10}

    def run(self, sequence):
        self.sequence = sequence
        self.sequence_pos = 0

        self.step()

    def step(self):
        print("\nStep: " + str(self.sequence_pos))
        if self.sequence_pos >= len(self.sequence):
            print "Finished"
            return

        if 'task' in self.sequence[self.sequence_pos].keys():
            self.p.set_task(self.sequence[self.sequence_pos]['task'])
            print ("Changing task to " + self.sequence[self.sequence_pos]['task'])

        # adjusting
        adjusted_sequence = self.sequence[self.sequence_pos]
        adjusted_sequence['our_defender'].angle = rotation_adjustment(adjusted_sequence['our_defender'].angle)

        self.w.update_positions(adjusted_sequence)
        self.p.plan_and_act(self.w)

        self.sequence_pos += 1

    def wait_and_next_step(self, delay):
        print("Waiting for " + str(delay) + " seconds...")
        t = Timer(delay, self.step)
        t.start()


class Scene(cocos.layer.ColorLayer):

    def __init__(self):
        super(Scene, self).__init__(100, 255, 100, 255)

        start_robot_x = 100
        start_robot_y = 100
        start_robot_rotation = math.radians(270)
        start_ball_x = 200
        start_ball_y = 170

        robot = self.add_robot([start_robot_x, start_robot_y], start_robot_rotation)
        ball = self.add_ball([start_ball_x, start_ball_y])
        t = Test(our_defender=robot, ball=ball)

        # 1) For starting rotation "math.radians(270)"
        sequence = [
            {'our_defender': Vector(start_robot_x, start_robot_y, start_robot_rotation, 0), 'ball': Vector(start_ball_x, start_ball_y, 0, 0)},
            {'our_defender': Vector(start_robot_x, start_robot_y, start_robot_rotation + 1.92014072481 - (math.pi * 2), 0), 'ball': Vector(start_ball_x, start_ball_y, 0, 0)},
        ]

        # 2) For starting rotation "math.radians(180)"
        # sequence = [
        #     {'our_defender': Vector(start_robot_x, start_robot_y, start_robot_rotation, 0), 'ball': Vector(start_ball_x, start_ball_y, 0, 0)},
        #     {'our_defender': Vector(start_robot_x, start_robot_y, start_robot_rotation - 1.57079632679, 0), 'ball': Vector(start_ball_x, start_ball_y, 0, 0)},
        # ]

        t.run(sequence)

    def add_robot(self, pos, rotation_radians):
        robot = Robot(pos=pos, rotation_radians=rotation_radians)
        self.add(robot)
        return robot

    def add_ball(self, pos):
        ball = Ball(pos=pos)
        self.add(ball)
        return ball


class SimulatorComms(CommsManager):

    def __init__(self, robot, ball, wait_and_next_step):
        self.robot = robot
        self.ball = ball
        self.wait_and_next_step = wait_and_next_step
        super(SimulatorComms, self).__init__(0)

    def move(self, d):
        super(SimulatorComms, self).move(d)
        print("Robot move distance: {0}".format(d))

        rotation = self.robot.rotation + 90
        dx = d * math.sin(math.radians(rotation))
        dy = d * math.cos(math.radians(rotation))
        # print("Robot dxdy: {0} {1}".format(dx, dy))
        delay = self.robot.move_to(
            self.robot.position[0] + dx,
            self.robot.position[1] + dy)
        self.wait_and_next_step(delay)

    def turn(self, angle):
        '''
        positive angle - counter-clockwise rotation
        negative angle - clockwise rotation
        angle 0 means turned right
        '''
        print("Robot rotation: {0}".format(self.robot.rotation))
        super(SimulatorComms, self).turn(angle)
        delay = self.robot.rotate_by(angle)
        self.wait_and_next_step(delay)

    def close_grabbers(self):
        super(SimulatorComms, self).close_grabbers()
        # TODO Fix assumption that we can grab ball
        self.ball.grab(self.robot)

        self.wait_and_next_step(1)

    def release_grabbers(self):
        super(SimulatorComms, self).release_grabbers()
        self.ball.release()

        self.wait_and_next_step(1)


class Sprite (cocos.sprite.Sprite):

    def __init__(self, sprite, pos):
        super(Sprite, self).__init__(sprite)
        self.velocity = (0, 0)
        self.scale = 1
        self.set_position(pos[0], pos[1])
        self.rotation = 0

    def set_position(self, x, y):
        self.position = (x, y)

    def move_to(self, x, y):
        self.do(ac.MoveTo((x, y), duration=self._movement_speed))
        print("Moving from: {0}".format(self.position))
        return self._movement_speed

    def rotate_by(self, radians):
        '''
        positive angle - counter-clockwise rotation
        negative angle - clockwise rotation
        angle 0 means turned right
        '''
        angle = math.degrees(radians)
        self.do(ac.RotateBy(angle, self._rotation_speed))
        return self._rotation_speed


class Robot(Sprite):

    def __init__(self, pos, rotation_radians):
        super(Robot, self).__init__('res/robot.png', pos)
        self._movement_speed = 2
        self._rotation_speed = 2

        # initial rotation adjustment
        rotation_radians -= math.pi / 2
        if rotation_radians < 0:
            rotation_radians += math.pi * 2
        self.rotation = math.degrees(rotation_radians)


class Ball(Sprite):

    def __init__(self, pos):
        super(Ball, self).__init__('res/ball.png', pos)
        self._movement_speed = 15

    def grab(self, robot):
        pass

    def release(self):
        self._anchor_x = 0
        self._anchor_y = 0


class Environment():
    def __init__(self, width, height):

        # director init takes the same arguments as pyglet.window
        cocos.director.director.init(width=width, height=height, resizable=False)
        world = Scene()

        # A scene that contains the layer hello_layer
        main_scene = cocos.scene.Scene(world)

        # And now, start the application, starting with main_scene
        cocos.director.director.run(main_scene)


if __name__ == "__main__":
    Environment(width=450, height=650)
