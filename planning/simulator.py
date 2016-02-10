import cocos
import cocos.actions as ac
from math import pi, sin, cos, radians, degrees
from threading import Timer

from position import Vector
from world import World
from planner import Planner
from comms import CommsManager


class Test:

    def __init__(self, our_robot, ball):
        self.our_robot = our_robot
        self.ball = ball
        self.sequence = None
        self.sequence_pos = None
        self.p = Planner(comms=SimulatorComms(self.our_robot, self.ball, self.wait_and_next_step))
        self.w = World('left', 0)
        self.w.our_robot._receiving_area = {'width': 50, 'height': 50, 'front_offset': 10}

    def run(self, sequence):
        self.sequence = sequence
        self.sequence_pos = 0

        self.step()

    def step(self):
        print("Step: " + str(self.sequence_pos))
        if self.sequence_pos >= len(self.sequence):
            print "Finished"
            return

        if 'task' in self.sequence[self.sequence_pos].keys():
            self.p.set_task(self.sequence[self.sequence_pos]['task'])
            print ("Changing task to " + self.sequence[self.sequence_pos]['task'])
        self.w.update_positions(self.sequence[self.sequence_pos])
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
        start_robot_rotation = 4.73  # 3.14159
        start_ball_x = 200
        start_ball_y = 250

        robot = self.add_robot([start_robot_x, start_robot_y], start_robot_rotation)
        ball = self.add_ball([start_ball_x, start_ball_y])
        t = Test(our_robot=robot, ball=ball)

        sequence = [
            {'our_robot': Vector(start_robot_x, start_robot_y, start_robot_rotation, 0), 'ball': Vector(start_ball_x, start_ball_y, 0, 0)},
            {'our_robot': Vector(start_robot_x, start_robot_y, 0.982793723247, 0), 'ball': Vector(start_ball_x, start_ball_y, 0, 0)},
            {'our_robot': Vector(174.3956213067677, 206.90641670977678, 0.982793723247, 0), 'ball': Vector(start_ball_x, start_ball_y, 0, 0)},
            {'task': 'turn-shoot', 'our_robot': Vector(174.3956213067677, 206.90641670977678, 0.982793723247, 0), 'ball': Vector(start_ball_x, start_ball_y, 0, 0)},
            {'our_robot': Vector(174.3956213067677, 206.90641670977678, 0.0, 0), 'ball': Vector(start_ball_x, start_ball_y, 0, 0)}
        ]
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
        rotation = self.robot.rotation + 90
        dx = d * sin(radians(rotation))
        dy = d * cos(radians(rotation))
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
        super(SimulatorComms, self).turn(-angle)
        delay = self.robot.rotate_by(-angle)
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
        angle = -(radians * 180 / pi)
        self.do(ac.RotateBy(angle, self._rotation_speed))
        return self._rotation_speed


class Robot(Sprite):

    def __init__(self, pos, rotation_radians):
        super(Robot, self).__init__('res/robot.png', pos)
        self._movement_speed = 2
        self._rotation_speed = 1
        self.rotation = -degrees(rotation_radians)


class Ball(Sprite):

    def __init__(self, pos):
        super(Ball, self).__init__('res/ball.png', pos)
        self._movement_speed = 15

    def grab(self, robot):
        self._anchor_x = robot.position[0]
        self._anchor_y = robot.position[1]

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
    Environment(width=400, height=600)
