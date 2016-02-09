import cocos
import cocos.actions as ac
from math import pi
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
        self.p = Planner(comms=SimulatorComms(self.our_robot, self.wait_and_next_step))
        self.w = World('left', 0)
        self.w.our_robot._receiving_area = {'width': 10, 'height': 10, 'front_offset': 10}

    def run(self, sequence):
        self.sequence = sequence
        self.sequence_pos = 0

        self.step()

    def step(self):
        print("Step: " + str(self.sequence_pos))
        if self.sequence_pos >= len(self.sequence):
            print "Finished"
            return

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

        start_x = 100
        start_y = 100
        robot = self.add_robot([start_x, start_y])
        ball = self.add_ball([100, 200])
        t = Test(our_robot=robot, ball=ball)

        sequence = [
            {'our_robot': Vector(start_x, start_y, 0, 0), 'ball': Vector(100, 200, 0, 0)},
            {'our_robot': Vector(start_x, start_y, pi / 2, 0), 'ball': Vector(100, 200, 0, 0)}]
        t.run(sequence)

    def add_robot(self, pos):
        robot = Robot(pos=pos)
        self.add(robot)
        return robot

    def add_ball(self, pos):
        ball = Ball(pos=pos)
        self.add(ball)
        return ball


class SimulatorComms(CommsManager):

    def __init__(self, robot, wait_and_next_step):
        self.robot = robot
        self.wait_and_next_step = wait_and_next_step
        super(SimulatorComms, self).__init__(0)

    def move_to(self, x, y):
        super(SimulatorComms, self).move_to(x, y)
        delay = self.robot.move_to(x, y)
        self.wait_and_next_step(delay)

    def turn(self, angle):
        print("Turning robot {0} angle {1}".format(self.robot_index, angle))
        delay = self.robot.rotate_by(angle)
        self.wait_and_next_step(delay)


class Robot(cocos.sprite.Sprite):

    def __init__(self, pos):
        super(Robot, self).__init__('res/robot.png')
        self.velocity = (0, 0)
        self.scale = 1
        self.set_position(pos[0], pos[1])
        self.rotation = 0

        self._movement_speed = 2
        self._rotation_speed = 1

    def set_position(self, x, y):
        self.position = self.get_center_position(x, y)

    def get_center_position(self, x, y):
        return (x - self.width / 2, y - self.height / 2)

    def move_to(self, x, y):
        self.do(ac.MoveTo(self.get_center_position(x, y), duration=self._movement_speed))
        return self._movement_speed

    def rotate_by(self, radians):
        angle = radians * 180 / pi
        self.do(ac.RotateBy(angle, self._rotation_speed))
        return self._rotation_speed


class Ball(cocos.sprite.Sprite):

    def __init__(self, pos):
        super(Ball, self).__init__('res/ball.png')
        self.velocity = (0, 0)
        self.scale = 1
        self.set_position(pos[0], pos[1])
        self.rotation = 0

        self._movement_speed = 15

    def get_center_position(self, x, y):
        return (x - self.width / 2, y - self.height / 2)

    def set_position(self, x, y):
        self.position = self.get_center_position(x, y)

    def move_to(self, x, y):
        self.do(ac.MoveTo(self.get_center_position(x, y), duration=self._movement_speed))
        return self._movement_speed


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
