import sys
import pygame
from pygame.locals import *


class World:
    def __init__(self, vision_world):
        self.robots = []
        self.robots.append(Robot(50, 50, pygame.Color(200, 200, 200, 0)))
        self.ball = None


class SimulatorMain:

    def __init__(self, width=340, height=525):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((self.width, self.height))

    def loop(self):
        world = World(None)
        while True:
            # Call planner
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()
            self.draw(world, self.screen)

    def draw(self, world, surface):
        for r in world.robots:
            r.draw(surface)
        # world.ball.draw()
        pygame.display.flip()


class Object (object):

    def __init__(self, left, top, direction, velocity, color):
        self.rect = pygame.Rect(left, top, self.WIDTH, self.HEIGHT)
        self.color = color
        self.direction = direction
        self.velocity = 0

    def draw(self, surface):
        pygame.draw.rect(surface, self.color, self.rect)

    def move(self, x, y):
        self.rect.move_ip(x, y)


class Robot (Object):
    WIDTH = 20
    HEIGHT = 20

    KICKER_DISTANCE = 5
    ROBOT_VELOCITY = 2

    def __init__(self, left, top, color):
        self.kicker_area = pygame.Rect(left, top + self.KICKER_DISTANCE,
                                       self.WIDTH, self.KICKER_DISTANCE)
        self.facing = 0
        self.destination = (0, 0)
        super(Robot, self).__init__(left, top, 0, self.ROBOT_VELOCITY, color)

    def move(self, x, y):
        self.kicker_area.move_ip(x, y)
        super(Robot, self).move(x, y)

    def move_to(self, x, y):
        self.direction = math.degrees(math.atan2(y, x))


class Ball (Object):
    WIDTH = 10
    HEIGHT = 10

if __name__ == "__main__":
    window = SimulatorMain()
    window.loop()
