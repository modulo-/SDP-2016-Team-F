from collections import namedtuple
from logging import debug

Positions = namedtuple('Positions', 'robot_blue_pink robot_blue_green ' +
        'robot_yellow_pink robot_yellow_green ball')
Position = namedtuple('Position', 'x y angle')

class Predictor:
    def __init__(self):
        self._position_history = []
        for _ in range(10):
            self._position_history.append(Positions(None, None, None, None, None))

    def update(self, world):
        del self._position_history[0]
        self._position_history.append(Positions(
            world.robot_blue_pink,
            world.robot_blue_green,
            world.robot_yellow_pink,
            world.robot_yellow_green,
            world.ball,
        ))

    def predict(self):
        # TODO
        debug("Predition: " + self._position_history[-1])
        return self._position_history[-1]
