from collections import namedtuple
from logging import debug
from Vision.world import Vector

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

    def _predict(self, hist):
        # TODO
        return hist[-1]

    def predict(self):
        ret = []
        for i in range(5):
            ret.append(self._predict([h[i] for h in self._position_history]))
        ret = Positions(*ret)
        debug("Predition: " + str(ret))
        return ret
