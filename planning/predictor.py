from collections import namedtuple
from logging import debug
from Vision.world import Vector
from time import time

Positions = namedtuple('Positions', 'robot_blue_pink robot_blue_green ' +
        'robot_yellow_pink robot_yellow_green ball')
Position = namedtuple('Position', 'x y angle velocity')

def min_with_index(l):
    min = None
    min_index = None
    for (i, item) in enumerate(l):
        if min == None or item < min:
            min = item
            min_index = i
    return (min_index, min)

class Predictor:
    # A threshold for the deltas (square of the distance) per second squared.
    # Given in pixels (1px ~ 0.5cm). Set to 3m/s
    _DELTA_THRESHOLD = 360000
    _HIST_LEN = 50
    _VISION_DELAY = 0.2

    def __init__(self):
        self._robot_blue_pink_history = []
        self._robot_blue_green_history = []
        self._robot_yellow_pink_history = []
        self._robot_yellow_green_history = []
        self._ball_history = []
        self._history_times = []
        for _ in range(Predictor._HIST_LEN):
            self._robot_blue_pink_history.append(None)
            self._robot_blue_green_history.append(None)
            self._robot_yellow_pink_history.append(None)
            self._robot_yellow_green_history.append(None)
            self._ball_history.append(None)
            self._history_times.append(None)

    def update(self, world):
        del self._robot_blue_pink_history[0]
        del self._robot_blue_green_history[0]
        del self._robot_yellow_pink_history[0]
        del self._robot_yellow_green_history[0]
        del self._ball_history[0]
        del self._history_times[0]
        self._robot_blue_pink_history.append(world.robot_blue_pink)
        self._robot_blue_green_history.append(world.robot_blue_green)
        self._robot_yellow_pink_history.append(world.robot_yellow_pink)
        self._robot_yellow_green_history.append(world.robot_yellow_green)
        self._ball_history.append(world.ball)
        self._history_times.append(world.time)

    def _derive_future(self, c):
        if len(c) == 1:
            return c[-1][0]
        # Weighted by the index. TODO: Maybe use a better weighting.
        deltas = (
            sum((c[i][0].x - c[i-1][0].x)*(i**2) for i in range(1, len(c))),
            sum((c[i][0].y - c[i-1][0].y)*(i**2) for i in range(1, len(c))),
            sum((c[i][1] - c[i-1][1]*(i**2)) for i in range(1, len(c))),
        )
        weighting = sum(i**2 for i in range(1, len(c)))
        vec = (deltas[0] / (deltas[2] * weighting), deltas[1] / (deltas[2] * weighting))
        t = time()
        tdelta = t + Predictor._VISION_DELAY - c[-1][1]
        return Position(c[-1][0].x + vec[0]*tdelta, c[-1][0].y + vec[1]*tdelta,
                c[-1][0].angle, 0)

    # TODO: add angle prediction.
    def _predict(self, hist):
        # General idea: To eliminate misdetections, we build sequential chains
        # of possible positions. If a position is too far from any existing
        # chains, it is inserted as a new chain. The chain with the most
        # positions is then considered the "actual" chain.
        chains = []
        for (pos, time) in zip(hist, self._history_times):
            # Try to add to an existing chain. If this fails, create a new one.
            if pos == None or time == None:
                continue
            deltas = [
                ((c[-1][0].x - pos.x)**2 + (c[-1][0].y - pos.y)**2) / (c[-1][1] - time)**2
                for c in chains
            ]
            (index, d) = min_with_index(deltas)
            if d != None and d <= Predictor._DELTA_THRESHOLD:
                chains[index].append((pos, time))
            else:
                chains.append([(pos, time)])
        longest_chain = None
        for chain in chains:
            if longest_chain == None or len(chain) >= len(longest_chain):
                longest_chain = chain
        if longest_chain:
            return self._derive_future(longest_chain)
        else:
            return None

    def predict(self):
        ret = Positions(
            self._predict(self._robot_blue_pink_history),
            self._predict(self._robot_blue_green_history),
            self._predict(self._robot_yellow_pink_history),
            self._predict(self._robot_yellow_green_history),
            self._predict(self._ball_history)
        )
        debug("Predition: " + str(ret))
        return ret
