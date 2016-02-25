from collections import namedtuple
from logging import debug
from Vision.world import Vector

Positions = namedtuple('Positions', 'robot_blue_pink robot_blue_green ' +
        'robot_yellow_pink robot_yellow_green ball')
Position = namedtuple('Position', 'x y angle')

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

    def _predict(self, hist):
        # General idea: To eliminate misdetections, we build sequential chains
        # of possible positions. If a position is too far from any existing
        # chains, it is inserted as a new chain. The chain with the most
        # positions is then considered the "actual" chain.
        chains = []
        chain_endtimes = []
        for (pos, time) in zip(hist, self._history_times):
            # Try to add to an existing chain. If this fails, create a new one.
            if pos == None or time == None:
                continue
            deltas = [
                ((c[-1].x - pos.x)**2 + (c[-1].y - pos.y)**2) / (ct - time)**2
                for (c, ct) in zip(chains, chain_endtimes)
            ]
            (index, d) = min_with_index(deltas)
            if d != None and d <= Predictor._DELTA_THRESHOLD:
                chains[index].append(pos)
                chain_endtimes[index] = time
            else:
                chains.append([pos])
                chain_endtimes.append(time)
        longest_chain = None
        for chain in chains:
            if longest_chain == None or len(chain) >= len(longest_chain):
                longest_chain = chain
        if longest_chain:
            return longest_chain[-1]
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
