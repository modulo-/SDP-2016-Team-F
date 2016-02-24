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
    _DELTA_THRESHOLD = 30

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
        # General idea: To eliminate misdetections, we build sequential chains
        # of possible positions. If a position is too far from any existing
        # chains, it is inserted as a new chain. The chain with the most
        # positions is then considered the "actual" chain.
        chains = []
        for pos in hist:
            # Try to add to an existing chain. If this fails, create a new one.
            dists = [(c[-1].x - pos.x)**2 + (c[-1].y - pos.y)**2 for c in chains]
            (d, index) = min_with_index(dists)
            if d != None and d <= Predictor._DELTA_THRESHOLD:
                chains[index].append(pos)
            else:
                chains.append([pos])
        longest_chain = None
        for chain in chains:
            if longest_chain == None or len(chain) >= len(longest_chain):
                longest_chain = chain
        return longest_chain[-1]

    def predict(self):
        ret = []
        for i in range(5):
            ret.append(self._predict([h[i] for h in self._position_history]))
        ret = Positions(*ret)
        debug("Predition: " + str(ret))
        return ret
