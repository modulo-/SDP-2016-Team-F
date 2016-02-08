from postprocessing.postprocessing import Vector
from world import World
from planner import Planner
from math import pi

pos_dict = { 'our_attacker': Vector(50, 10, pi, 0), 'their_attacker': Vector(0, 0, 0, 0), 'our_defender': Vector(0, 0, 0, 0), 'their_defender': Vector(0, 0, 0, 0), 'ball': Vector(60, 60, 0, 0) }

w = World('left', 0)
w.our_defender._receiving_area = {'width': 30, 'height': 30, 'front_offset': 12}
w.our_attacker._receiving_area = {'width': 30, 'height': 30, 'front_offset': 12}
w.update_positions(pos_dict)

p = Planner()
p.plan_and_act(w)
