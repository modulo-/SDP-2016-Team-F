from position import Vector
from world import World
from planner import Planner

pos_dict = {'our_robot': Vector(0, 0, 0, 0), 'ball': Vector(60, 60, 0, 0)}

w = World('left', 0)
w.our_robot._receiving_area = {'width': 30, 'height': 30, 'front_offset': 12}
w.update_positions(pos_dict)

p = Planner()
p.plan_and_act(w)
