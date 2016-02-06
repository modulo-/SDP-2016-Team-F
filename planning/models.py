class Goal (object):
    def __init__(self, world):
        self.world = world

    def generate_action(self):
        raise NotImplemented

class Action (object):
    preconditions = []
    def is_possible(self, world):
        for condition in self.preconditions:
            if not condition(world):
                return False
        return True

    def perform(self):
        raise NotImplemented
