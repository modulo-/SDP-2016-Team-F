def get_rotation_to_point(self_x, self_y, self_angle, x, y):
        '''
        This method returns an angle by which the robot needs to rotate to achieve alignment.
        It takes either an x, y coordinate of the object that we want to rotate to
        positive angle - clockwise rotation
        negative angle - counter-clockwise rotation
        '''
        delta_x = x - self_x
        delta_y = y - self_y
        print("get_rotation_to_point from ({4} {5}) facing {6} to ({0} {1}) deltas ({2} {3})".format(x, y, delta_x, delta_y, self_x, self_y, self_angle))
        displacement = hypot(delta_x, delta_y)
        if displacement == 0:
            theta = 0
        else:
            theta = atan2(delta_y, delta_x) - self.angle  # atan2(sin(self.angle), cos(self.angle))
            if theta > pi:
                theta -= 2 * pi
            elif theta < -pi:
                theta += 2 * pi
        assert -pi <= theta <= pi
        print ("rotation = {0}".format(-theta))
        return -theta

