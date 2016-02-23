import cv2
import tools
from tracker import BallTracker, RobotTracker
from multiprocessing import Process, Queue
from colors import BGR_COMMON
from collections import namedtuple
import numpy as np
from findHSV import CalibrationGUI


TEAM_COLORS = set(['yellow', 'blue'])
SIDES = ['left', 'right']
PITCHES = [0, 1]

PROCESSING_DEBUG = False

Center = namedtuple('Center', 'x y')


class _Vision:
    """
    Locate objects on the pitch.
    """

    def __init__(self,
                 pitch,
                 color,
                 our_side,
                 frame_shape,
                 frame_center,
                 calibration):
        """
        Initialize the oldvision system.

        Params:
            [int] pitch         pitch number (0 or 1)
            [string] color      color of our robot
            [string] our_side   our side
        """
        self.pitch = pitch
        self.color = color
        self.our_side = our_side
        self.frame_center = frame_center

        height, width, channels = frame_shape

        # Find the zone division
        self.zones = zones = self._get_zones(width, height)

        opponent_color = self._get_opponent_color(color)

        if our_side == 'left':
            self.us = [
                RobotTracker(
                    color=color,
                    crop=(0, width, 0, height),
                    offset=0,
                    pitch=pitch,
                    name='Our Defender',
                    calibration=calibration),   # defender
                RobotTracker(
                    color=color,
                    crop=(0, width, 0, height),
                    offset=0,
                    pitch=pitch,
                    name='Our Attacker',
                    calibration=calibration)   # attacker
            ]

            self.opponents = [
                RobotTracker(
                    color=opponent_color,
                    crop=(0, width, 0, height),
                    offset=0,
                    pitch=pitch,
                    name='Their Defender',
                    calibration=calibration),
                RobotTracker(
                    color=opponent_color,
                    crop=(0, width, 0, height),
                    offset=0,
                    pitch=pitch,
                    name='Their Attacker',
                    calibration=calibration)

            ]
        else:
            self.us = [
                RobotTracker(
                    color=color,
                    crop=(0, width, 0, height),
                    offset=0,
                    pitch=pitch,
                    name='Our Defender',
                    calibration=calibration),
                RobotTracker(
                    color=color,
                    crop=(0, width, 0, height),
                    offset=0,
                    pitch=pitch,
                    name='Our Attacker',
                    calibration=calibration)
            ] 

            self.opponents = [
                RobotTracker(
                    color=opponent_color,
                    crop=(0, width, 0, height),
                    offset=0,
                    pitch=pitch,
                    name='Their Defender',
                    calibration=calibration),   # defender
                RobotTracker(
                    color=opponent_color,
                    crop=(0, width, 0, height),
                    offset=0,
                    pitch=pitch,
                    name='Their Attacker',
                    calibration=calibration)
            ]

        # Set up trackers
        self.ball_tracker = BallTracker(
            (0, width, 0, height), 0, pitch, calibration)

    def _get_zones(self, width, height):
        return [(val[0], val[1], 0, height)
                for val in tools.get_zones(width,
                                           height,
                                           pitch=self.pitch)]

    def _get_opponent_color(self, our_color):
        return (TEAM_COLORS - set([our_color])).pop()

    def locate(self, frame):
        """
        Find objects on the pitch using multiprocessing.

        Returns:
            [5-tuple] Location of the robots and the ball
        """
        # Run trackers as processes
        positions = self._run_trackers(frame)
        # Correct for perspective
        positions = self.get_adjusted_positions(positions)

        # Wrap list of positions into a dictionary
        keys = ['our_defender',
                'our_attacker',
                'their_defender',
                'their_attacker',
                'ball']
        regular_positions = dict()
        for i, key in enumerate(keys):
            regular_positions[key] = positions[i]

        # Error check we got a frame
        height, width, channels = frame.shape if frame is not None \
            else (None, None, None)

        model_positions = {
            'our_attacker': self.to_info(positions[1], height),
            'their_attacker': self.to_info(positions[3], height),
            'our_defender': self.to_info(positions[0], height),
            'their_defender': self.to_info(positions[2], height),
            'ball': self.to_info(positions[4], height)
        }

        return model_positions, regular_positions

    def get_adjusted_point(self, point):
        """
        Given a point on the plane, calculate the
        adjusted point, by taking into account the
        height of the robot, the height of the
        camera and the distance of the point
        from the center of the lens.
        """
        plane_height = 250.0
        # TWEAK
        robot_height = 18.0
        coefficient = robot_height/plane_height

        x = point[0]
        y = point[1]

        dist_x = float(x - self.frame_center[0])
        dist_y = float(y - self.frame_center[1])

        delta_x = dist_x * coefficient
        delta_y = dist_y * coefficient

        return (int(x-delta_x), int(y-delta_y))

    def get_adjusted_positions(self, positions):
        print positions
        try:
            for robot in range(4):
                # Adjust each corner of the plate
                for i in range(4):
                    x = positions[robot]['box'][i][0]
                    y = positions[robot]['box'][i][1]
                    positions[robot]['box'][i] = self.get_adjusted_point((x, y))

                new_direction = []
                for i in range(2):
                    # Adjust front line
                    x = positions[robot]['front'][i][0]
                    y = positions[robot]['front'][i][1]
                    positions[robot]['front'][i] = self.get_adjusted_point((x, y))

                    # Adjust direction line
                    x = positions[robot]['direction'][i][0]
                    y = positions[robot]['direction'][i][1]
                    adj_point = self.get_adjusted_point((x, y))
                    new_direction.append(adj_point)

                # Change the namedtuples used for storing direction points
                positions[robot]['direction'] = (
                    Center(new_direction[0][0], new_direction[0][1]),
                    Center(new_direction[1][0], new_direction[1][1]))

                # Adjust the center point of the plate
                x = positions[robot]['x']
                y = positions[robot]['y']
                new_point = self.get_adjusted_point((x, y))
                positions[robot]['x'] = new_point[0]
                positions[robot]['y'] = new_point[1]
        except:
            # At least one robot has not been found
            pass

        return positions

    def _run_trackers(self, frame):
        """
        Run trackers as separate processes

        Params:
            [np.frame] frame        - frame to run trackers on

        Returns:
            [5-tuple] positions     - locations of the robots and the ball
        """
        queues = [Queue() for i in range(5)]
        objects = [self.us[0],
                   self.us[1],
                   self.opponents[0],
                   self.opponents[1],
                   self.ball_tracker]

        # Define processes
        processes = [
            Process(target=obj.find, args=((frame, queues[i])))
                                            for (i, obj)
                                            in enumerate(objects)]

        # Start processes
        for process in processes:
            process.start()

        # Find robots and ball, use queue to
        # avoid deadlock and share resources
        positions = [q.get() for q in queues]

        # terminate processes
        for process in processes:
            process.join()

        return positions

    def to_info(self, args, height):
        """
        Returns a dictionary with object position information
        """
        x, y, angle, velocity = None, None, None, None
        if args is not None:
            if 'x' in args and 'y' in args:
                x = args['x']
                y = args['y']
                if y is not None:
                    y = height - y

            if 'angle' in args:
                angle = args['angle']

            if 'velocity' in args:
                velocity = args['velocity']

        return {'x': x, 'y': y, 'angle': angle, 'velocity': velocity}
