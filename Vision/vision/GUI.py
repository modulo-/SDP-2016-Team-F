import cv2
import tools
from colors import BGR_COMMON
from collections import namedtuple
import numpy as np
from findHSV import CalibrationGUI


"""
Some of these stupid constants are not used
ANYWHERE LIKE  PROCESSING_DEBUG FFS.
Must work out what to do with them.
"""

TEAM_COLORS = set(['yellow', 'blue'])
SIDES = ['left', 'right']
PITCHES = [0, 1]

PROCESSING_DEBUG = False

Center = namedtuple('Center', 'x y')

class GUI(object):

    VISION = 'VISION'
    BG_SUB = 'BG Subtract'
    NORMALIZE = 'Normalize  '
    # TODO: 
    COMMS = 'Communications on/off '
    RADIAL_DIST = "Undistort on/off"

    def nothing(self, x):
        pass

    def __init__(self, calibration, pitch):
        self.zones = None
        # print calibration
        self.calibration_gui = CalibrationGUI(calibration)
        # self.arduino = arduino
        self.pitch = pitch

        cv2.namedWindow(self.VISION)

        cv2.createTrackbar(self.BG_SUB, self.VISION, 0, 1, self.nothing)
        cv2.createTrackbar(self.NORMALIZE, self.VISION, 0, 1, self.nothing)
        # cv2.createTrackbar(self.RADIAL_DIST, self.VISION, 0, 1, self.nothing)
        # cv2.createTrackbar(
        #     self.COMMS, self.VISION, self.arduino.comms, 1, lambda x:  self.arduino.setComms(x))

    def to_info(self, args):
        """
        Convert a tuple into a vector

        Return a Vector
        """
        x, y, angle, velocity = None, None, None, None
        if args is not None:
            if 'location' in args:
                x = args['location'][0] if args['location'] is not None else None
                y = args['location'][1] if args['location'] is not None else None

            elif 'x' in args and 'y' in args:
                x = args['x']
                y = args['y']

            if 'angle' in args:
                angle = args['angle']

            if 'velocity' in args:
                velocity = args['velocity']

        return {'x': x, 'y': y, 'angle': angle, 'velocity': velocity}

    def cast_binary(self, x):
        return x == 1

    def draw(self, frame, model_positions, actions, regular_positions, fps,
             aState, dState, a_action, d_action, grabbers, our_color, our_side,
             key=None, preprocess=None):
        """
        Draw information onto the GUI given positions from the vision and post processing.

        NOTE: model_positions contains coordinates with y coordinate reversed!
        """
        # Get general information about the frame
        frame_height, frame_width, channels = frame.shape

        # Draw the calibration gui
        self.calibration_gui.show(frame, key)
        # Draw dividors for the zones
        self.draw_zones(frame, frame_width, frame_height)

        their_color = list(TEAM_COLORS - set([our_color]))[0]

        key_color_pairs = zip(
            ['our_defender', 'their_defender', 'our_attacker', 'their_attacker'],
            [our_color, their_color]*2)

        self.draw_ball(frame, regular_positions['ball'])

        for key, color in key_color_pairs:
            self.draw_robot(frame, regular_positions[key], color)

        # Draw fps on the canvas
        if fps is not None:
            self.draw_text(frame, 'FPS: %.1f' % fps, 0, 10, BGR_COMMON['green'], 1)

        if preprocess is not None:
            # print preprocess
            preprocess['normalize'] = self.cast_binary(
                cv2.getTrackbarPos(self.NORMALIZE, self.VISION))
            preprocess['background_sub'] = self.cast_binary(
                cv2.getTrackbarPos(self.BG_SUB, self.VISION))

        if grabbers:
            self.draw_grabbers(frame, grabbers, frame_height)

        # Extend image downwards and draw states.
        blank = np.zeros_like(frame)[:200, :, :]
        frame_with_blank = np.vstack((frame, blank))
        # self.draw_states(frame_with_blank, aState, dState, (frame_width, frame_height))

        if model_positions and regular_positions:
            for key in ['ball', 'our_defender', 'our_attacker', 'their_defender', 'their_attacker']:
                if model_positions[key] and regular_positions[key]:
                    self.data_text(
                        frame_with_blank, (frame_width, frame_height), our_side, key,
                        model_positions[key].x, model_positions[key].y,
                        model_positions[key].angle, model_positions[key].velocity)
                    self.draw_velocity(
                        frame_with_blank, (frame_width, frame_height),
                        model_positions[key].x, model_positions[key].y,
                        model_positions[key].angle, model_positions[key].velocity)

        # Draw center of uncroppped frame (test code)
        # cv2.circle(frame_with_blank, (266,147), 1, BGR_COMMON['black'], 1)

        cv2.imshow(self.VISION, frame_with_blank)

    def draw_zones(self, frame, width, height):
        # Re-initialize zones in case they have not been initalized
        if self.zones is None:
            self.zones = tools.get_zones(width, height, pitch=self.pitch)

        for zone in self.zones:
            cv2.line(frame, (zone[1], 0), (zone[1], height), BGR_COMMON['orange'], 1)

    def draw_ball(self, frame, position_dict):
        if position_dict and position_dict['x'] and position_dict['y']:
            frame_height, frame_width, _ = frame.shape
            self.draw_line(
                frame, ((int(position_dict['x']), 0), (int(position_dict['x']), frame_height)), 1)
            self.draw_line(
                frame, ((0, int(position_dict['y'])), (frame_width, int(position_dict['y']))), 1)

    def draw_dot(self, frame, location):
        if location is not None:
            cv2.circle(frame, location, 2, BGR_COMMON['white'], 1)

    def draw_robot(self, frame, position_dict, color):
        if position_dict['box']:
            cv2.polylines(frame, [np.array(position_dict['box'])], True, BGR_COMMON[color], 2)

        if position_dict['front']:
            p1 = (position_dict['front'][0][0], position_dict['front'][0][1])
            p2 = (position_dict['front'][1][0], position_dict['front'][1][1])
            cv2.circle(frame, p1, 3, BGR_COMMON['white'], -1)
            cv2.circle(frame, p2, 3, BGR_COMMON['white'], -1)
            cv2.line(frame, p1, p2, BGR_COMMON['red'], 2)

        if position_dict['dot']:
            cv2.circle(
                frame, (int(position_dict['dot'][0]), int(position_dict['dot'][1])),
                4, BGR_COMMON['black'], -1)

        if position_dict['direction']:
            cv2.line(
                frame, position_dict['direction'][0], position_dict['direction'][1],
                BGR_COMMON['orange'], 2)

    def draw_line(self, frame, points, thickness=2):
        if points is not None:
            cv2.line(frame, points[0], points[1], BGR_COMMON['red'], thickness)

    def data_text(self, frame, frame_offset, our_side, text, x, y, angle, velocity):

        if x is not None and y is not None:
            frame_width, frame_height = frame_offset
            if text == "ball":
                y_offset = frame_height + 130
                draw_x = 30
            else:
                x_main = lambda zz: (frame_width/4)*zz
                x_offset = 30
                y_offset = frame_height+20

                if text == "our_defender":
                    draw_x = x_main(0) + x_offset
                elif text == "our_attacker":
                    draw_x = x_main(2) + x_offset
                elif text == "their_defender":
                    draw_x = x_main(3) + x_offset
                else:
                    draw_x = x_main(1) + x_offset

                if our_side == "right":
                    draw_x = frame_width-draw_x - 80

            self.draw_text(frame, text, draw_x, y_offset)
            self.draw_text(frame, 'x: %.2f' % x, draw_x, y_offset + 10)
            self.draw_text(frame, 'y: %.2f' % y, draw_x, y_offset + 20)

            if angle is not None:
                self.draw_text(frame, 'angle: %.2f' % angle, draw_x, y_offset + 30)

            if velocity is not None:
                self.draw_text(frame, 'velocity: %.2f' % velocity, draw_x, y_offset + 40)


    def draw_text(self, frame, text, x, y, color=BGR_COMMON['green'], thickness=1.3, size=0.3,):
        if x is not None and y is not None:
            cv2.putText(
                frame, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness)

    def draw_grabbers(self, frame, grabbers, height):
        def_grabber = grabbers['our_defender'][0]
        att_grabber = grabbers['our_attacker'][0]

        def_grabber = [(x, height - y) for x, y in def_grabber]
        att_grabber = [(x, height - y) for x, y in att_grabber]

        def_grabber = [(int(x) if x > -1 else 0, int(y) if y > -1 else 0) for x, y in def_grabber]
        att_grabber = [(int(x) if x > -1 else 0, int(y) if y > -1 else 0) for x, y in att_grabber]

        def_grabber[2], def_grabber[3] = def_grabber[3], def_grabber[2]
        att_grabber[2], att_grabber[3] = att_grabber[3], att_grabber[2]

        cv2.polylines(frame, [np.array(def_grabber)], True, BGR_COMMON['red'], 1)
        cv2.polylines(frame, [np.array(att_grabber)], True, BGR_COMMON['red'], 1)

    def draw_velocity(self, frame, frame_offset, x, y, angle, vel, scale=10):
        if not(None in [frame, x, y, angle, vel]) and vel is not 0:
            frame_width, frame_height = frame_offset
            r = vel*scale
            y = frame_height - y
            start_point = (x, y)
            end_point = (x + r * np.cos(angle), y - r * np.sin(angle))
            self.draw_line(frame, (start_point, end_point))

    def draw_states(self, frame, aState, dState, frame_offset):
        frame_width, frame_height = frame_offset
        x_main = lambda zz: (frame_width/4)*zz
        x_offset = 20
        y_offset = frame_height+140

        self.draw_text(frame, "Attacker State:", x_main(1) - x_offset, y_offset, size=0.6)
        self.draw_text(frame, aState[0], x_main(1) - x_offset, y_offset + 15, size=0.6)
        self.draw_text(frame, aState[1], x_main(1) - x_offset, y_offset + 30, size=0.6)

        self.draw_text(frame, "Defender State:", x_main(2) + x_offset, y_offset, size=0.6)
        self.draw_text(frame, dState[0], x_main(2) + x_offset, y_offset + 15, size=0.6)
        self.draw_text(frame, dState[1], x_main(2)+x_offset, y_offset + 30, size=0.6)

    def draw_actions(self, frame, action, x, y):
        self.draw_text(
            frame, "Left Motor: " + str(action['left_motor']), x, y+5, color=BGR_COMMON['white'])
        self.draw_text(
            frame, "Right Motor: " + str(action['right_motor']), x, y+15, color=BGR_COMMON['white'])
        self.draw_text(
            frame, "Speed: " + str(action['speed']), x, y + 25, color=BGR_COMMON['white'])
        self.draw_text(frame, "Kicker: " + str(action['kicker']), x, y + 35, color=BGR_COMMON['white'])
        self.draw_text(frame, "Catcher: " + str(action['catcher']), x, y + 45, color=BGR_COMMON['white'])
