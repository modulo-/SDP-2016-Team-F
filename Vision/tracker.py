import Queue
import cv2
import math
import numpy as np
from collections import namedtuple
import warnings
import matplotlib.pyplot as plt
import filters

# Turn off warnings for PolynomialFit
warnings.simplefilter('ignore', np.RankWarning)
warnings.simplefilter('ignore', RuntimeWarning)

BoundingBox = namedtuple('BoundingBox', 'x y width height')
Center = namedtuple('Center', 'x y')


class Tracker(object):
    @staticmethod
    def oddify(inte):
        if inte == 0:
            inte += 1
        elif inte % 2 == 0:
            inte -= 1
        else:
            pass
        return inte

    def get_contours(self, frame, crop, colour, o_type=None):
        """
        Adjust the given frame based on 'min', 'max', 'contrast' and 'blur'
        keys in adjustments dictionary.
        """
        try:
            if o_type == 'BALL':
                frame = frame[crop[2]:crop[3], crop[0]:crop[1]]
            if frame is None:
                return None

            # Convert frame to HSV
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # print colour
            # Create a mask
            frame_mask = cv2.inRange(frame_hsv,
                                     colour['min'],
                                     colour['max'])

            # print frame_mask

            contours, hierarchy = cv2.findContours(
                frame_mask,
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE
            )

            return (contours, hierarchy, frame_mask)
        except:
            # bbbbb
            raise

    def get_contour_extremes(self, cnt):
        """
        Get extremes of a countour.
        """
        leftmost = tuple(cnt[cnt[:, :, 0].argmin()][0])
        rightmost = tuple(cnt[cnt[:, :, 0].argmax()][0])
        topmost = tuple(cnt[cnt[:, :, 1].argmin()][0])
        bottommost = tuple(cnt[cnt[:, :, 1].argmax()][0])
        return (leftmost,
                topmost,
                rightmost,
                bottommost)

    def get_bounding_box(self, points):
        """
        Find the bounding box given points by looking at the extremes of each coordinate.
        """
        leftmost = min(points, key=lambda x: x[0])[0]
        rightmost = max(points, key=lambda x: x[0])[0]
        topmost = min(points, key=lambda x: x[1])[1]
        bottommost = max(points, key=lambda x: x[1])[1]
        return BoundingBox(leftmost,
                           topmost,
                           rightmost - leftmost,
                           bottommost - topmost)

    def get_contour_corners(self, contour):
        """
        Get exact corner points for the plate given one contour.
        """
        if contour is not None:
            rectangle = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rectangle)
            return np.int0(box)

    def join_contours(self, contours):
        """
        Joins multiple contours together.
        """
        cnts = []
        for i, cnt in enumerate(contours):
            if cv2.contourArea(cnt) > 100:
                cnts.append(cnt)
        return reduce(lambda x, y: np.concatenate((x, y)),
                      cnts) if len(cnts) else None

    def get_largest_contour(self, contours):
        """
        Find the largest of all contours.
        """
        areas = [cv2.contourArea(c) for c in contours]
        return contours[np.argmax(areas)]

    def get_largest_contour_index(self, contours):
        """
        Find the largest of all contours.
        """
        areas = [cv2.contourArea(c) for c in contours]
        return np.argmax(areas)

    def get_smallest_contour(self, contours):
        """
        Find the smallest of all contours.
        """
        areas = [cv2.contourArea(c) for c in contours]
        ind = np.argsort(areas)
        # for i in range(len(ind)):
        #     if areas[ind[i]] > 5:
        #         return areas[ind[i]]
        return contours[np.argmin(areas)]

    def get_contour_centre(self, contour):
        """
        Find the center of a contour by minimum enclousing circle approximation.

        Returns: ((x, y), radius)
        """
        return cv2.minEnclosingCircle(contour)

    def get_angle(self, line, dot):
        """
        From dot to line
        """
        diff_x = dot[0] - line[0]
        diff_y = line[1] - dot[1]
        angle = np.arctan2(diff_y, diff_x) % (2 * np.pi)
        return angle


class RobotTracker(Tracker):
    def __init__(self,
                 color,
                 crop,
                 offset,
                 pitch,
                 name,
                 calibration):
        """
        Initialize tracker.

        Params:
            [string] color      the name of the color to pass in
            [(left-min, right-max, top-min, bot-max)]
                                crop  crop coordinates
            [int]       offset          how much to offset the coordinates
            [int]       pitch           the pitch we're tracking - used to find the right colors
            [string]    name            name for debug purposes
            [dict]      calibration     dictionary of calibration values
        """
        self.name = name
        self.crop = crop

        self.color = [calibration[color]]

        self.color_name = color
        self.offset = offset
        self.pitch = pitch
        self.calibration = calibration

    def get_plate(self, frame):
        """
        Given the frame to search, find a bounding rectangle for the green plate

        Returns:
            list of corner points
        """
        # Adjustments are colors and contrast/blur
        adjustments = self.calibration['plate']

        contours = self.get_contours(frame.copy(), self.crop, adjustments)[0]

        return self.get_contour_corners(self.join_contours(contours))

    def get_dot(self, frame, x_offset, y_offset):
        """
        Find center point of the black dot on the plate.

        Method:
            1. Assume that the dot is within some proximity of the center of the plate.
            2. Fill a dummy frame with black and draw white cirlce around to create a mask.
            3. Mask against the frame to eliminate any robot parts looking like dark dots.
            4. Use contours to detect the dot and return it's center.

        Params:
            frame       The frame to search
            x_offset    The offset from the uncropped image - to be added to the final values
            y_offset    The offset from the uncropped image - to be added to the final values
        """
        # Create dummy mask
        height, width, channel = frame.shape
        if height > 0 and width > 0:
            mask_frame = frame.copy()

            # Fill the dummy frame
            cv2.rectangle(mask_frame,
                          (0, 0),
                          (width, height),
                          (0, 0, 0),
                          -1)
            cv2.circle(mask_frame,
                       (width / 2, height / 2),
                       13,
                       (255, 255, 255),
                       -1)

            # Mask the original image
            mask_frame = cv2.cvtColor(mask_frame,
                                      cv2.COLOR_BGR2GRAY)
            frame = cv2.bitwise_and(frame,
                                    frame,
                                    mask=mask_frame)
            adjustment = self.calibration['blue']
            contours = self.get_contours(frame, self.crop, adjustment, 'dot')[0]
            if contours and len(contours) > 0:
                # Take the largest contour
                contour = self.get_smallest_contour(contours)
                (x, y), radius = self.get_contour_centre(contour)
                return Center(x + x_offset, y + y_offset)

    def find(self, frame, queue):
        """
        Retrieve coordinates for the robot, it's orientation and speed - if
        available.

        Process:
            1. Find green plate
            2. Create a smaller frame with just the plate
            3. Find dot inside the green plate (the smaller window)
            4. Use plate corner points from (1) to determine angle

        Params:
            [np.array] frame                - the frame to scan
            [multiprocessing.Queue] queue   - shared resource for process

        Returns:
            None. Result is put into the queue.
        """
        # Set up variables
        x = y = angle = None
        sides = direction = None
        plate_corners = None
        dot = front = None

        # Trim the image to only consist of one zone
        frame = frame[self.crop[2]:self.crop[3],
                self.crop[0]:self.crop[1]]

        cv2.imshow('frame', frame)
        cv2.destroyAllWindows()

        # (1) Find the plates
        plate_corners = self.get_plate(frame)

        # print plate_corners

        if plate_corners is not None:
            # Find the bounding box
            plate_bound_box = self.get_bounding_box(plate_corners)

            # set x and y coordinates
            x = plate_bound_box.x + plate_bound_box.width / 2
            y = plate_bound_box.y + plate_bound_box.height / 2

            if (plate_bound_box.width > 0
                and plate_bound_box.height > 0):
                # (2) Trim to create a smaller frame
                plate_frame = frame.copy()[
                              plate_bound_box.y:plate_bound_box.y \
                                                + plate_bound_box.height,
                              plate_bound_box.x:plate_bound_box.x + \
                                                plate_bound_box.width
                              ]

                # (3) Search for the dot
                dot = self.get_dot(plate_frame, plate_bound_box.x \
                                   + self.offset,
                                   plate_bound_box.y)

                if dot is not None:
                    # Since get_dot adds offset, we need to remove it
                    dot_temp = Center(dot[0] - self.offset, dot[1])

                    # Find two points from plate_corners that are the furthest from the dot

                    distances = [
                        (
                            (dot_temp.x - p[0]) ** 2 + \
                            (dot_temp.y - p[1]) ** 2,  # distance
                            p[0],  # x coord
                            p[1]  # y coord
                        ) for p in plate_corners]

                    distances.sort(key=lambda x: x[0],
                                   reverse=True)

                    # Front of the kicker should be the first two points in distances
                    front = distances[:2]
                    rear = distances[2:]

                    # Calculate which of the rear points belongs to the first of the front
                    first = front[0]
                    front_rear_distances = [
                        (
                            (first[1] - p[0]) ** 2 + (first[2] - p[1]) ** 2,
                            p[1],
                            p[2]
                        ) for p in rear]
                    front_rear_distances.sort(key=lambda x: x[0])

                    # Put the results together
                    sides = [
                        (
                            Center(first[1], first[2]),
                            Center(front_rear_distances[0][1],
                                   front_rear_distances[0][2])
                        ),
                        (
                            Center(front[1][1], front[1][2]),
                            Center(front_rear_distances[1][1],
                                   front_rear_distances[1][2])
                        )
                    ]

                    # Direction is a line between the front points and rear points
                    direction = (
                        Center(
                            (first[1] + front[1][1]) / 2 + self.offset,
                            (front[1][2] + first[2]) / 2),
                        Center(
                            (front_rear_distances[1][1] + front_rear_distances[0][1]) \
                            / 2 + self.offset,
                            (front_rear_distances[1][2] + front_rear_distances[0][2]) / 2)
                    )

                    angle = self.get_angle(direction[1], direction[0])

            # Offset the x coordinates
            plate_corners = [(p[0] + self.offset, p[1]) for p in plate_corners]

            if front is not None:
                front = [(p[1] + self.offset, p[2]) for p in front]

            queue.put({
                'x': x + self.offset, 'y': y,
                'name': self.name,
                'angle': angle,
                'dot': dot,
                'box': plate_corners,
                'direction': direction,
                'front': front
            })

        queue.put({
            'x': None, 'y': None,
            'name': self.name,
            'angle': None,
            'dot': None,
            'box': None,
            'direction': None,
            'front': None
        })
        pass


class DotTracker(Tracker):
    """
    Track red ball on the pitch.
    """

    def __init__(self,
                 offset,
                 pitch,
                 colour,
                 config,
                 item='ball'):
        """
        Initialize tracker.

        Params:
            [string] color      the name of the color to pass in
            [(left-min, right-max, top-min, bot-max)]
                                crop  crop coordinates
            [int] offset        how much to offset the coordinates
        """
        # if pitch == 0:
        #     self.color = PITCH0['red']
        # else:
        #     self.color = PITCH1['red']
        from collections import defaultdict

        self.config = config
        self.colour_name = colour
        self.colour = defaultdict(int, self.config.colours[colour])
        self.offset = offset
        self.item = item

    def find(self, frame, queue, colour_name=None, corner=False):
        if colour_name is None:
            colour = self.colour
        else:
            colour = self.config.colours[colour_name]
        """
        contours, hierarchy, mask = self.preprocess(
            frame,
            self.crop,
            color['min'],
            color['max'],
            color['contrast'],
            color['blur']
        )
        """
        # adjustments = {'min':,'mz'}
        w, h, d = frame.shape
        contours, hierarchy, mask = self.get_contours(frame,
                                                      (0, h, 0, w),
                                                      colour,
                                                      'BALL')
        if len(contours) <= 0:
            print 'No {} {} found.'.format(self.colour_name, self.item)
            return
        if len(contours) < 2 and self.item == 'robot':
            print 'Only {} {} {} found.'.format(len(contours), self.colour_name, self.item)

        if self.item == 'ball' or corner:
            cnt = self.get_largest_contour(contours)
            if cv2.contourArea(cnt) < self.config.dot_areas[self.colour_name]:
                return

            # Get center
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            queue.put({
                'name': self.item,
                'x': x,
                'y': y,
                'area': cv2.contourArea(cnt),
                'colour': self.colour_name
            })
        else:
            for i in xrange(2):
                # Trim contours matrix
                if len(contours) < 1:
                    break

                cnt_index = self.get_largest_contour_index(contours)
                cnt = contours.pop(cnt_index)
                if cv2.contourArea(cnt) < self.config.dot_areas[self.colour_name]:
                    break

                # Get center
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                croprange = 20

                section = self.crop(frame, (x, y), croprange)
                identification = self.getID(section)

                assert identification in ['pink', 'green']
                corner = self.getCorner(section, (x,y), identification)

                if corner is not None:
                    corner['y'] = corner['y']-croprange
                    corner['x'] = corner['x']-croprange
                    orientation = math.degrees(math.atan2(-(y-corner['y']), (x-corner['x']))) + self.config.delta_angle
                else:
                    orientation = None

                queue.put({
                    'name': self.item + '_' + str(i),
                    'x': x,
                    'y': y,
                    'corner': corner,
                    'orientation': orientation,
                    'identification': identification,
                    'area': cv2.contourArea(cnt),
                    'colour': self.colour_name
                })

        queue.put(None)

    def getID(self, section):
        pinkMask = filters.filter_colour("pink", section, self.config)
        greenMask = filters.filter_colour("green", section, self.config)

        pinkedness = np.sum(pinkMask)
        greenness = np.sum(greenMask)

        return "pink" if pinkedness > greenness else "green"

    def getCorner(self, section, (x,y), identification):
        if (identification == 'pink'):
            leftcorner = 'green'
        else:
            leftcorner = 'pink'

        q = Queue.Queue()

        self.find(section, q, leftcorner, corner=True)
        try:
            corner = q.get_nowait()
        except Queue.Empty:
            return None

        return {"x":corner['x']+x, "y":corner['y']+y}

    def crop(self, frame, (x,y), croprange):
        h, w, d = frame.shape
        left = int(max(0, x - croprange))
        right = int(min(w, x + croprange))
        top = int(max(0, y - croprange))
        bottom = int(min(h, y + croprange))

        section = frame[top:bottom, left:right]
        return section
