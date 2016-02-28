import Queue
import cv2
import math
import numpy as np
from collections import namedtuple
import warnings
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
            
            #
            # if self.config.open >= 1:
            #     kernel = np.ones((2,2),np.uint8)
            #     frame_mask = cv2.morphologyEx(frame_mask,
            #                                   cv2.MORPH_OPEN,
            #                                   kernel,
            #                                   iterations=self.config.open)
            #
            # if self.config.close >= 1:
            #     kernel = np.ones((2,2),np.uint8)
            #     frame_mask = cv2.dilate(frame_mask,
            #                             kernel,
            #                             iterations=self.config.close)
            #
            # if self.config.erode >= 1:
            #     kernel = np.ones((2,2),np.uint8)
            #     frame_mask = cv2.erode(frame_mask,
            #                             kernel,
            #                             iterations=self.config.erode)
            #
            # if self.config.dilate >= 1:
            #     kernel = np.ones((2,2),np.uint8)
            #     frame_mask = cv2.dilate(frame_mask,
            #                             kernel,
            #                             iterations=self.config.dilate)

            # print frame_mask

            stuff = cv2.findContours(
                frame_mask,
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE
            )

            if len(stuff)==3:
                _, contours, hierarchy = stuff
            else:
                contours, hierarchy = stuff

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
        self.colour = defaultdict(lambda:[0,0,0], self.config.colours[colour])
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
            #print 'No {} {} found.'.format(self.colour_name, self.item)
            return
        #if len(contours) < 2 and self.item == 'robot':
            #print 'Only {} {} {} found.'.format(len(contours), self.colour_name, self.item)

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
            robots_found = 0
            
            while(1):
                # Trim contours matrix
                if len(contours) < 1 or robots_found == 2:
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
                
                if identification == None:
                    continue
                
                robots_found += 1

                assert identification in ['pink', 'green']
                corner = self.getCorner(section, (x,y), identification)

                if corner is not None:
                    corner['y'] = corner['y']-croprange
                    corner['x'] = corner['x']-croprange
                    orientation = -math.degrees(math.atan2((y-corner['y']), (x-corner['x']))) + self.config.delta_angle
                else:
                    orientation = None

                queue.put({
                    'name': self.item + '_' + str(robots_found - 1),
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

        minimum = 10
        if pinkedness < minimum or greenness < minimum:
            return None
    
        h, w, d = section.shape
        
        contours, _, _ = self.get_contours(section, (0, w, 0, h), self.config.colours['pink'])

        if(len(contours) >= 3):
            return "pink"
        
        return "green"

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
