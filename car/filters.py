import numpy as np
import cv2
import math


class LineDetectionFilter:
    isInit = None
    avg_angle = None

    def __init__(self):
        self.isInit = False
        self.avg_angle = None

    def mean_angle(self, deg):
        return np.mean(deg)

    def calculate_angle(self, x1, y1, x2, y2):
        left = 0
        if (y2 >= y1):
            radians = math.atan2(y2 - y1, x2 - x1)
        else:
            radians = math.atan2(y1 - y2, x1 - x2)
        degrees = math.degrees(radians)
        # convert to 0(left)-180(right)
        # degrees = 180 - degrees
        return degrees

    def apply(self, frame):
        if (self.isInit == False):
            if (len(frame.shape) == 3):
                height, width, channels = frame.shape
            else:
                height, width = frame.shape
            self.isInit = True

        dst = cv2.Canny(frame, 50, 200, None, 3)

        # Copy edges to the images that will display the results in BGR
        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        cdstP = np.copy(cdst)

        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

        # initialize empty angles array
        angles = np.array([])
        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                #cv2.line(frame, (l[0], l[1]), (l[2], l[3]), (0, 255, 0), 3, cv2.LINE_AA)
                angles = np.append(angles, np.array([self.calculate_angle(l[0], l[1], l[2], l[3])]))

        lines = cv2.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)

        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                #cv2.line(frame, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)
                angles = np.append(angles, np.array([self.calculate_angle(int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)),
                                                                          int(x0 - 1000 * (-b)),
                                                                          int(y0 - 1000 * (a)))]))
        self.avg_angle = round(self.mean_angle(angles), 5)
        return frame


class FrameMaskFilter:
    isInit = None
    leftMaskPts = None
    rightMaskPts = None

    def __init__(self):
        self.isInit = False

    def apply(self, frame):
        if (self.isInit == False):
            height, width, channels = frame.shape
            self.leftMaskPts = np.array([[0, 0], [0, height], [(width / 3), 0]], np.int32)
            self.rightMaskPts = np.array([[width, 0], [width, height], [(width / 3) * 2, 0]], np.int32)
            self.isInit = True
        leftMaskedFrame = cv2.fillPoly(frame, [self.leftMaskPts], (0, 0, 0))
        fullMaskedFrame = cv2.fillPoly(frame, [self.rightMaskPts], (0, 0, 0))

        return fullMaskedFrame


class ColorThreshholdFilter:
    isInit = None
    lower_red = None
    upper_red = None

    def __init__(self):
        self.isInit = False

    def apply(self, frame, rgb_min, rgb_max):

        if ((rgb_min[0] + rgb_min[1] + rgb_min[2]) == 0):
            rgb_min[0] = 1

        if (self.isInit == False):
            lower = np.array(rgb_min)
            upper = np.array(rgb_max)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)

        return mask
