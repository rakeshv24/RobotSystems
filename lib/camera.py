import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import logging
import math


class Camera():
    def __init__(self, resolution=(640, 480), framerate=24):
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.raw_cap = PiRGBArray(self.camera, size=self.camera.resolution)
        
    def blue_mask(self, frame):
        lowerBlue = np.array([60, 40, 40])
        upperBlue = np.array([150, 255, 255])
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        return cv2.inRange(hsv, lowerBlue, upperBlue)

    def detect_edges(self, frame):
        mask = self.blue_mask(frame)
        edges = cv2.Canny(mask, 200, 400)
        return edges

    def filter_ROI(self, edges):
        height, width = edges.shape
        mask = np.zeros_like(edges)

        # only focus bottom half of the screen
        polygon = np.array([[
            (0, height * 1 / 2),
            (width, height * 1 / 2),
            (width, height),
            (0, height),
        ]], np.int32)

        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)
        return cropped_edges

    def detect_line_segments(self, cropped_edges):
        # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
        rho = 1  # distance precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
        min_threshold = 10  # minimal of votes
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold,
                                        np.array([]), minLineLength=8, maxLineGap=4)

        return line_segments

    def avg_slope_intercept(self, frame, line_segments):
        """
        This function combines line segments into one or two lane lines
        If all line slopes are < 0: then we only have detected left lane
        If all line slopes are > 0: then we only have detected right lane
        """
        lane_lines = []
        if line_segments is None:
            logging.info('No line_segment segments detected')
            return lane_lines

        height, width, _ = frame.shape
        left_fit = []
        right_fit = []

        boundary = 1 / 3
        leftRegionBoundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
        rightRegionBoundary = width * boundary  # right lane line segment should be on left 2/3 of the screen

        for lineSegment in line_segments:
            for x1, y1, x2, y2 in lineSegment:
                if x1 == x2:
                    logging.info('skipping vertical line segment (slope=inf): %s' % lineSegment)
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    if x1 < leftRegionBoundary and x2 < leftRegionBoundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > rightRegionBoundary and x2 > rightRegionBoundary:
                        right_fit.append((slope, intercept))

        leftFitAvg = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            lane_lines.append(self.make_points(frame, leftFitAvg))

        rightFitAvg = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            lane_lines.append(self.make_points(frame, rightFitAvg))

        logging.debug('lane lines: %s' % lane_lines)

        return lane_lines

    def make_points(self, frame, line):
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height  # bottom of the frame
        y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]

    def detect_lane(self, frame):
        edges = self.detect_edges(frame)
        cropped_edges = self.filter_ROI(edges)
        line_segments = self.detect_line_segments(cropped_edges)
        lane_lines = self.avg_slope_intercept(frame, line_segments)

        return lane_lines

    def get_steering_angle(self, frame, lane_lines):
        height, width, _ = frame.shape

        if len(lane_lines) == 2:
            _, _, leftX2, _ = lane_lines[0][0]
            _, _, rightX2, _ = lane_lines[1][0]
            mid = int(width / 2)
            xOffset = (leftX2 + rightX2) / 2 - mid
            yOffset = int(height / 2)
        elif len(lane_lines) == 1:
            x1, _, x2, _ = lane_lines[0][0]
            xOffset = x2 - x1
            yOffset = int(height / 2)
        else:
            xOffset = 0
            yOffset = 1

        midRadianAngle = math.atan(xOffset / yOffset)  # angle (in radian) to center vertical line
        midDegAngle = int(midRadianAngle * 180.0 / math.pi)  # angle (in degrees) to center vertical line
        steering_angle = midDegAngle + 90  # this is the steering angle needed by picar front wheel

        return steering_angle

    def display_lane_lines(self, frame, lines, lineColor=(0, 255, 0), lineWidth=2):
        line_image = np.zeros_like(frame)

        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), lineColor, lineWidth)

        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        return line_image

    def display_heading_line(self, frame, steering_angle, lineColor=(0, 0, 255), lineWidth=5):
        heading_image = np.zeros_like(frame)
        height, width, _ = frame.shape

        # figure out the heading line from steering angle
        # heading line (x1,y1) is always center bottom of the screen
        # (x2, y2) requires a bit of trigonometry

        # Note: the steering angle of:
        # 0-89 degree: turn left
        # 90 degree: going straight
        # 91-180 degree: turn right
        steering_angle_radian = steering_angle / 180.0 * math.pi
        x1 = int(width / 2)
        y1 = height
        x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
        y2 = int(height / 2)

        cv2.line(heading_image, (x1, y1), (x2, y2), lineColor, lineWidth)
        heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

        return heading_image
