#!/usr/bin/python3
# coding=utf8

import cv2
import time
import rospy
import numpy as np
from threading import RLock, Timer

from std_srvs.srv import *
from sensor_msgs.msg import Image

from sensor.msg import Led
from object_tracking.srv import *
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from kinematics import ik_transform
from armpi_fpv import bus_servo_control

ik = ik_transform.ArmIK()

lock = RLock()

size = (320, 240)
start_move = True
__isRunning = False
org_image_sub_ed = False

x_start = 0.0
y_start = 0.1
z_start = 0.20


def initMove(delay=True):
    with lock:
        target = ik.setPitchRanges((x_start, y_start, z_start), -90, -92, -88)
        if target:
            servo_data = target[1]
            bus_servo_control.set_servos(joints_pub, 1500, ((
                1, 200), (2, 500), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
    if delay:
        rospy.sleep(2)


def turn_off_rgb():
    led = Led()
    led.index = 0
    led.rgb.r = 0
    led.rgb.g = 0
    led.rgb.b = 0
    rgb_pub.publish(led)
    led.index = 1
    rgb_pub.publish(led)


def reset():
    with lock:
        turn_off_rgb()


def init():
    rospy.loginfo("Image tracing Init")
    initMove()
    reset()


def run(features):
    global start_move

    # SQUARE
    # points = np.array([[100, 60], [220, 180], [100, 180], [220, 60], [100, 60]])

    # TRIANGLE
    # points = np.array([[160, 60], [220, 180], [100, 180], [160, 60]])

    # CIRCLE
    # theta = np.linspace(0, 2*np.pi, 10).reshape(-1, 1)
    # r = 100
    # cX = r * np.cos(theta) + (size[0] / 2)
    # cY = r * np.sin(theta) + (size[1] / 2)
    # points = np.hstack((cX, cY))

    for points in features:
        x_move = 0.0
        x_min, x_max = -.02, .02
        z_min, z_max = .01, .05

        sorted_points = np.array([points[0]])

        i = 0
        while len(sorted_points) != len(points):
            min_dist = 99999
            p1 = sorted_points[i]
            for j in range(len(points)):
                p2 = points[j]
                flag = 0
                for k in sorted_points:
                    if p2[0] == k[0] and p2[1] == k[1]:
                        flag = 1
                        break
                if flag == 1:
                    continue
                else:
                    dist = np.linalg.norm(p2 - p1)
                    if dist < min_dist:
                        min_dist = dist
                        next_wp = p2

            sorted_points = np.insert(sorted_points, len(sorted_points), next_wp, axis=0)
            i += 1

        sorted_points = np.insert(sorted_points, len(sorted_points), sorted_points[0], axis=0)
        print(f"Waypoints: {sorted_points}")

        if start_move:
            for i in range(len(sorted_points)):
                x, z = sorted_points[i].ravel()

                x = (size[0] / 2) - x
                z = (size[1] / 2) - z

                print(f"x_im:{x}, z_im:{z}")

                x = np.sign(x) * (((abs(x) * (x_max - x_min)) / size[0]) + x_min)
                z = np.sign(z) * (((abs(z) * (z_max - z_min)) / size[1]) + z_min)

                x_move = round(x + x_start, 3)
                z_move = round(z + z_start, 3)

                print(f"x_rw:{x}, z_rw:{z}")
                print(f"x_move:{x_move}, z_move:{z_move}")
                target = ik.setPitchRanges((x_move, 0.10, z_move), -90, -100, -80)
                print(f"target: {target}\n")

                if target:
                    servo_data = target[1]
                    bus_servo_control.set_servos(joints_pub, 20, (
                        (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))

                time.sleep(2)

        time.sleep(2)
        initMove()


heartbeat_timer = None


def start_running():
    global lock
    global __isRunning

    rospy.loginfo("Initialize arm")
    init()

    rospy.loginfo("Start image tracing")
    with lock:
        __isRunning = True

        img = cv2.imread('/home/ubuntu/shapes.png')
        imgObj = Perception(img)
        imgObj.image_processing()
        imgObj.find_contours()
        features = imgObj.feature_detection()

        run(features)

        ros_image = Image()
        rgb_image = cv2.cvtColor(imgObj.org_image, cv2.COLOR_BGR2RGB).tostring()
        ros_image.data = rgb_image

        image_pub.publish(ros_image)

        __isRunning = False


def stop_running():
    global lock
    global __isRunning

    rospy.loginfo("stop running object tracking")
    with lock:
        __isRunning = False
        reset()
        initMove(delay=False)


def set_running(msg):
    if msg.data:
        start_running()
    else:
        stop_running()

    return [True, 'set_running']


def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/image_tracer/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp


class Perception():
    def __init__(self, img):
        self.img = img
        self.img_copy = img.copy()
        self.width = 320
        self.height = 240

    def image_processing(self):
        self.resized_image = cv2.resize(self.img, (self.width, self.height), interpolation=cv2.INTER_AREA)
        self.org_image = self.resized_image.copy()
        self.gray_image = cv2.cvtColor(self.resized_image, cv2.COLOR_BGR2GRAY)
        self.blur_image = cv2.GaussianBlur(self.gray_image, (5, 5), 0)
        self.thresh_image = cv2.threshold(self.blur_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        self.morph_image = self.morph_transformation(self.thresh_image)
        self.canny_image = self.edge_detection(self.morph_image)

    def feature_detection(self):
        features = []
        for img in self.cont_imgs:
            gray = np.float32(img)
            corners = cv2.goodFeaturesToTrack(gray, 8, 0.01, 50)
            if corners is not None:
                corners = np.int0(corners).reshape(-1, 2)

                for i in corners:
                    x, y = i.ravel()
                    cv2.circle(self.org_image, (x, y), 4, 200, -1)

                features.append(corners)

        features = np.array(features)
        return features

    def morph_transformation(self, frame):
        kernel = np.ones((5, 5), np.uint8)
        # opening = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, kernel)
        opening = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)
        return opening

    def edge_detection(self, frame, sigma=0.33):
        # compute the median of the single channel pixel intensities
        v = np.median(frame)

        # apply automatic Canny edge detection using the computed median
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edged = cv2.Canny(frame, lower, upper)

        return edged

    def find_contours(self):
        self.contours = cv2.findContours(self.canny_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
        self.cont_imgs = []

        for _, c in enumerate(self.contours):
            contArea = cv2.contourArea(c)
            minArea = 999

            if contArea > minArea:
                cont_img = np.zeros((self.height, self.width), dtype=np.uint8)
                cv2.drawContours(cont_img, [c], 0, 255, 0)

                self.cont_imgs.append(cont_img)


if __name__ == '__main__':
    rospy.init_node('image_tracer', log_level=rospy.DEBUG)

    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

    image_pub = rospy.Publisher('/image_tracer/image_result', Image, queue_size=1)  # register result image publisher

    rgb_pub = rospy.Publisher('/sensor/rgb_led', Led, queue_size=1)

    running_srv = rospy.Service('/image_tracer/set_running', SetBool, set_running)
    heartbeat_srv = rospy.Service('/image_tracer/heartbeat', SetBool, heartbeat_srv_cb)

    debug = False
    if debug:
        rospy.sleep(0.2)
        start_running()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        cv2.destroyAllWindows()
