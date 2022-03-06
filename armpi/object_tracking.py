#!/usr/bin/python3
# coding=utf8

import sys
import cv2
import math
import rospy
import numpy as np
from threading import RLock, Timer

from std_srvs.srv import *
from sensor_msgs.msg import Image

from sensor.msg import Led
from object_tracking.srv import *
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from kinematics import ik_transform

from armpi_fpv import PID
from armpi_fpv import Misc
from armpi_fpv import bus_servo_control

ik = ik_transform.ArmIK()

lock = RLock()

size = (320, 240)
start_move = True
__target_color = ''
__isRunning = False
org_image_sub_ed = False

x_dis = 500
y_dis = 0.167
Z_DIS = 0.2
z_dis = Z_DIS
x_pid = PID.PID(P=0.06, I=0.005, D=0)
y_pid = PID.PID(P=0.00001, I=0, D=0)
z_pid = PID.PID(P=0.00003, I=0, D=0)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}


def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 10:
                area_max_contour = c

    return area_max_contour, contour_area_max


def initMove(delay=True):
    with lock:
        target = ik.setPitchRanges((0, y_dis, Z_DIS), -90, -92, -88)
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
    global x_dis, y_dis, z_dis
    global __target_color

    with lock:
        x_dis = 500
        y_dis = 0.167
        z_dis = Z_DIS
        x_pid.clear()
        y_pid.clear()
        z_pid.clear()
        turn_off_rgb()
        __target_color = ''


color_range = None


def init():
    global color_range

    rospy.loginfo("object tracking Init")
    color_range = rospy.get_param('/lab_config_manager/color_range_list', {})  # get lab range from ros param server
    initMove()
    reset()


def run(img):
    global start_move
    global x_dis, y_dis, z_dis
    global color_range
    global __target_color

    imgObj = Perception(img)

    # Drawing a plus in the middle of image frame
    imgObj.draw_center()

    # Resizing the image frame and getting the color conversion code
    imgObj.modify_frame_params()

    # Checking if the target color is in the color range list ('/lab_config_manager/color_range_list')
    if __target_color in color_range:
        # Getting the range of the target color
        target_color_range = color_range[__target_color]

        # Creating a mask for the image
        mask = imgObj.get_mask(target_color_range)

        # Removes the white noise of the image
        eroded_frame = imgObj.erode(mask)

        # Accentuates image features
        dilated_frame = imgObj.dilate(eroded_frame)

        # Finding the contours of the image frame
        contours = imgObj.get_contours(dilated_frame)

        # Getting the max area and the max contour area
        imgObj.update_max_cont_area(contours)

    if imgObj.max_area > 100:
        imgObj.get_circle_params()

        # If the radius is bigger than 100 pixels then we return the original image
        if imgObj.radius > 100:
            return img

        imgObj.draw_circle(__target_color)

        xPid = Motion(x_pid, x_dis)
        yPid = Motion(y_pid, y_dis)
        zPid = Motion(z_pid, z_dis)

        if start_move:
            xPid.pid.SetPoint = imgObj.img_width / 2.0
            xPid.update(imgObj.cX)
            xPid.mod_dist(xPid.pid.output)
            xPid.clamp_dist(200, 800)

            yPid.pid.SetPoint = 900
            if abs(imgObj.max_area - 900) < 50:
                imgObj.max_area = 900
            yPid.update(imgObj.max_area)
            yPid.mod_dist(yPid.pid.output)
            yPid.clamp_dist(0.12, 0.25)

            zPid.pid.SetPoint = imgObj.img_height / 2.0
            yPid.update(imgObj.cY)
            yPid.mod_dist(yPid.pid.output)
            yPid.clamp_dist(0.17, 0.22)

            target = ik.setPitchRanges((0, round(yPid.dis, 4), round(zPid.dis, 4)), -90, -85, -95)
            if target:
                servo_data = target[1]
                bus_servo_control.set_servos(joints_pub, 20, (
                    (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, xPid.dis)))
    return img


def image_callback(ros_image):
    global lock

    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    frame = cv2_img.copy()
    frame_result = frame
    with lock:
        if __isRunning:
            frame_result = run(frame)
    rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tostring()
    ros_image.data = rgb_image

    image_pub.publish(ros_image)


def enter_func(msg):
    global lock
    global image_sub
    global __isRunning
    global org_image_sub_ed

    rospy.loginfo("enter object tracking")
    init()
    with lock:
        if not org_image_sub_ed:
            org_image_sub_ed = True
            image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    return [True, 'enter']


heartbeat_timer = None


def exit_func(msg):
    global lock
    global image_sub
    global __isRunning
    global org_image_sub_ed

    rospy.loginfo("exit object tracking")
    with lock:
        __isRunning = False
        reset()
        try:
            if org_image_sub_ed:
                org_image_sub_ed = False
                heartbeat_timer.cancel()
                image_sub.unregister()
        except BaseException as e:
            rospy.loginfo('%s', e)

    return [True, 'exit']


def start_running():
    global lock
    global __isRunning

    rospy.loginfo("start running object tracking")
    with lock:
        __isRunning = True


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


def set_target(msg):
    global lock
    global __target_color

    rospy.loginfo("%s", msg)
    with lock:
        __target_color = msg.data
        led = Led()
        led.index = 0
        led.rgb.r = range_rgb[__target_color][2]
        led.rgb.g = range_rgb[__target_color][1]
        led.rgb.b = range_rgb[__target_color][0]
        rgb_pub.publish(led)
        led.index = 1
        rgb_pub.publish(led)
        rospy.sleep(0.1)

    return [True, 'set_target']


def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/object_tracking/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp


class Perception():
    def __init__(self, img):
        self.img = img
        self.img_copy = img.copy()
        self.img_height, self.img_width = img.shape[:2]

        self.max_area = 0
        self.max_area_contour = 0

    def draw_center(self):
        cv2.line(self.img,
                 (int(self.img_width / 2 - 10), int(self.img_height / 2)),
                 (int(self.img_width / 2 + 10), int(self.img_height / 2)),
                 (0, 255, 255), 2)

        cv2.line(self.img,
                 (int(self.img_width / 2), int(self.img_height / 2 - 10)),
                 (int(self.img_width / 2), int(self.img_height / 2 + 10)),
                 (0, 255, 255), 2)

    def modify_frame_params(self):
        self.frame = cv2.resize(self.img_copy, size, interpolation=cv2.INTER_NEAREST)
        self.frame_Lab = cv2.cvtColor(self.frame, cv2.COLOR_BGR2LAB)

    def get_mask(self, color_range):
        return cv2.inRange(self.frame_Lab, tuple(color_range['min']), tuple(color_range['max']))

    def erode(self, frame):
        return cv2.erode(frame, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

    def dilate(self, frame):
        return cv2.dilate(frame, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

    def get_contours(self, frame):
        return cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

    def update_max_cont_area(self, contours):
        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > self.max_area:
                self.max_area = contour_area_temp
                if contour_area_temp > 10:
                    self.max_area_contour = c

    def get_circle_params(self):
        (cX, cY), radius = cv2.minEnclosingCircle(self.max_area_contour)

        self.cX = int(Misc.map(cX, 0, size[0], 0, self.img_width))
        self.cY = int(Misc.map(cY, 0, size[1], 0, self.img_height))
        self.radius = int(Misc.map(radius, 0, size[0], 0, self.img_width))

    def draw_circle(self, color):
        cv2.circle(self.img, (int(self.cX), int(self.cY)), int(self.radius), range_rgb[color], 2)


class Motion():
    def __init__(self, pid, dis):
        self.pid = pid
        self.dis = dis

    def update(self, value):
        self.pid.update(value)

    def mod_dist(self, value):
        self.dis += value

    def clamp_dist(self, lb, ub):
        self.dis = lb if self.dis < lb else self.dis
        self.dis = ub if self.dis > ub else self.dis


if __name__ == '__main__':
    rospy.init_node('object_tracking', log_level=rospy.DEBUG)

    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

    image_pub = rospy.Publisher('/object_tracking/image_result', Image, queue_size=1)  # register result image publisher

    rgb_pub = rospy.Publisher('/sensor/rgb_led', Led, queue_size=1)

    enter_srv = rospy.Service('/object_tracking/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/object_tracking/exit', Trigger, exit_func)
    running_srv = rospy.Service('/object_tracking/set_running', SetBool, set_running)
    set_target_srv = rospy.Service('/object_tracking/set_target', SetTarget, set_target)
    heartbeat_srv = rospy.Service('/object_tracking/heartbeat', SetBool, heartbeat_srv_cb)

    debug = False
    if debug:
        rospy.sleep(0.2)
        enter_func(1)

        msg = SetTarget()
        msg.data = 'blue'

        set_target(msg)
        start_running()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        cv2.destroyAllWindows()
