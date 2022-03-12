#!/usr/bin/python3
# coding=utf8

import sys
import cv2
import time
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
__isRunning = False
org_image_sub_ed = False

x_start = 0.0
y_start = 0.1
z_start = 0.15


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
    global x_dis, y_dis, z_dis
    global __target_color

    with lock:
        turn_off_rgb()

def init():
    global color_range

    rospy.loginfo("object tracking Init")
    initMove()
    reset()
    
def run(img):
    global start_move
    global x_dis, y_dis, z_dis
    global lock, __isRunning

    imgObj = Perception(img)
    # imgObj.image_processing()
    # imgObj.feature_detection()
    # imgObj.draw_features()
    
    # points = np.array([[160, 120], [200, 120], [200, 160], [160, 160], [160, 120]])
    # points = np.array([[140, 100], [180, 100], [180, 140], [140, 140], [140, 100]])
    
    # SQUARE
    points = np.array([[100, 60], [220, 60], [220, 180], [100, 180], [100, 60]])
    
    # TRIANGLE
    points = np.array([[160, 60], [220, 180], [100, 180], [160, 60]])
    
    # CIRCLE
    theta = np.linspace(0, 2*np.pi, 10).reshape(-1, 1)
    r = 40
    cX = r * np.cos(theta)
    cY = r * np.sin(theta)
    points = np.hstack((cX, cY))
    
    x_move = 0.0
    x_min, x_max = -.02, .02
    z_min, z_max = .01, .05
    x_prev, z_prev = 0.0, 0.0
    
    if start_move:
        # for f in imgObj.features:
        for i in range(len(points)):
            x, z = points[i].ravel()
            
            # if i > 0:
            #     x_prev, z_prev = points[i-1].ravel()
            #     x_diff = x - x_prev
            #     z_diff = z - z_prev
                
            #     x = (size[0] / 2) + x_diff
            #     z = (size[1] / 2) + z_diff
            # else:
            #     x = x - (size[0] / 2)
            #     z = z - (size[1] / 2)
                
            x = x - (size[0] / 2)
            z = z - (size[1] / 2)
            
            print(f"x_im:{x}, z_im:{z}")
            
            x = np.sign(x) * (((abs(x) * (x_max - x_min)) / size[0]) + x_min)
            z = np.sign(z) * (((abs(z) * (z_max - z_min)) / size[1]) + z_min)  
            
            x_move = round(x + x_start, 3) 
            z_move = round(z + z_start, 3) 
            # x_move = round(x, 3) 
            # z_move = round(z, 3) 
            
            # x = int(Misc.map(x, 0, size[0], 0, imgObj.img_width))
            # z = int(Misc.map(z, 0, size[1], 0, imgObj.img_height))
                        
            print(f"x_rw:{x}, z_rw:{z}")
            print(f"x_prev:{x_prev}, z_prev:{z_prev}")
            print(f"x_move:{x_move}, z_move:{z_move}")
            target = ik.setPitchRanges((x_move, 0.10, z_move), -90, -100, -80)
            print(f"target: {target}\n")
            
            if target:
                servo_data = target[1]
                bus_servo_control.set_servos(joints_pub, 20, (
                    (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            
            # x_prev = x
            # z_prev = z
            
            time.sleep(2)
    
    with lock:
        __isRunning = False
    
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
        self.img_height, self.img_width = img.shape[:2]

        self.max_area = 0
        self.max_area_contour = 0

    def draw_features(self):
        for i in self.features:
            x, y = i.ravel()
            x = int(Misc.map(x, 0, size[0], 0, self.img_width))
            y = int(Misc.map(y, 0, size[1], 0, self.img_height))
            cv2.circle(self.img, (x, y), 4, 200, -1)

    def image_processing(self):
        self.frame = cv2.resize(self.img_copy, size, interpolation=cv2.INTER_NEAREST)
        self.gray_image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        
    def feature_detection(self):
        gray = np.float32(self.gray_image)
        self.features = cv2.goodFeaturesToTrack(gray, 10, 0.01, 10)
        self.features = np.int0(self.features).reshape(-1, 2)
        self.features = self.features[np.argsort(self.features[:, 0])]
        
    def erode(self, frame):
        return cv2.erode(frame, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

    def dilate(self, frame):
        return cv2.dilate(frame, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

    def get_contours(self, frame):
        return cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]


if __name__ == '__main__':
    rospy.init_node('image_tracer', log_level=rospy.DEBUG)

    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

    image_pub = rospy.Publisher('/image_tracer/image_result', Image, queue_size=1)  # register result image publisher

    rgb_pub = rospy.Publisher('/sensor/rgb_led', Led, queue_size=1)

    enter_srv = rospy.Service('/image_tracer/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/image_tracer/exit', Trigger, exit_func)
    running_srv = rospy.Service('/image_tracer/set_running', SetBool, set_running)
    heartbeat_srv = rospy.Service('/image_tracer/heartbeat', SetBool, heartbeat_srv_cb)

    debug = False
    if debug:
        rospy.sleep(0.2)
        enter_func(1)
        start_running()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        cv2.destroyAllWindows()
