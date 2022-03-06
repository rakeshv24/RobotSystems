#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import math
import time
import copy
import rospy
import threading
import numpy as np
from threading import Timer

from std_msgs.msg import *
from std_srvs.srv import *
from sensor_msgs.msg import Image

from sensor.msg import Led
from warehouse.msg import Grasp
from object_pallezting.srv import *
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from kinematics import ik_transform

from armpi_fpv import PID
from armpi_fpv import Misc
from armpi_fpv import apriltag
from armpi_fpv import bus_servo_control


d_tag_map = 0

tag_z_min = 0.01
tag_z_max = 0.015

d_color_map = 30

color_z_min = 0.01
color_z_max = 0.015
d_color_y = 20
color_y_adjust = 400

center_x = 340

__target_data = (('red', 'green', 'blue'), ('tag1', 'tag2', 'tag3'))

__isRunning = False

lock = threading.RLock()
ik = ik_transform.ArmIK()

mask1 = cv2.imread('/home/ubuntu/armpi_fpv/src/object_sorting/scripts/mask1.jpg', 0)
mask2 = cv2.imread('/home/ubuntu/armpi_fpv/src/object_sorting/scripts/mask2.jpg', 0)
rows, cols = mask1.shape

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

################################################
# x, y, z(m)
place_position = {'red': [-0.13, -0, 0.01],
                  'green': [-0.13, -0, 0.01],
                  'blue': [-0.13, -0, 0.01],
                  'tag1': [0.13, -0, 0.01],
                  'tag2': [0.13, -0, 0.01],
                  'tag3': [0.13, -0, 0.01]}
################################################


def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 100:
                area_max_contour = c

    return area_max_contour, contour_area_max


def initMove(delay=True):
    with lock:
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 75), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))
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


x_dis = 500
Y_DIS = 0
y_dis = Y_DIS
last_x_dis = x_dis
last_x_dis = y_dis
x_pid = PID.PID(P=0.01, I=0.001, D=0)
y_pid = PID.PID(P=0.00001, I=0, D=0)

tag_x_dis = 500
tag_y_dis = 0
tag_x_pid = PID.PID(P=0.01, I=0.001, D=0)
tag_y_pid = PID.PID(P=0.02, I=0, D=0)

stop_state = 0
move_state = 1
adjust = False
approach = False
rotation_angle = 0
start_move = False
adjust_error = False
last_X, last_Y = 0, 0
box_rotation_angle = 0
last_box_rotation_angle = 0
tag1 = ['tag1', -1, -1, -1, 0]
tag2 = ['tag2', -1, -1, -1, 0]
tag3 = ['tag3', -1, -1, -1, 0]
current_tag = ['tag1', 'tag2', 'tag3']
detect_color = ('red', 'green', 'blue')

count = 0
count2 = 0
count3 = 0
count_d = 0
count_timeout = 0
count_tag_timeout = 0
count_adjust_timeout = 0


def reset():
    global X, Y
    global adjust
    global approach
    global move_state
    global start_move
    global current_tag
    global detect_color
    global x_dis, y_dis
    global adjust_error
    global last_X, last_Y
    global tag1, tag2, tag3
    global box_rotation_angle
    global tag_x_dis, tag_y_dis
    global last_x_dis, last_y_dis
    global rotation_angle, last_box_rotation_angle
    global count, count2, count3, count_timeout, count_adjust_timeout, count_d, count_tag_timeout

    with lock:
        X = 0
        Y = 0

        x_dis = 500
        y_dis = Y_DIS
        tag_x_dis = 500
        tag_y_dis = 0
        x_pid.clear()
        y_pid.clear()
        tag_x_pid.clear()
        tag_y_pid.clear()
        last_x_dis = x_dis
        last_y_dis = y_dis

        adjust = False
        approach = False
        start_move = False
        adjust_error = False

        move_state = 1
        turn_off_rgb()
        rotation_angle = 0
        box_rotation_angle = 0
        last_box_rotation_angle = 0

        count = 0
        count2 = 0
        count3 = 0
        count_d = 0
        count_timeout = 0
        count_tag_timeout = 0
        count_adjust_timeout = 0

        tag1 = ['tag1', -1, -1, -1, 0]
        tag2 = ['tag2', -1, -1, -1, 0]
        tag3 = ['tag3', -1, -1, -1, 0]
        current_tag = ['tag1', 'tag2', 'tag3']
        detect_color = ('red', 'green', 'blue')


color_range = None


def init():
    global stop_state
    global color_range

    rospy.loginfo("object pallezting Init")
    color_range = rospy.get_param('/lab_config_manager/color_range_list', {})  # get lab range from ros param server
    initMove()
    stop_state = 0
    reset_position()
    reset()


place_z_color = copy.deepcopy(place_position['red'][2])
place_z_tag = copy.deepcopy(place_position['red'][2])


def reset_position():
    global place_z_color
    global place_z_tag

    place_z_color = copy.deepcopy(place_position['red'][2])
    place_z_tag = copy.deepcopy(place_position['red'][2])


y_d = 0
roll_angle = 0
gripper_rotation = 0
# Half the diagonal length of the wooden block
square_diagonal = 0.03 * math.sin(math.pi / 4)
F = 1000 / 240.0


def pick(grasps, have_adjust=False):
    global roll_angle, last_x_dis
    global adjust, x_dis, y_dis, tag_x_dis, tag_y_dis, adjust_error, gripper_rotation

    position = grasps.grasp_pos.position
    rotation = grasps.grasp_pos.rotation
    approach = grasps.grasp_approach
    retreat = grasps.grasp_retreat

    # Calculate whether the target position can be reached, if not, return
    target1 = ik.setPitchRanges((position.x + approach.x, position.y + approach.y,
                                position.z + approach.z), rotation.r, -180, 0)
    target2 = ik.setPitchRanges((position.x, position.y, position.z), rotation.r, -180, 0)
    target3 = ik.setPitchRanges((position.x, position.y, position.z + grasps.up), rotation.r, -180, 0)
    target4 = ik.setPitchRanges((position.x + retreat.x, position.y + retreat.y,
                                position.z + retreat.z), rotation.r, -180, 0)

    if not __isRunning:
        return False
    if target1 and target2 and target3 and target4:
        if not have_adjust:
            servo_data = target1[1]
            bus_servo_control.set_servos(
                joints_pub, 1800, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
            rospy.sleep(2)
            if not __isRunning:
                return False

            # step 3: move to the target point
            servo_data = target2[1]
            bus_servo_control.set_servos(
                joints_pub, 1500, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
            rospy.sleep(2)
            if not __isRunning:
                servo_data = target4[1]
                bus_servo_control.set_servos(
                    joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                rospy.sleep(1)
                return False

            roll_angle = target2[2]
            gripper_rotation = box_rotation_angle

            x_dis = tag_x_dis = last_x_dis = target2[1]['servo6']
            y_dis = tag_y_dis = 0

            if state == 'color':
                # step 4: fine-tune the position
                if not adjust:
                    adjust = True
                    return True
            else:
                return True
        else:
            # step 5: align
            bus_servo_control.set_servos(joints_pub, 500, ((2, 500 + int(F * gripper_rotation)), ))
            rospy.sleep(0.8)
            if not __isRunning:
                servo_data = target4[1]
                bus_servo_control.set_servos(
                    joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                rospy.sleep(1)
                return False

            # step 6: clamp
            bus_servo_control.set_servos(joints_pub, 500, ((1, grasps.grasp_posture), ))
            rospy.sleep(0.8)
            if not __isRunning:
                bus_servo_control.set_servos(joints_pub, 500, ((1, grasps.pre_grasp_posture), ))
                rospy.sleep(0.5)
                servo_data = target4[1]
                bus_servo_control.set_servos(
                    joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                rospy.sleep(1)
                return False

            # step 7: lift the object
            if grasps.up != 0:
                servo_data = target3[1]
                bus_servo_control.set_servos(
                    joints_pub, 500, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                rospy.sleep(0.6)
            if not __isRunning:
                bus_servo_control.set_servos(joints_pub, 500, ((1, grasps.pre_grasp_posture), ))
                rospy.sleep(0.5)
                servo_data = target4[1]
                bus_servo_control.set_servos(
                    joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                rospy.sleep(1)
                return False

            # step 8: move to the evacuation point
            servo_data = target4[1]
            if servo_data != target3[1]:
                bus_servo_control.set_servos(
                    joints_pub, 1000, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                rospy.sleep(1)
                if not __isRunning:
                    bus_servo_control.set_servos(joints_pub, 500, ((1, grasps.pre_grasp_posture), ))
                    rospy.sleep(0.5)
                    return False

            # step 9: move to the stable point
            servo_data = target1[1]
            bus_servo_control.set_servos(joints_pub, 1500, ((2, 500), (3, 80), (4, 825), (5, 625)))
            rospy.sleep(1.5)
            if not __isRunning:
                bus_servo_control.set_servos(joints_pub, 500, ((1, grasps.pre_grasp_posture), ))
                rospy.sleep(0.5)
                return False

            return target2[2]
    else:
        rospy.loginfo('pick failed')
        return False


def place(places):
    position = places.grasp_pos.position
    rotation = places.grasp_pos.rotation
    approach = places.grasp_approach
    retreat = places.grasp_retreat

    target1 = ik.setPitchRanges((position.x + approach.x, position.y + approach.y,
                                position.z + approach.z), rotation.r, -180, 0)
    target2 = ik.setPitchRanges((position.x, position.y, position.z), rotation.r, -180, 0)
    target3 = ik.setPitchRanges((position.x, position.y, position.z + places.up), rotation.r, -180, 0)
    target4 = ik.setPitchRanges((position.x + retreat.x, position.y + retreat.y,
                                position.z + retreat.z), rotation.r, -180, 0)

    if not __isRunning:
        return False
    if target1 and target2 and target3 and target4:
        # step 1: turn the gimbal to the target direction
        servo_data = target1[1]
        bus_servo_control.set_servos(joints_pub, 1000, ((1, places.pre_grasp_posture), (2, int(
            F * rotation.y)), (3, 150), (4, 825), (5, 625), (6, servo_data['servo6'])))
        rospy.sleep(1)
        if not __isRunning:
            bus_servo_control.set_servos(joints_pub, 500, ((1, places.grasp_posture), ))
            rospy.sleep(0.5)
            return False

        # step 2: move to the close point
        bus_servo_control.set_servos(joints_pub, 1500, ((
            3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
        rospy.sleep(1.6)
        if not __isRunning:
            bus_servo_control.set_servos(joints_pub, 500, ((1, places.grasp_posture), ))
            rospy.sleep(0.5)
            return False

        # step 3: move to the target point
        servo_data = target2[1]
        bus_servo_control.set_servos(joints_pub, 1000, ((
            3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
        rospy.sleep(1.5)
        if not __isRunning:
            bus_servo_control.set_servos(joints_pub, 500, ((1, places.grasp_posture), ))
            rospy.sleep(0.5)
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1000, ((
                1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(1)
            return False

        # step 4: lift
        if places.up != 0:
            servo_data = target3[1]
            bus_servo_control.set_servos(joints_pub, 800, ((
                3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(0.8)
        if not __isRunning:
            bus_servo_control.set_servos(joints_pub, 500, ((1, places.grasp_posture), ))
            rospy.sleep(0.5)
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1000, ((
                1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(1)
            return False

        # step 5: place
        bus_servo_control.set_servos(joints_pub, 400, ((1, places.pre_grasp_posture - 40), ))
        rospy.sleep(0.8)
        bus_servo_control.set_servos(joints_pub, 500, ((1, places.grasp_posture), ))
        rospy.sleep(1)
        if not __isRunning:
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1000, ((
                1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(1)
            return False

        # step 6: move to the evacuation point
        servo_data = target4[1]
        if servo_data != target3[1]:
            bus_servo_control.set_servos(joints_pub, 600, ((
                3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(0.6)
            if not __isRunning:
                return False

        # step 7: move to the stable point
        servo_data = target1[1]
        bus_servo_control.set_servos(joints_pub, 1000, ((2, 500), (3, 80),
                                     (4, 825), (5, 625), (6, servo_data['servo6'])))
        rospy.sleep(1)
        if not __isRunning:
            return False

        return True
    else:
        rospy.loginfo('place failed')
        return False


grasps = Grasp()


def move():
    global y_d
    global grasps
    global approach
    global x_adjust
    global move_state
    global place_z_tag
    global place_z_color

    dz = 0.031
    init_z = copy.deepcopy(place_position['red'][2])
    while True:
        if __isRunning:
            if approach:
                position = None
                approach = True

                if not adjust and move_state == 1:
                    grasps.grasp_pos.position.x = X
                    grasps.grasp_pos.position.y = Y
                    if state == 'color':
                        grasps.grasp_pos.position.z = Misc.map(Y - 0.15, 0, 0.15, color_z_min, color_z_max)
                    else:
                        grasps.grasp_pos.position.z = Misc.map(Y - 0.12, 0, 0.15, tag_z_min, tag_z_max)
                    grasps.grasp_pos.rotation.r = -175

                    grasps.up = 0

                    grasps.grasp_approach.y = -0.01
                    grasps.grasp_approach.z = 0.02

                    grasps.grasp_retreat.z = 0.04

                    grasps.grasp_posture = 450
                    grasps.pre_grasp_posture = 75
                    buzzer_pub.publish(0.1)
                    result = pick(grasps)
                    if result:
                        move_state = 2
                    else:
                        reset()
                        initMove(delay=False)
                elif not adjust and move_state == 2:
                    result = pick(grasps, have_adjust=True)
                    if not result:
                        reset()
                        initMove(delay=False)
                    move_state = 3
                elif not adjust and move_state == 3:
                    if result:
                        if state == 'color':
                            position = copy.deepcopy(place_position[pick_color])
                            position[2] = place_z_color
                            place_z_color += dz
                            if position[2] == init_z:
                                position[0] -= 0
                            if round(position[2], 4) >= round(init_z + 2 * dz, 4):
                                place_z_color = init_z
                        elif state == 'tag':
                            position = copy.deepcopy(place_position[current_tag])
                            position[2] = place_z_tag
                            place_z_tag += dz
                            if position[2] == init_z:
                                position[0] += 0
                            if round(position[2], 4) >= round(init_z + 2 * dz, 4):
                                place_z_tag = init_z

                        if position[0] < 0:
                            yaw = int(120 - (90 + math.degrees(math.atan2(position[0], position[1]))))
                        else:
                            yaw = int(120 + (90 - math.degrees(math.atan2(position[0], position[1]))))

                        places = Grasp()
                        places.grasp_pos.position.x = position[0]
                        places.grasp_pos.position.y = position[1]
                        places.grasp_pos.position.z = position[2]
                        if result > -160 and position[2] == init_z:
                            places.grasp_pos.position.z -= 0
                        places.grasp_pos.rotation.r = result
                        places.grasp_pos.rotation.y = yaw

                        places.up = 0.0
                        places.grasp_approach.z = 0.02
                        places.grasp_retreat.z = 0.04

                        places.grasp_posture = 75
                        places.pre_grasp_posture = 450
                        place(places)

                    initMove(delay=False)
                    reset()
                else:
                    rospy.sleep(0.001)
            else:
                rospy.sleep(0.01)
        else:
            rospy.sleep(0.01)


th = threading.Thread(target=move)
th.setDaemon(True)
th.start()


detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())


def apriltagDetect(img):
    global tag1, tag2, tag3

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray, return_image=False)

    tag1 = ['tag1', -1, -1, -1, 0]
    tag2 = ['tag2', -1, -1, -1, 0]
    tag3 = ['tag3', -1, -1, -1, 0]
    if len(detections) != 0:
        for i, detection in enumerate(detections):
            corners = np.rint(detection.corners)
            cv2.drawContours(img, [np.array(corners, np.int)], -1, (0, 255, 255), 2)

            tag_family = str(detection.tag_family, encoding='utf-8')
            tag_id = int(detection.tag_id)

            object_center_x, object_center_y = int(detection.center[0]), int(detection.center[1])
            object_angle = int(
                math.degrees(
                    math.atan2(
                        corners[0][1] -
                        corners[1][1],
                        corners[0][0] -
                        corners[1][0])))

            cv2.putText(img, str(tag_id), (object_center_x - 10, object_center_y + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 255, 255], 2)

            if tag_id == 1:
                tag1 = ['tag1', object_center_x, object_center_y, object_angle]
            elif tag_id == 2:
                tag2 = ['tag2', object_center_x, object_center_y, object_angle]
            elif tag_id == 3:
                tag3 = ['tag3', object_center_x, object_center_y, object_angle]


def getROI(rotation_angle):
    rotate1 = cv2.getRotationMatrix2D((rows * 0.5, cols * 0.5), int(rotation_angle), 1)
    rotate_rotate1 = cv2.warpAffine(mask2, rotate1, (cols, rows))
    mask_and = cv2.bitwise_and(rotate_rotate1, mask1)
    rotate2 = cv2.getRotationMatrix2D((rows * 0.5, cols * 0.5), int(-rotation_angle), 1)
    rotate_rotate2 = cv2.warpAffine(mask_and, rotate2, (cols, rows))
    frame_resize = cv2.resize(rotate_rotate2, (710, 710), interpolation=cv2.INTER_NEAREST)
    roi = frame_resize[40:280, 184:504]

    return roi


size = (320, 240)

last_x = 0
last_y = 0
state = None
x_adjust = 0
pick_color = ''


def color_sort(img, target):
    global X, Y
    global count
    global state
    global adjust
    global approach
    global x_adjust
    global pick_color
    global current_tag
    global adjust_error
    global x_dis, y_dis
    global detect_color
    global count_timeout
    global rotation_angle
    global box_rotation_angle
    global last_x_dis, last_y_dis
    global last_box_rotation_angle
    global last_x, last_y, count_d, start_move

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gray = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2GRAY)
    frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB)

    max_area = 0
    color_area_max = None
    areaMaxContour_max = 0

    roi = getROI(rotation_angle)

    for i in color_range:
        if i in target:
            if i in detect_color:
                target_color_range = color_range[i]
                frame_mask1 = cv2.inRange(
                    frame_lab, tuple(
                        target_color_range['min']), tuple(
                        target_color_range['max']))
                #mask = cv2.bitwise_and(roi, frame_gray)
                frame_mask2 = cv2.bitwise_and(roi, frame_mask1)
                eroded = cv2.erode(frame_mask2, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                #cv2.imshow('mask', dilated)
                # cv2.waitKey(1)
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = getAreaMaxContour(contours)
                if areaMaxContour is not None:
                    if area_max > max_area and area_max > 100:
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
    if max_area > 100:
        rect = cv2.minAreaRect(areaMaxContour_max)
        box_rotation_angle = rect[2]
        if box_rotation_angle > 45:
            box_rotation_angle = box_rotation_angle - 90

        box = np.int0(cv2.boxPoints(rect))
        for j in range(4):
            box[j, 0] = int(Misc.map(box[j, 0], 0, size[0], 0, img_w))
            box[j, 1] = int(Misc.map(box[j, 1], 0, size[1], 0, img_h))

        cv2.drawContours(img, [box], -1, range_rgb[color_area_max], 2)

        centerX = int(Misc.map(((areaMaxContour_max[areaMaxContour_max[:, :, 0].argmin()][0])[
                      0] + (areaMaxContour_max[areaMaxContour_max[:, :, 0].argmax()][0])[0]) / 2, 0, size[0], 0, img_w))
        centerY = int(Misc.map((areaMaxContour_max[areaMaxContour_max[:, :, 1].argmin()][0])[1], 0, size[1], 0, img_h))

        #cv2.circle(img, (int(centerX), int(centerY)), 5, range_rgb[color_area_max], -1)
        if abs(centerX - last_x) <= 5 and abs(centerY - last_y) <= 5 and not start_move:
            count_d += 1
            if count_d > 5:
                count_d = 0
                start_move = True

                led = Led()
                led.index = 0
                led.rgb.r = range_rgb[color_area_max][2]
                led.rgb.g = range_rgb[color_area_max][1]
                led.rgb.b = range_rgb[color_area_max][0]
                rgb_pub.publish(led)
                led.index = 1
                rgb_pub.publish(led)
                rospy.sleep(0.1)

                if 298 + d_color_map < centerY <= 424 + d_color_map:
                    Y = Misc.map(centerY, 298 + d_color_map, 424 + d_color_map, 0.12, 0.12 - 0.04)
                elif 198 + d_color_map < centerY <= 298 + d_color_map:
                    Y = Misc.map(centerY, 198 + d_color_map, 298 + d_color_map, 0.12 + 0.04, 0.12)
                elif 114 + d_color_map < centerY <= 198 + d_color_map:
                    Y = Misc.map(centerY, 114 + d_color_map, 198 + d_color_map, 0.12 + 0.08, 0.12 + 0.04)
                elif 50 + d_color_map < centerY <= 114 + d_color_map:
                    Y = Misc.map(centerY, 50 + d_color_map, 114 + d_color_map, 0.12 + 0.12, 0.12 + 0.08)
                elif 0 + d_color_map < centerY <= 50 + d_color_map:
                    Y = Misc.map(centerY, 0 + d_color_map, 50 + d_color_map, 0.12 + 0.16, 0.12 + 0.12)
                else:
                    Y = 1
        else:
            count_d = 0

        last_x = centerX
        last_y = centerY
        if (not approach or adjust) and start_move:
            detect_color = (color_area_max, )
            x_pid.SetPoint = center_x
            x_pid.update(centerX)
            dx = x_pid.output
            x_dis += dx

            x_dis = 0 if x_dis < 0 else x_dis
            x_dis = 1000 if x_dis > 1000 else x_dis

            if adjust:
                y_pid.SetPoint = color_y_adjust
                start_move = True

                centerY += abs(Misc.map(70 * math.sin(math.pi / 4) / 2, 0, size[0], 0, img_w) * math.sin(
                    math.radians(abs(gripper_rotation) + 45))) + 65 * math.sin(math.radians(abs(roll_angle)))
                if Y < 0.12 + 0.04:
                    centerY += d_color_y
                if 0 < centerY - color_y_adjust <= 5:
                    centerY = color_y_adjust
                y_pid.update(centerY)

                dy = y_pid.output
                y_dis += dy
                y_dis = 0.1 if y_dis > 0.1 else y_dis
                y_dis = -0.1 if y_dis < -0.1 else y_dis
            else:
                dy = 0
            if abs(dx) < 0.1 and abs(dy) < 0.0001 and (abs(last_box_rotation_angle -
                                                           rect[2]) <= 10 or abs(last_box_rotation_angle - rect[2] >= 80)):
                count += 1
                if (adjust and count > 10) or (not adjust and count >= 10):
                    count = 0
                    if adjust:
                        adjust = False
                    else:
                        rotation_angle = 240 * (x_dis - 500) / 1000.0
                        X = round(-Y * math.tan(math.radians(rotation_angle)), 4)
                        state = 'color'
                        pick_color = detect_color[0]
                        adjust_error = False
                        approach = True
            else:
                count = 0

            if adjust and (abs(last_x_dis - x_dis) >= 2 or abs(last_y_dis - y_dis) > 0.002):
                position = grasps.grasp_pos.position
                rotation = grasps.grasp_pos.rotation
                target = ik.setPitchRanges((position.x, position.y + y_dis, position.z), rotation.r, -180, 0)
                if target:
                    servo_data = target[1]
                    bus_servo_control.set_servos(
                        joints_pub, 100, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, int(x_dis))))
                    rospy.sleep(0.1)
                    last_x_dis = x_dis
                    last_y_dis = y_dis
                else:
                    bus_servo_control.set_servos(joints_pub, 20, ((6, int(x_dis)), ))
            else:
                bus_servo_control.set_servos(joints_pub, 20, ((6, int(x_dis)), ))

            last_box_rotation_angle = rect[2]
    else:
        count_timeout += 1
        if count_timeout > 20:
            adjust_error = True
            count_timeout = 0
            current_tag = ['tag1', 'tag2', 'tag3']
            detect_color = __target_data[0]
    return img


d_map = 0.015
tag_map = [425, 384, 346, 310, 272, 239, 208, 177, 153, 129, 106, 86, 68, 51]


def tag_sort(img, target):
    global X, Y
    global state
    global count2
    global count3
    global adjust
    global approach
    global start_move
    global current_tag
    global adjust_error
    global last_X, last_Y
    global box_rotation_angle
    global tag_x_dis, tag_y_dis

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    centerX = target[1]
    centerY = target[2]

    box_rotation_angle = abs(target[3])
    if box_rotation_angle > 90:
        box_rotation_angle -= 90
    if box_rotation_angle > 45:
        box_rotation_angle = box_rotation_angle - 90
    if target[3] < 0:
        box_rotation_angle = -box_rotation_angle

    distance = math.sqrt(pow(centerX - last_X, 2) + pow(centerY - last_Y, 2))
    if distance < 5 and not start_move:
        count2 += 1
        if count2 > 20:
            count2 = 0
            start_move = True
    else:
        count2 = 0

    if (not approach or adjust) and start_move:
        tag_x_pid.SetPoint = center_x
        tag_x_pid.update(centerX)
        dx = tag_x_pid.output
        tag_x_dis += dx
        tag_x_dis = 0 if tag_x_dis < 0 else tag_x_dis
        tag_x_dis = 1000 if tag_x_dis > 1000 else tag_x_dis

        if abs(centerX - last_X) <= 1 and X != -1:
            count3 += 1
            rospy.sleep(0.01)
            if count3 > 30:
                count3 = 0
                if adjust:
                    adjust = False
                else:
                    current_tag = target[0]
                    if tag_map[1] + d_tag_map < centerY <= tag_map[0] + d_tag_map:
                        Y = Misc.map(
                            centerY,
                            tag_map[1] + d_tag_map,
                            tag_map[0] + d_tag_map,
                            0.12 + d_map,
                            0.12) - 0.005
                    elif tag_map[2] + d_tag_map < centerY <= tag_map[1] + d_tag_map:
                        Y = Misc.map(
                            centerY,
                            tag_map[2] + d_tag_map,
                            tag_map[1] + d_tag_map,
                            0.12 + 2 * d_map,
                            0.12 + d_map)
                    elif tag_map[3] + d_tag_map < centerY <= tag_map[2] + d_tag_map:
                        Y = Misc.map(
                            centerY,
                            tag_map[3] + d_tag_map,
                            tag_map[2] + d_tag_map,
                            0.12 + 3 * d_map,
                            0.12 + 2 * d_map)
                    elif tag_map[4] + d_tag_map < centerY <= tag_map[3] + d_tag_map:
                        Y = Misc.map(
                            centerY,
                            tag_map[4] + d_tag_map,
                            tag_map[3] + d_tag_map,
                            0.12 + 4 * d_map,
                            0.12 + 3 * d_map)
                    elif tag_map[5] + d_tag_map < centerY <= tag_map[4] + d_tag_map:
                        Y = Misc.map(
                            centerY,
                            tag_map[5] + d_tag_map,
                            tag_map[4] + d_tag_map,
                            0.12 + 5 * d_map,
                            0.12 + 4 * d_map)
                    elif tag_map[6] + d_tag_map < centerY <= tag_map[5] + d_tag_map:
                        Y = Misc.map(
                            centerY,
                            tag_map[6] + d_tag_map,
                            tag_map[5] + d_tag_map,
                            0.12 + 6 * d_map,
                            0.12 + 5 * d_map)
                    elif tag_map[7] + d_tag_map < centerY <= tag_map[6] + d_tag_map:
                        Y = Misc.map(
                            centerY,
                            tag_map[7] + d_tag_map,
                            tag_map[6] + d_tag_map,
                            0.12 + 7 * d_map,
                            0.12 + 6 * d_map)
                    elif tag_map[8] + d_tag_map < centerY <= tag_map[7] + d_tag_map:
                        Y = Misc.map(
                            centerY,
                            tag_map[8] + d_tag_map,
                            tag_map[7] + d_tag_map,
                            0.12 + 8 * d_map,
                            0.12 + 7 * d_map)
                    elif tag_map[9] + d_tag_map < centerY <= tag_map[8] + d_tag_map:
                        Y = Misc.map(
                            centerY,
                            tag_map[9] + d_tag_map,
                            tag_map[8] + d_tag_map,
                            0.12 + 9 * d_map,
                            0.12 + 8 * d_map)
                    elif tag_map[10] + d_tag_map < centerY <= tag_map[9] + d_tag_map:
                        Y = Misc.map(
                            centerY,
                            tag_map[10] + d_tag_map,
                            tag_map[9] + d_tag_map,
                            0.12 + 10 * d_map,
                            0.12 + 9 * d_map)
                    elif tag_map[11] + d_tag_map < centerY <= tag_map[10] + d_tag_map:
                        Y = Misc.map(
                            centerY,
                            tag_map[11] + d_tag_map,
                            tag_map[10] + d_tag_map,
                            0.12 + 11 * d_map,
                            0.12 + 10 * d_map)
                    elif tag_map[12] + d_tag_map < centerY <= tag_map[11] + d_tag_map:
                        Y = Misc.map(
                            centerY,
                            tag_map[12] + d_tag_map,
                            tag_map[11] + d_tag_map,
                            0.12 + 12 * d_map,
                            0.12 + 11 * d_map)
                    elif tag_map[13] + d_tag_map < centerY <= tag_map[12] + d_tag_map:
                        Y = Misc.map(
                            centerY,
                            tag_map[13] + d_tag_map,
                            tag_map[12] + d_tag_map,
                            0.12 + 13 * d_map,
                            0.12 + 12 * d_map)
                    else:
                        Y = 1

                    X = round(-Y * math.tan(math.radians(rotation_angle)), 4)
                    state = 'tag'
                    approach = True
                    adjust_error = False
                    adjust = False
        else:
            count3 = 0

        bus_servo_control.set_servos(joints_pub, 20, ((6, int(tag_x_dis)), ))
    last_X, last_Y = centerX, centerY

    return img


def run(img):
    global current_tag
    global adjust_error
    global count_tag_timeout
    global count_adjust_timeout

    if len(__target_data[1]) != 0:
        apriltagDetect(img)

    if 'tag1' in __target_data[1] and 'tag1' in current_tag:
        if tag1[1] != -1:
            count_adjust_timeout = 0
            img = tag_sort(img, tag1)
        else:
            if adjust:
                count_adjust_timeout += 1
                if count_adjust_timeout > 50:
                    count_adjust_timeout = 0
                    adjust_error = True
            else:
                count_tag_timeout += 1
                if count_tag_timeout > 3:
                    count_tag_timeout = 0
                    if current_tag != 'tag1':
                        current_tag.remove('tag1')
    elif 'tag2' in __target_data[1] and 'tag2' in current_tag:
        if tag2[1] != -1:
            count_adjust_timeout = 0
            img = tag_sort(img, tag2)
        else:
            if adjust:
                count_adjust_timeout += 1
                if count_adjust_timeout > 50:
                    count_adjust_timeout = 0
                    adjust_error = True
            else:
                count_tag_timeout += 1
                if count_tag_timeout > 3:
                    count_tag_timeout = 0
                    if current_tag != 'tag2':
                        current_tag.remove('tag2')
    elif 'tag3' in __target_data[1] and 'tag3' in current_tag:
        if tag3[1] != -1:
            count_adjust_timeout = 0
            img = tag_sort(img, tag3)
        else:
            if adjust:
                count_adjust_timeout += 1
                if count_adjust_timeout > 50:
                    count_adjust_timeout = 0
                    adjust_error = True
            else:
                count_tag_timeout += 1
                if count_tag_timeout > 3:
                    count_tag_timeout = 0
                    if current_tag != 'tag3':
                        current_tag.remove('tag3')
    elif len(__target_data[0]) != 0:
        img = color_sort(img, __target_data[0])
    else:
        current_tag = ['tag1', 'tag2', 'tag3']

    img_h, img_w = img.shape[:2]

    cv2.line(img, (int(img_w / 2 - 10), int(img_h / 2)), (int(img_w / 2 + 10), int(img_h / 2)), (0, 255, 255), 2)
    cv2.line(img, (int(img_w / 2), int(img_h / 2 - 10)), (int(img_w / 2), int(img_h / 2 + 10)), (0, 255, 255), 2)

    return img


def image_callback(ros_image):
    global lock
    global stop_state

    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    frame = cv2_img.copy()
    frame_result = frame

    with lock:
        if __isRunning:
            frame_result = run(frame)
        else:
            if stop_state:
                stop_state = 0
                initMove(delay=False)
    rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tostring()
    ros_image.data = rgb_image
    image_pub.publish(ros_image)


org_image_sub_ed = False


def enter_func(msg):
    global lock
    global image_sub
    global __isRunning
    global org_image_sub_ed

    rospy.loginfo("enter object pallezting")
    with lock:
        init()
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

    rospy.loginfo("exit object pallezting")
    with lock:
        __isRunning = False
        try:
            if org_image_sub_ed:
                org_image_sub_ed = False
                if heartbeat_timer is not None:
                    heartbeat_timer.cancel()
                image_sub.unregister()
        except BaseException:
            pass

    return [True, 'exit']


def start_running():
    global lock
    global __isRunning

    rospy.loginfo("start running object pallezting")
    with lock:
        __isRunning = True


def stop_running():
    global lock
    global stop_state
    global __isRunning

    rospy.loginfo("stop running object pallezting")
    with lock:
        __isRunning = False
        if (not approach and start_move) or adjust:
            stop_state = 1
        reset_position()
        reset()


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
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/object_pallezting/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp


if __name__ == '__main__':
    rospy.init_node('object_pallezting', log_level=rospy.DEBUG)

    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

    image_pub = rospy.Publisher('/object_pallezting/image_result', Image,
                                queue_size=1)  # register result image publisher

    enter_srv = rospy.Service('/object_pallezting/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/object_pallezting/exit', Trigger, exit_func)
    running_srv = rospy.Service('/object_pallezting/set_running', SetBool, set_running)
    heartbeat_srv = rospy.Service('/object_pallezting/heartbeat', SetBool, heartbeat_srv_cb)

    buzzer_pub = rospy.Publisher('/sensor/buzzer', Float32, queue_size=1)
    rgb_pub = rospy.Publisher('/sensor/rgb_led', Led, queue_size=1)

    config = rospy.get_param('config', {})
    if config != {}:
        d_tag_map = config['d_tag_map']

        tag_z_min = config['tag_z_min']
        tag_z_max = config['tag_z_max']

        d_color_map = config['d_color_map']

        color_z_min = config['color_z_min']
        color_z_max = config['color_z_max']
        d_color_y = config['d_color_y']
        color_y_adjust = config['color_y_adjust']

        center_x = config['center_x']

    debug = False
    if debug:
        rospy.sleep(0.2)
        enter_func(1)
        start_running()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
