import sys
sys.path.append(r'/home/vader/picar-x/lib')
sys.path.append(r'/home/darth/workspace/picar-x/lib')
import time
import logging
from logdecorator import log_on_start, log_on_end, log_on_error
from picarx_improved import Picarx
from camera import Camera
import cv2
  

if __name__=="__main__":
    px = Picarx()
    camera = Camera()
    
    px.set_camera_servo1_angle(-15)

    for frame in camera.camera.capture_continuous(camera.rawCapture, format='bgr', use_video_port=True):
        px.forward(30)

        frame_array = frame.array

        lane_lines = camera.detect_lane(frame_array)
        lines_img = camera.display_lane_lines(frame_array, lane_lines)

        steering_angle = camera.get_steering_angle(frame_array, lane_lines)
        heading_img = camera.display_heading_line(lines_img, steering_angle)
        cv2.imshow('heading', heading_img)

        px.set_dir_servo_angle(-steering_angle)

        camera.rawCapture.truncate(0)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            camera.camera.close()
            px.stop()
            break