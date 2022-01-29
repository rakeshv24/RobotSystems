import sys
sys.path.append(r'/home/vader/picar-x/lib')
sys.path.append(r'/home/darth/workspace/picar-x/lib')
import time
import logging
from logdecorator import log_on_start, log_on_end, log_on_error
from picarx_improved import Picarx
from camera import Camera
from picamera.array import PiRGBArray
import cv2
  

if __name__=="__main__":
    px = Picarx()
    camera = Camera()
    px.set_camera_servo2_angle(-15)
    scaling_factor = 0.25

    while True:
        frame = camera.raw_capture()
        px.forward(30)

        frame_array = frame.array

        lane_lines = camera.detect_lane(frame_array)
        lines_img = camera.display_lane_lines(frame_array, lane_lines)

        steering_angle = camera.get_steering_angle(frame_array, lane_lines)
        heading_img = camera.display_heading_line(lines_img, steering_angle)
        cv2.imshow('heading', heading_img)
        
        turn = -1 * scaling_factor * steering_angle
        px.set_dir_servo_angle(int(turn))

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            cv2.destroyAllWindows()
            camera.camera.close()
            px.stop()
            break
        
        camera.raw_cap = PiRGBArray(camera.camera, size=camera.camera.resolution)