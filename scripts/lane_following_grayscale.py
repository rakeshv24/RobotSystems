import sys
sys.path.append(r'/home/vader/picar-x/lib')
sys.path.append(r'/home/darth/workspace/picar-x/lib')
import time
from picarx_improved import Picarx
from lane_grayscale import Sensing, Interpreter, Controller


if __name__=="__main__":
    px = Picarx()
    sensor = Sensing()
    interp = Interpreter(sensitivity=0.5, polarity=0)
    controller = Controller(px, scale=20)
    
    timeout = 3.0 + time.time()
    
    while True:
        sensor_values = sensor.sensor_reading()
        robot_pos = interp.output(sensor_values)
        controller.control(robot_pos)
        
        if time.time() > timeout:
            break
            
    px.stop()