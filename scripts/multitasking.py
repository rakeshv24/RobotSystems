import concurrent.futures
import sys
import os
sys.path.append(r'/home/vader/picar-x/lib')
sys.path.append(r'/home/darth/workspace/picar-x/lib')
import logging
from bus import Bus
from picarx_improved import Picarx
from lane_grayscale import Sensing, Interpreter, Controller
import time
logging.basicConfig(format="%(asctime)s:%(message)s", level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.INFO)


if __name__ == "__main__":
    px = Picarx()

    sense_bus = Bus()
    interpret_bus = Bus()
    control_bus = Bus()

    sensor = Sensing()
    interpreter = Interpreter()
    controller = Controller(px)

    delay = 0.5
    timeout = 3.0 + time.time()

    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        sensor_ex = executor.submit(sensor.sensor_thread, sense_bus, delay)
        interpret_ex = executor.submit(interpreter.interpreter_thread, sense_bus, interpret_bus, delay)
        control_ex = executor.submit(controller.controller_thread, interpret_bus, control_bus, delay)

    sensor_ex.result()
    interpret_ex.result()
    control_ex.result()
    
    while time.time() < timeout:    
        px.forward(30)
    
    px.stop()    
