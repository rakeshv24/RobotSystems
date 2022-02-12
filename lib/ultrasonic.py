import sys
sys.path.append(r'/home/vader/picar-x/lib')
sys.path.append(r'/home/darth/workspace/picar-x/lib')
import time
import numpy as np


class UltrasonicSensing():
    def __init__(self, px):
        self.px = px

    def get_data(self):
        return self.px.Get_distance()

    def read(self):
        for i in range(10):
            dist = self.get_data()
            if dist != -1 and dist <= 300:
                return dist
        return -1


class UltrasonicInterpreter():
    def __init__(self, stopping_range=10):
        self.stopping_range = stopping_range

    def processing(self, distance):
        if distance < 0:
            return 1
        elif distance > self.stopping_range:
            return 1
        else:
            return 0


class UltrasonicController():
    def __init__(self, px):
        self.px = px

    def control(self, move_signal):
        if move_signal == 1:
            self.px.forward(50)
            time.sleep(0.05)
        else:
            self.px.stop()
            time.sleep(0.05)
