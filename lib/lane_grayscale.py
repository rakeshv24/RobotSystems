import sys
sys.path.append(r'/home/vader/picar-x/lib')
sys.path.append(r'/home/darth/workspace/picar-x/lib')
import time
import logging
from logdecorator import log_on_start, log_on_end, log_on_error
from picarx_improved import Picarx
from adc import ADC


class Sensing():
    def __init__(self):
        self.channel_0 = ADC("A0")
        self.channel_1 = ADC("A1")
        self.channel_2 = ADC("A2")

    def sensor_reading(self):
        sensor_value_list = []
        sensor_value_list.append(self.channel_0.read())
        sensor_value_list.append(self.channel_1.read())
        sensor_value_list.append(self.channel_2.read())
        return sensor_value_list


class Interpreter():
    def __init__(self, sensitivity=0.5, polarity=0):
        self.sensitivity = sensitivity
        self.polarity = polarity

        if sensitivity > 1:
            self.sensitivity = 1
        elif sensitivity < 0:
            self.sensitivity = 0
        else:
            self.sensitivity = sensitivity
    
    def processing(self, sensor_values):
        min_value = 0
        max_value = 450
        
        bounded_values = []
        
        for i in range(len(sensor_values)):
            if sensor_values[i] < min_value:
                bounded_values.append(min_value)
            elif sensor_values[i] > max_value:
                bounded_values.append(max_value)
            else:
                bounded_values.append(sensor_values[i])
    
        position = ((bounded_values[2] - bounded_values[0]) / bounded_values[1]) * self.sensitivity
        
        print(bounded_values)
        print(position)
        print(sensor_values)
        return position
        
    def output(self, sensor_values):
        position = self.processing(sensor_values)
        
        if self.polarity == 1:
            position *= -1
            
        print(position)
        return position


class Controller():
    def __init__(self, scale=20):
        self.scale = scale
    
    def control(self, px, position):
        steering_angle = self.scale * position
        px.set_dir_servo_angle(steering_angle)
        return steering_angle