import sys
sys.path.append(r'/home/vader/picar-x/lib')
sys.path.append(r'/home/darth/workspace/picar-x/lib')
import logging
from logdecorator import log_on_start, log_on_end, log_on_error
from adc import ADC
logging.basicConfig(format="%(asctime)s:%(message)s", level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)


class Sensing():
    def __init__(self):
        self.channel_0 = ADC("A0")
        self.channel_1 = ADC("A1")
        self.channel_2 = ADC("A2")

    @log_on_start(logging.DEBUG, "Reading sensor values")
    @log_on_error(logging.DEBUG, "Error when reading sensor values")
    @log_on_end(logging.DEBUG, "Successfully read the sensor values")
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
        '''
        Polarity:
        0 - Dark
        1 - Light
        '''

        # Limiting sensitivity to be between 0 and 1.
        if sensitivity > 1:
            self.sensitivity = 1
        elif sensitivity < 0:
            self.sensitivity = 0
        else:
            self.sensitivity = sensitivity
            
    @log_on_start(logging.DEBUG, "Obtaining the position of the robot")
    @log_on_error(logging.DEBUG, "Error obtainng the position of the robot")
    @log_on_end(logging.DEBUG, "Robot poisition obtained")
    def processing(self, sensor_values):
        '''
        Identifies if there is a sharp change in the sensor values (indicative of an edge), and then uses the edge 
        location and sign to determine both whether the system is to the left or right of being centered, and whether 
        it is very off-center or only slightly off-center. 
        '''
        
        min_value = 750
        max_value = 1500
        
        # Clamping the sensor values
        bounded_values = []
        
        for i in range(len(sensor_values)):
            if sensor_values[i] < min_value:
                bounded_values.append(min_value)
            elif sensor_values[i] > max_value:
                bounded_values.append(max_value)
            else:
                bounded_values.append(sensor_values[i])
        
        '''
        The difference between the outer sensor values is evaluated to determine if the car is oriented towards the left
        or right. It is then divided by the center value to normalize the value to be between -1 and 1.
        '''        
        position = ((bounded_values[2] - bounded_values[0]) / bounded_values[1]) * self.sensitivity
        return position
        
    def output(self, sensor_values):
        '''
        This method returns the position of the robot relative to the line as a value on the interval [−1,1], 
        with positive values being to when the line is to the left of the robot.
        '''
        position = self.processing(sensor_values)
        
        if self.polarity == 1:
            position *= -1
            
        return position


class Controller():
    def __init__(self, scale=20):
        self.scale = scale
    
    @log_on_start(logging.DEBUG, "Changing steer angle")
    @log_on_error(logging.DEBUG, "Error when changing steer angle")
    @log_on_end(logging.DEBUG, "Changed steer angle")
    def control(self, px, position):
        steering_angle = self.scale * position
        px.set_dir_servo_angle(steering_angle)
        return steering_angle