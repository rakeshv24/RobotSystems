import sys
sys.path.append(r'/home/vader/picar-x/lib')
sys.path.append(r'/home/darth/workspace/picar-x/lib')
import rossros as ros
from picarx_improved import Picarx
from lane_grayscale import Sensing, Interpreter, Controller
from ultrasonic import UltrasonicSensing, UltrasonicInterpreter, UltrasonicController
import concurrent.futures


if __name__ == "__main__":
    px = Picarx()

    ultrasonic_sensor = UltrasonicSensing()
    ultrasonic_interp = UltrasonicInterpreter()
    ultrasonic_controller = UltrasonicController()

    grayscale_sensor = Sensing()
    grayscale_interp = Interpreter()
    grayscale_controller = Controller(px)

    gs_bus = ros.Bus(name="Grayscale Sensor")
    gs_processor_bus = ros.Bus(name="Grayscale Processor")
    us_bus = ros.Bus(name="Ultrasonic Sensor")
    us_processor_bus = ros.Bus(name="Ultrasonic Processor")

    default_term_bus = ros.Bus(False)

    us_producer = ros.Producer(ultrasonic_sensor.read,
                               us_bus,
                               delay=0.05,
                               termination_bus=default_term_bus,
                               name="Ultrasonic Consumer")

    gs_producer = ros.Producer(grayscale_sensor.sensor_reading,
                               gs_bus,
                               delay=0.05,
                               termination_bus=default_term_bus,
                               name="Grayscale Producer")

    us_prod_cons = ros.ConsumerProducer(ultrasonic_interp.processing,
                                        us_bus,
                                        us_processor_bus,
                                        delay=0.05,
                                        termination_bus=default_term_bus,
                                        name="Ultrasonic Consumer-Producer")

    gs_prod_cons = ros.ConsumerProducer(grayscale_interp.output,
                                        gs_bus,
                                        gs_processor_bus,
                                        delay=0.05,
                                        termination_bus=default_term_bus,
                                        name="Grayscale Consumer-Producer")

    us_cons = ros.Consumer(ultrasonic_controller.control,
                           us_processor_bus,
                           delay=0.05,
                           termination_bus=default_term_bus,
                           name="Ultrasonic Consumer")

    gs_cons = ros.Consumer(grayscale_controller.control,
                           gs_processor_bus,
                           delay=0.05,
                           termination_bus=default_term_bus,
                           name="Grayscale Consumer")

    timer = ros.Timer(default_term_bus,
                      duration=5,
                      delay=0.05,
                      termination_bus=default_term_bus,
                      name="Timer")

    try:
        ros.runConcurrently([us_producer, gs_producer, us_prod_cons, gs_prod_cons, us_cons, gs_cons, timer])
    except BaseException:
        print("error in execution")
    finally:
        ultrasonic_controller.stop()
        grayscale_controller.stop()
