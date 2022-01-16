import sys
sys.path.append(r'/home/vader/picar-x/lib')
from picarx_improved import Picarx
import time
import logging
from logdecorator import log_on_start, log_on_end, log_on_error

logging_format = "%(asctime)s : %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

@log_on_start(logging.DEBUG, "Initiating parallel parking towards the {dir} at speed {speed}")
@log_on_error(logging.DEBUG, "Error while parking parallely!")
@log_on_end(logging.DEBUG, "Successfully parked")        
def parallel_parking(px, speed, dir="Right"):
    angle = 30.0 if dir == "Right" else -30.0
    rev_speed = -abs(speed)
    
    px.move(rev_speed, angle)
    time.sleep(1.5)
    px.move(rev_speed, -angle)
    time.sleep(1.5)
    px.move(abs(speed), 0.0)
    time.sleep(0.5)
    px.stop()

@log_on_start(logging.DEBUG, "Initiating K-turn towards the {dir} at speed {speed}")
@log_on_error(logging.DEBUG, "Error while turning!")
@log_on_end(logging.DEBUG, "Executed the K-turn")       
def k_turn(px, speed, dir="Right"):
    speed = abs(speed)
    angle = 30.0 if dir == "Right" else -30.0
    
    px.move(speed, angle/2)
    time.sleep(0.5)
    px.stop()
    px.move(speed, -angle)
    time.sleep(1.0)
    px.stop()
    px.move(-speed, angle)
    time.sleep(1.0)
    px.stop()
    px.move(speed, -angle/2)
    time.sleep(1.0)
    px.move(speed, 0.0)
    time.sleep(1.5)
    px.stop()

@log_on_start(logging.DEBUG, "Moving forward and backward at speed {speed}")
@log_on_error(logging.DEBUG, "Error while executing motion!")
@log_on_end(logging.DEBUG, "Movement complete!")       
def test_motion(px, speed, angle = 0.0):
    px.move(speed, angle)
    time.sleep(1.0)
    px.move(-speed, angle)        
    time.sleep(1.0)
    px.stop()
    

if __name__=="__main__":
    while(1):
        px = Picarx()
        speed = 50
        choice = int(input('1: Move forward and backward \n2: Parallel parking \n3: K-Turn \n4: Set speed (Default=50) \n5: Exit \nEnter Choice: '))
        if choice == 1:
            test_motion(px, speed)
        elif choice == 2:
            dir = str(input('\nEnter Direction (Left/Right): '))
            parallel_parking(px, speed, dir=dir)
        elif choice == 3:
            dir = str(input('\nEnter Direction (Left/Right): '))
            k_turn(px, speed, dir=dir)
        elif choice == 4:
            speed = int(input('\nEnter Speed: '))
        elif choice == 5:
            break
        else:
            logging.error("Entered invalid option")