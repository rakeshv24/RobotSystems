import sys
sys.path.append(r'/home/vader/picar-x/lib')
sys.path.append(r'/home/darth/workspace/picar-x/lib')
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
    
    # To move backwards 
    rev_speed = -abs(speed)
    
    # Moving the car in a curved manner as one would do for parallel parking
    px.move(rev_speed, angle)
    time.sleep(1.5)
    # Changing the steer angle
    px.move(rev_speed, -angle)
    time.sleep(1.5)
    
    # Finally move forward a little and stop
    px.move(abs(speed), 0.0)
    time.sleep(0.5)
    px.stop()

@log_on_start(logging.DEBUG, "Initiating K-turn towards the {dir} at speed {speed}")
@log_on_error(logging.DEBUG, "Error while turning!")
@log_on_end(logging.DEBUG, "Executed the K-turn")       
def k_turn(px, speed, dir="Right"):
    # To move forward
    speed = abs(speed)
    
    angle = 30.0 if dir == "Right" else -30.0
    
    # Move to the curb
    px.move(speed, angle/2)
    time.sleep(0.5)
    px.stop()
    
    # Move forward at an angle
    px.move(speed, -angle)
    time.sleep(1.0)
    px.stop()
    
    # Reversing with the opposite steer angle
    px.move(-speed, angle)
    time.sleep(1.0)
    px.stop()
    
    # Moving forward to merge with the correct lane
    px.move(speed, -angle/2)
    time.sleep(1.0)
    px.move(speed, 0.0)
    time.sleep(1.5)
    
    # Stopping
    px.stop()

@log_on_start(logging.DEBUG, "Moving forward and backward at speed {speed}")
@log_on_error(logging.DEBUG, "Error while executing motion!")
@log_on_end(logging.DEBUG, "Movement complete!")       
def test_motion(px, speed, angle = 0.0):
    # Moving forward
    px.move(speed, angle)
    time.sleep(1.0)
    
    # Moving Backward
    px.move(-speed, angle)        
    time.sleep(1.0)
    px.stop()
    

if __name__=="__main__":
    px = Picarx()
    speed = 50
    
    while(1):
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