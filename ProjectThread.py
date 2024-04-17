from threading import Thread, Lock, Condition
import time
from Motor import *
import RPi.GPIO as GPIO
from servo import *
from PCA9685 import PCA9685
from ObjectAvoidance import *
##### Include your files here
from Turn_To_Angle import *

# Global variables for shared data and synchronization
mutex = Lock()  # Mutex to ensure thread-safe access to shared variables
wait_sound = Condition(mutex)  # Condition variable for sound detection
wait_OV = Condition(mutex)  # Condition variable for object avoidance

sound_detected = False  # Shared variable to signal sound detection
object_avoidance_running = True  # Shared variable to indicate whether object avoidance is running

def echolocation_thread():
    print('Echolocation thread is starting...')
    ##### Initialization of object
    global sound_detected, object_avoidance_running
    while True:
        # Listen for Sound
        echolocation.listen()
        # Update local sound condition
        sound_detected = echolocation.get_sound_status()
        
        with mutex:
            while object_avoidance_running:
                wait_OV.wait()
            PWM.setMotorModel(0,0,0,0) # Stop Motors from OV    
            ##### Run your motor here
            echolocation.turn_to_angle()
            ##### Stop your motor here
            sound_detected = False
            wait_sound.notify_all()

def object_avoidance_thread():
    print('Object Avoidance thread is starting...')
    global sound_detected, object_avoidance_running
    while True:
        # L to R Scan - Allow Faster response
        with mutex:
            while sound_detected:
                wait_sound.wait() # Block
            object_avoidance_running = True
            ## Move Motor Here
            objectAVD.scanLtoR()
            objectAVD.run_motor(objectAVD.L, objectAVD.M, objectAVD.R)
            ##################
            object_avoidance_running = False
            wait_OV.notify()

        # R to L Scan - Allow Faster response
        with mutex:
            while sound_detected:
                wait_sound.wait() # Block
            object_avoidance_running = True
            ## Move Motor Here
            objectAVD.scanRtoL()
            objectAVD.run_motor(objectAVD.L, objectAVD.M, objectAVD.R)
            ##################
            object_avoidance_running = False
            wait_OV.notify()


if __name__ == '__main__':
    try:
        print('Program is starting ...')
        
        ##### Echolocation Object
        echolocation = Turn_To_Angle()
        ##### Run your calibration Here
        echolocation.calibrate_angles()
        ##### Initialize your object here
        echolocation.init_mic()
        
        ##### Avoidance Object
        objectAVD = ObjectAvoidance()

        echolocation_thread = Thread(target=echolocation_thread)
        echolocation_thread.daemon = True
        echolocation_thread.start()

        object_avoidance_thread = Thread(target=object_avoidance_thread)
        object_avoidance_thread.daemon = True
        object_avoidance_thread.start()

        # Main program continues here if needed
        while True:
            time.sleep(1)  # Example sleep to keep the main program running

    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program  will be  executed.
        PWM.setMotorModel(0,0,0,0)
