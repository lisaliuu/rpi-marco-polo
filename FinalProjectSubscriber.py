from threading import Thread, Lock, Condition
import time
from Motor import *
import RPi.GPIO as GPIO
from servo import *
from PCA9685 import PCA9685 
from ObjectAvoidance import *
from Turn_To_Angle import *
import zmq


##### Include your files here

# Global variables for shared data and synchronization
mutex = Lock()  # Mutex to ensure thread-safe access to shared variables
wait_sound = Condition(mutex)  # Condition variable for sound detection
wait_OV = Condition(mutex)  # Condition variable for object avoidance

sound_detected = False  # Shared variable to signal sound detection
object_avoidance_running = True  # Shared variable to indicate whether object avoidance is running
sound_waiting = False

#### ZMQ
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://10.68.201.215:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, "")

def echolocation_thread():
    print('Echolocation thread is starting...')
    PWM.setMotorModel(0,0,0,0)
    ##### Initialization of object
    global sound_detected, object_avoidance_running, sound_waiting
    while True:
        ##### Pause for detection
        message = socket.recv_string()
        print("Received:", message)
        sound_detected = True 

        with mutex:
            while object_avoidance_running:
                wait_OV.wait()
            PWM.setMotorModel(0,0,0,0) # Stop Motors from OV    
            ##### Run your motor here
            print("")
            print("----------------------------")
            print("Running Motors: Echolocation")

            time.sleep(1)
            while(not sound_waiting):
                echolocation.listen()
                sound_waiting = echolocation.get_sound_status()

            echolocation.turn_to_angle()
            #time.sleep(5) # Emulate some actions
            print("END: Echolocation")
            print("----------------------------")
            ##### Stop your motor here
            sound_detected = False
            sound_waiting = False
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
        ##### Run your calibration Here
        ##### Initialize your object here
        objectAVD = ObjectAvoidance()
        ##### Echolocation Object
        echolocation = Turn_To_Angle()
        echolocation.calibrate_angles()
        echolocation.init_mic()

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