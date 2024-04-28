import zmq
import time
import RPi.GPIO as GPIO
from Command import COMMAND as cmd
from Motor import *   

# Defining ZMQ publisher
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")

GPIO.setwarnings(False)
Buzzer_Pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(Buzzer_Pin,GPIO.OUT)

# Buzzer object
class Buzzer:
    def run(self,command):
        if command!="0":
            GPIO.output(Buzzer_Pin,True)
        else:
            GPIO.output(Buzzer_Pin,False)

if __name__=='__main__':
    B=Buzzer()
    PWM=Motor()   
    # Command line control for prey car
    while(True):
        try:
            i = input() # Waiting for user input, press enter after input
            if (i=="w"): # Foward
                print("w pressed")
                PWM.setMotorModel(1500, 1500, 1500, 1500)
            elif (i=="a"): # Turn Left
                print("a pressed")
                PWM.setMotorModel(-1500, -1500, 1500, 1500)
            elif (i=="d"): # Turn Right
                print("d pressed")        
                PWM.setMotorModel(1500, 1500, -1500, -1500) 
            elif (i=="s"): # Stop
                print("s pressed")        
                PWM.setMotorModel(0,0,0,0)
            elif(i=="b"): # Buzzer
                # Send ZMQ message to predator
                message = "Sound Detected" 
                socket.send_string(message)
                print("Sent:", message)
                time.sleep(2) # Wait for predator to process

                # Sound buzzer
                B.run('1')
                time.sleep(.5)
                B.run('0')
            elif(i=="q"): # Quit program
                PWM.setMotorModel(0,0,0,0)
                B.run('0')
                break
        except Exception:
            PWM.setMotorModel(0,0,0,0)
            B.run('0')


