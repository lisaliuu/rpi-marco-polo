import zmq
import time
import RPi.GPIO as GPIO
from Command import COMMAND as cmd
from Motor import *   

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")

GPIO.setwarnings(False)
Buzzer_Pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(Buzzer_Pin,GPIO.OUT)
class Buzzer:
    def run(self,command):
        if command!="0":
            GPIO.output(Buzzer_Pin,True)
        else:
            GPIO.output(Buzzer_Pin,False)

if __name__=='__main__':
    B=Buzzer()
    PWM=Motor()   
    while(True):
        try:
            i = input()
            if (i=="w"):
                print("w pressed")
                PWM.setMotorModel(1500, 1500, 1500, 1500)
            elif (i=="a"):
                print("a pressed")
                PWM.setMotorModel(-1500, -1500, 1500, 1500)
            elif (i=="d"):
                print("d pressed")        
                PWM.setMotorModel(1500, 1500, -1500, -1500) 
            elif (i=="s"):
                print("s pressed")        
                PWM.setMotorModel(0,0,0,0)
            elif(i=="b"):

                message = "Sound Detected"
                socket.send_string(message)
                print("Sent:", message)
                time.sleep(2)

                B.run('1')
                time.sleep(.5)
                B.run('0')
            elif(i=="q"):
                PWM.setMotorModel(0,0,0,0)
                B.run('0')
                break
        except Exception:
            PWM.setMotorModel(0,0,0,0)
            B.run('0')


