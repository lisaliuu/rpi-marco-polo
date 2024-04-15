import time
import RPi.GPIO as GPIO
from Command import COMMAND as cmd
from Motor import *            

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
                PWM.setMotorModel(1000,1000,1000,1000)
                #sleep(.5)
            elif (i=="a"):
                print("a pressed")
                PWM.setMotorModel(-2000,2000,2000,-2000) #PWM.setMotorModel(-1500,-1500,2000,2000)#Turn left
                #time.sleep(.5)
            elif (i=="d"):
                print("d pressed")        
                PWM.setMotorModel(2000,-2000,-2000,2000)#PWM.setMotorModel(2000,2000,-1500,-1500) #Turn right 
                #time.sleep(1)
            elif (i=="s"):
                print("s pressed")        
                PWM.setMotorModel(0,0,0,0)
            elif(i=="b"):
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





