from usb_4_mic_array.tuning import Tuning
import usb.core
import usb.util
import time 
from Motor import *
from ADC import * 

class Turn_To_Angle: 
    def __init__(self) -> None:
        self.__left_conversion = 0
        self.__right_conversion = 0
        self.sound = False
        self.__angle = 0
        self.adc=Adc() 
        self.PWM=Motor()
    
    def run(self): 
        try: 
            
            self.calibrate_angles()
            while True:  
                # Setup microphone
                self.init_mic()
                
                # Listen for sound
                self.listen()
                
                # Get angle if a sound is present
                if self.sound:
                    self.find_angle()
                    print(self.__angle)
                
                    # Turn to angle
                    if (self.__angle <= 180):
                        elapse_time = self.__left_conversion * self.__angle
                        print(f'Calculated time: {elapse_time}\n')
                        self.turn_left()
                    else:
                        self.__angle = 360 - self.__angle
                        elapse_time = self.__right_conversion * self.__angle
                        print(f'Calculated time: {elapse_time}\n')
                        self.turn_right()
                    time.sleep(elapse_time)
                    self.stop()
                    self.sound = 0  
                    time.sleep(1)
                     
        except KeyboardInterrupt: 
           self.PWM.setMotorModel(0,0,0,0)  

    def turn_right(self):
        self.PWM.setMotorModel(4095,4095,-4095,-4095)

    def turn_left(self):
        self.PWM.setMotorModel(-4095,-4095,4095,4095)

    def stop(self):
        self.PWM.setMotorModel(0,0,0,0)
        
    def calibrate_angles(self):
        ### Calibrate right conversion value.
        _ = input("Press enter to begin right calibration. Press enter again once robot has made a full turn.\n")
        # Get start time
        start = time.clock_gettime(time.CLOCK_MONOTONIC)
        # Turn 360
        self.turn_right()
        _ = input("Press enter once car has made a full turn\n")
        # Get stop time and stop the car
        end = time.clock_gettime(time.CLOCK_MONOTONIC)
        self.stop()
        # Set conversion value
        self.__right_conversion = (end - start) / 360
        
        ### Calibrate left conversion value.
        _ = input("Press enter to begin left calibration. Press enter again once robot has made a full turn.\n")
        start = time.clock_gettime(time.CLOCK_MONOTONIC)
        self.turn_left()
        _ = input("Press enter once car has made a full turn\n")
        end = time.clock_gettime(time.CLOCK_MONOTONIC)
        self.stop()
        self.__left_conversion = (end - start) / 360
        
        print(f'Angles calibrated. Right conversion == {self.__right_conversion}, Left conversion == {self.__left_conversion}\n')
        
        time.sleep(3)
        
    def turn_to_angle(self):
        print("!!!!Echolocation: Turn_to_angle called")
        self.find_angle()
        print(self.__angle)
    
        # Turn to angle
        if (self.__angle <= 180):
            elapse_time = self.__left_conversion * self.__angle
            print(f'Calculated time: {elapse_time}\n')
            self.turn_left()
        else:
            self.__angle = 360 - self.__angle
            elapse_time = self.__right_conversion * self.__angle
            print(f'Calculated time: {elapse_time}\n')
            self.turn_right()

        time.sleep(elapse_time)
        self.stop()
        self.sound = False
        time.sleep(1)
        
    def init_mic(self):
        self.dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        self.Mic_tuning = Tuning(self.dev)
        self.Mic_tuning.set_vad_threshold(3)
        
    def listen(self):
        if self.dev:
            if (self.Mic_tuning.is_voice()):
                print("Sound detected", end=" ")
                self.sound = True
                print(self.sound)
        
    def find_angle(self):
        print(self.Mic_tuning.direction)
        self.__angle = self.Mic_tuning.direction
        
    def get_sound_status(self):
        return self.sound
        
 
if __name__=='__main__': 
    print ('Program is starting ... ') 
    led_Car=Turn_To_Angle() 
    led_Car.run()