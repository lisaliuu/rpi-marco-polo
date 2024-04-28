import time
from Motor import *
import RPi.GPIO as GPIO
from servo import *
from PCA9685 import PCA9685

class ObjectAvoidance:
    # Defining private variables
    def __init__(self):
        GPIO.setwarnings(False)
        self.trigger_pin = 27
        self.echo_pin = 22
        self.MAX_DISTANCE = 300  # define the maximum measuring distance, unit: cm
        self.timeOut = self.MAX_DISTANCE * 60  # calculate timeout according to the maximum measuring distance
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        self.angles = [23, 45, 68, 90, 113, 135, 158]
        self.left_angles = [23, 45]
        self.middle_angles = [68, 90, 113]
        self.right_angles = [135, 158]
        self.left_distances = []
        self.middle_distances = []
        self.right_distances = []
        self.L = 0
        self.M = 0
        self.R = 0
        self.PWM = Motor()
        self.pwm = Servo()
        self.distance_cm = [0, 0, 0, 0, 0]

    # Helper function: Set all distance group array to 0
    def resetDistanceArray(self):
        self.left_distances.clear()
        self.middle_distances.clear()
        self.right_distances.clear()

    # Scan obstacle from left to right using ultrasonic sensor, update distance array
    def scanLtoR(self):
        self.resetDistanceArray()
        for angle in self.angles:
            self.pwm.setServoPwm('0', angle)
            time.sleep(0.01)
            distance = self.get_distance()  # Get the value

            if angle in self.left_angles:
                self.left_distances.append(distance)
            if angle in self.middle_angles:
                self.middle_distances.append(distance)
            if angle in self.right_angles:
                self.right_distances.append(distance)   
        # Take the minimum and make them L,M,R for decision
        self.L = min(self.left_distances)
        self.M = min(self.middle_distances)
        self.R = min(self.right_distances)

        time.sleep(0.01)

    # Scan obstacle from right to left using ultrasonic sensor, update distance array
    def scanRtoL(self):
        self.resetDistanceArray()
        for angle in reversed(self.angles):
            self.pwm.setServoPwm('0', angle)
            time.sleep(0.01)
            distance = self.get_distance()  # Get the value

            if angle in self.left_angles:
                self.left_distances.append(distance)
            if angle in self.middle_angles:
                self.middle_distances.append(distance)
            if angle in self.right_angles:
                self.right_distances.append(distance)
        # Take the minimum and make them L,M,R for decision
        self.L = min(self.left_distances)
        self.M = min(self.middle_distances)
        self.R = min(self.right_distances)

        time.sleep(0.01)

    def pulseIn(self, pin, level, timeOut):  # obtain pulse time of a pin under timeOut
        t0 = time.time()
        while (GPIO.input(pin) != level):
            if ((time.time() - t0) > timeOut * 0.000001):
                return 0;
        t0 = time.time()
        while (GPIO.input(pin) == level):
            if ((time.time() - t0) > timeOut * 0.000001):
                return 0;
        pulseTime = (time.time() - t0) * 1000000
        return pulseTime

    def get_distance(self):  # get the measurement results of ultrasonic module,with unit: cm
        for i in range(5):
            GPIO.output(self.trigger_pin, GPIO.HIGH)  # make trigger_pin output 10us HIGH level
            time.sleep(0.00001)  # 10us
            GPIO.output(self.trigger_pin, GPIO.LOW)  # make trigger_pin output LOW level
            pingTime = self.pulseIn(self.echo_pin, GPIO.HIGH, self.timeOut)  # read plus time of echo_pin
            self.distance_cm[i] = pingTime * 340.0 / 2.0 / 10000.0  # calculate distance with sound speed 340m/s
        self.distance_cm = sorted(self.distance_cm)

        # IMPORTANT: Ultrasonic sensor returns 0 when distance > 2m. Need to cap this value. 
        if (self.distance_cm[2] == 0): # Filter Data
            return int(200)
        return int(self.distance_cm[2])

    def stop_motor(self):
        self.PWM.setMotorModel(0, 0, 0, 0)

    # Using L,M,R, the distance from obstacle, and run a decision based control
    def run_motor(self, L, M, R):
        
        # Too Close - Back UP
        if (L < 30 and M < 30 and R < 30) or M < 30:
            self.PWM.setMotorModel(-1500, -1500, -1500, -1500)
            time.sleep(0.1)
            if L < R:
                self.PWM.setMotorModel(1500, 1500, -1500, -1500)
                time.sleep(0.1)
            else:
                self.PWM.setMotorModel(-1500, -1500, 1500, 1500)
                time.sleep(0.1)
        elif L < 30 and M < 30:
            self.PWM.setMotorModel(2000, 2000, -2000, -2000)
        elif R < 30 and M < 30:
            self.PWM.setMotorModel(-2000, -2000, 2000, 2000)
        elif L < 20:
            self.PWM.setMotorModel(2000, 2000, -1000, -1000)
            if L < 10:
                self.PWM.setMotorModel(2000, 2000, -1500, -1500)
        elif R < 20:
            self.PWM.setMotorModel(-1000, -1000, 2000, 2000)
            if R < 10:
                self.PWM.setMotorModel(-1500, -1500, 1500, 1500)
        else:
            self.PWM.setMotorModel(1300, 1300, 1300, 1300)

    def run(self):
        while True:
            self.scanLtoR()
            # self.run_motor(self.L, self.M, self.R)
            self.scanRtoL()
            # self.run_motor(self.L, self.M, self.R)

if __name__ == '__main__':
    print('Program is starting ... ')
    try:
        objectAVD = ObjectAvoidance()
        while True:
            objectAVD.run()
            time.sleep(0.01)
    except KeyboardInterrupt:
        # Stop motors and reset servo
        objectAVD.PWM.setMotorModel(0, 0, 0, 0)
        objectAVD.pwm_S.setServoPwm('0', 90)
