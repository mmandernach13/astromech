from adafruit_servokit import ServoKit
from gpiozero import Motor
from gpiozero.pins.pigpio import PiGPIOFactory
import time

# channels of servos on driver
LEFT_SHOULDER = 0  
RIGHT_SHOULDER = 1 

# pins for motor driver
IN1 = 7
IN2 = 8
IN3 = 9
IN4 = 10

# rom of shoulder servos
SHOULDER_RANGE = 170

class MovementMotors:
    def __init__(self):
        print("Initializing drive wheels and shoulder servos")

        factory = PiGPIOFactory()

        self.right_wheel = Motor(IN1, IN2, pin_factory=factory)
        self.left_wheel = Motor(IN3, IN4, pin_factory=factory)

        self.driver = ServoKit(channels=16)

        self.left_shoulder = self.driver.servo[LEFT_SHOULDER]
        self.right_shoulder = self.driver.servo[RIGHT_SHOULDER]

        self.left_shoulder.actuation_angle = SHOULDER_RANGE
        self.right_shoulder.actuation_angle = SHOULDER_RANGE

        self.left_shoulder.angle = 0
        self.right_shoulder.angle = 0

        print("done\n")

    def drive(self, left_speed=0.0, right_speed=None):
        if right_speed is not None:
            if left_speed < 0:
                self.left_wheel.backward(-left_speed)
            else:
                self.left_wheel.forward(left_speed)
            if right_speed < 0:
                self.right_wheel.backward(-right_speed)
            else:
                self.right_wheel.forward(right_speed)
        else:
            if left_speed < 0:
                self.left_wheel.backward(-left_speed)
                self.right_wheel.backward(-left_speed)
            else:
                self.left_wheel.forward(left_speed)
                self.right_wheel.forward(left_speed)

    def stop(self):
        self.left_wheel.stop()
        self.right_wheel.stop()

    def reset_shoulders(self):
        self.left_shoulder.angle = 0
        self.right_shoulder.angle = 0

    def rotate(self, speed=0.0, direction=None):
        if speed > 1 or speed < 0:
            print("invalid speed argument")
            m.stop()
            exit()

        if direction == 'CW':
            self.left_wheel.forward(speed)
            self.right_wheel.backward(speed)
        elif direction == 'CCW':
            self.left_wheel.backward(speed)
            self.right_wheel.forward(speed)

if __name__ == "__main__":
    m = MovementServos()

    try:
       while True:
           print("driving")
           m.rotate(1, 'CCW')
           time.sleep(1)
    except KeyboardInterrupt:
        m.stop()
        print("\nended successfully")
            


