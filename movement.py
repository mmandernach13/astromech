from adafruit_servokit import ServoKit
from gpiozero import Motor
from gpiozero.pins.pigpio import PiGPIOFactory
import time

driver = ServoKit(channels=16)

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
SHOULDER_REST = 5

class MovementMotors:
    def __init__(self):
        factory = PiGPIOFactory()

        self.right_wheel = Motor(IN1, IN2, pin_factory=factory)
        self.left_wheel = Motor(IN3, IN4, pin_factory=factory)

        self.left_shoulder = driver.servo[LEFT_SHOULDER]
        self.right_shoulder = driver.servo[RIGHT_SHOULDER]

        self.left_shoulder.actuation_angle = SHOULDER_RANGE
        self.right_shoulder.actuation_angle = SHOULDER_RANGE

        self.left_shoulder.angle = SHOULDER_REST
        self.right_shoulder.angle = SHOULDER_REST

    def drive(self, left_speed=0.0, right_speed=None):
        """ drives the wheel motors at same or different speeds
            
            param: left_speed between -1 and 1 (<0 backwards, >0 forwards)
                   right_speed (optional)
        """
        left_speed = max(-1, min(1, left_speed))

        if right_speed is not None:
            right_speed = max(-1, min(1, right_speed))

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
        self.left_shoulder.angle = SHOULDER_REST
        self.right_shoulder.angle = SHOULDER_REST

    def turn(self, speed=1.0, direction='CW'):
        """ rotates robot at specified speed in specified direction 

            param: speed= between 0 and 1
                   direction= clockwise or counterclockwise
        """
        speed = max(0, min(1, speed))

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
            


