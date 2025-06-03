from adafruit_servokit import ServoKit
import time

# channels of servos on driver
LEFT_WHEEL = 0  
LEFT_SHOULDER = 1
RIGHT_WHEEL = 2     
RIGHT_SHOULDER = 3  

# rom of shoulder servos
SHOULDER_RANGE = 180 
SHOULDER_MIDPT = SHOULDER_RANGE / 2
USABLE_S_RANGE = 30

class MovementServos:
    def __init__(self):
        self.driver = ServoKit(channels=16)

        self.left_wheel = self.driver.continuous_servo[LEFT_WHEEL]
        self.right_wheel = self.driver.continuous_servo[RIGHT_WHEEL]

        self.left_shoulder = self.driver.servo[LEFT_SHOULDER]
        self.right_shoulder = self.driver.servo[RIGHT_SHOULDER]

        self.left_shoulder.actuation_angle = SHOULDER_RANGE
        self.right_shoulder.actuation_angle = SHOULDER_RANGE

        self.left_shoulder.angle = SHOULDER_MIDPT - 15
        self.right_shoulder.angle = SHOULDER_MIDPT + 15

        self.shoulder_angle = 0.0

    def drive(self, left_speed=0.0, right_speed=None):
        if right_speed is not None:
            self.left_wheel.throttle = left_speed
            self.right_wheel.throttle = right_speed
        else:
            self.left_wheel.throttle = left_speed
            self.right_wheel.throttle = -left_speed

    def stop(self):
        self.left_wheel.throttle = 0
        self.right_wheel.throttle = 0

    def reset_shoulders(self):
        self.left_shoulder.angle = SHOULDER_MIDPT - 15
        self.right_shoulder.angle = SHOULDER_MIDPT + 15

    def rotate(self, speed=0.0, direction=None):
        if direction == 'CW':
            self.left_wheel.throttle = speed
            self.right_wheel.throttle = speed
        elif direction == 'CCW':
            self.left_wheel.throttle = -speed
            self.right_wheel.throttle = -speed

    def tip(self, deg):
        def update_shoulders(self):
            self.left_shoulder.angle = SHOULDER_MIDPT + self.shoulder_angle - 15
            self.right_shoulder.angle = SHOULDER_MIDPT - self.shoulder_angle + 15

        if deg + self.shoulder_angle > USABLE_S_RANGE or deg + self.shoulder_angle < -USABLE_S_RANGE:
            exit()

        for i in range(0, abs(deg)+1, 2):
            d = 2
            if deg < 0:
                d = -d
            self.shoulder_angle += d
            print(self.shoulder_angle)
            update_shoulders(self)
            time.sleep(0.1)
            

if __name__ == "__main__":
    m = MovementServos()

    try:
       while True:
           m.tip(10)
           time.sleep(1)
           m.tip(-20)
           time.sleep(1)
           m.tip(10)
           time.sleep(1)
           #m.drive(0.25)
           #time.sleep(1)
           #m.stop()
           #time.sleep(1)
           m.rotate(0.5, 'CCW')
           time.sleep(5)
           m.stop()
           time.sleep(1)
    except KeyboardInterrupt:
        m.reset_shoulders()
        m.stop()
        print("\nended successfully")
            


