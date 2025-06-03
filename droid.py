from movement import MovementServos
from controls import MPU, SensorArray
from RPi.GPIO import cleanup as gpio_cleanup
import time 

class Droid:
    def __init__(self):
        self.movement = MovementServos()
        self.mpu = MPU()
        self.dist_sensors = SensorArray(left_trig=22, left_echo=18,
                                        center_trig=17, center_echo=27,
                                        right_trig=23, right_echo=24)

        self.current_heading = 0.0
        
        # PID constants
        self.kp = 1.7
        self.ki = 0.7
        self.kd = 0.4

        # PID state variables
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.last_time = time.time()

    def reset_pid(self):
        """ call before new maneuver """
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.last_time = time.time()

    def update_heading(self):
        self.current_heading = self.mpu.get_heading()

    def maintain_heading(self, target_heading, speed, tolerance=0.1, reset_pid=False):
        """ drive in specified direction monitored by a PID controller """
        if reset_pid:
            self.reset_pid()

        now = time.time()
        dt = now - self.last_time

        self.update_heading()

        heading_error = target_heading - self.current_heading

        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360

        P = self.kp * heading_error 

        self.integral_error += heading_error * dt
        self.integral_error = max(-150, min(150, self.integral_error))

        I = self.ki * self.integral_error

        if dt > 0:
            derivative_error = (heading_error - self.previous_error) / dt
            D = self.kd * derivative_error 
        else:
            D = 0

        correction = P + I + D

        left_speed = speed + correction
        right_speed = speed - correction

        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        self.movement.drive(left_speed, right_speed)

        self.last_time = now
        self.previous_error = heading_error

        return abs(heading_error) < tolerance

    def turn_to_heading(self, target_heading, turn_speed=0.5, tolerance=1.0, timeout=10.0):
        """ turn to specified heading """
        start = time.time()
        self.reset_pid()

        while time.time() - start < timeout:
            self.update_heading()

            heading_error = target_heading - self.current_heading
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360 

            if abs(heading_error) < tolerance:
                self.movement.stop()
                return True

            if heading_error > 0: 
                self.movement.rotate(turn_speed, 'CCW')
            else:
                self.movement.rotate(turn_speed, 'CW')

            time.sleep(0.05)

        self.movement.stop()

        return False


if __name__ == "__main__":
    r3m1 = Droid()
    cycles = 0

    try:
        while True:
            dist = r3m1.dist_sensors.center.get_reading()

            if cycles % 2 == 0:
                heading = 0
            else:
                heading = 180

            while dist > 20:
                print(f"dist: {dist}")
                r3m1.maintain_heading(heading, 1.0)
                dist = r3m1.dist_sensors.center.get_reading()

            print(f"heading: {r3m1.current_heading}")

            r3m1.turn_to_heading(r3m1.current_heading + 180)
            cycles += 1

            time.sleep(1)
    except KeyboardInterrupt:
        gpio_cleanup()
        r3m1.movement.stop()
        print("\nended successfully")
