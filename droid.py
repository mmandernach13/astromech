from movement import MovementMotors
from controls import MPU, SensorArray
import time 

class Droid:
    def __init__(self):
        self.movement = MovementMotors()
        self.mpu = MPU()
        self.dist_sensors = SensorArray(left_trig=22, left_echo=18,
                                        center_trig=17, center_echo=27,
                                        right_trig=23, right_echo=24)
        
        # current heading in deg: -180 to 180
        self.current_heading = 0.0
        self.target_heading = 0.0

        # motor imbalances
        self.left_bias = 0.2
        self.right_bias = -0.03
        
        # PID constants
        self.kp = 0.25
        self.ki = 0.05
        self.kd = 0.01

        # PID state variables
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.filtered_derivative = 0.0
        self.last_time = time.time()

    def apply_motor_bias(self, left_speed, right_speed):
        left_speed += self.left_bias
        right_speed += self.right_bias

        left_speed = max(-1, min(1, left_speed))
        right_speed = max(-1, min(1, right_speed))

        return left_speed, right_speed

    def reset_pid(self):
        """ call before new maneuver """
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.last_time = time.time()

    def normalize_heading(self, heading):
        while heading > 180:
            heading -= 360
        while heading < -180:
            heading += 360

        return heading

    def update_heading(self):
        self.current_heading = self.normalize_heading(self.mpu.get_heading())

    def maintain_heading(self, speed, tolerance=0.05, reset_pid=False):
        """ drive in specified direction monitored by a PID controller """
        if reset_pid:
            self.reset_pid()

        now = time.time()
        dt = now - self.last_time

        if dt < 0.01:
            return False

        self.update_heading()

        heading_error = self.normalize_heading(self.target_heading - self.current_heading)

        if abs(heading_error) > 0.1 and (heading_error * self.previous_error < 0):
            self.integral_error = 0

        P = self.kp * heading_error 
        
        if abs(heading_error) < 0.2:
            integral_change = heading_error * dt
            self.integral_error += integral_change
            max_integral_change = 0.5 / max(self.ki, 0.01)
            self.integral_error = max(-max_integral_change, min(max_integral_change, integral_change))

        I = self.ki * self.integral_error

        if dt > 0:
            derivative_error = (heading_error - self.previous_error) / dt
            self.filtered_derivative = 0.7 * self.filtered_derivative + 0.3 * derivative_error
            D = self.kd * self.filtered_derivative 
        else:
            D = 0

        correction = P + I + D

        max_correction = min(0.8, 1 - abs(speed))
        correction = max(-max_correction, min(max_correction, correction))

        left_speed = speed + correction
        right_speed = speed + correction

        left_speed, right_speed = self.apply_motor_bias(left_speed, right_speed)

        print(f"Target: {self.target_heading:6.2f}, Current: {self.current_heading:6.2f}, Error: {heading_error:6.2f}, P: {P:6.3f}, I: {I:6.3f}, D: {D:6.3f}, L: {left_speed:6.2f}, R: {right_speed:6.2f}")

        self.movement.drive(left_speed, right_speed)

        self.last_time = now
        self.previous_error = heading_error

        return abs(heading_error) < tolerance

    def turn_deg(self, relative_deg, turn_speed=0.5, tolerance=1.0, timeout=10.0):
        """ turn to specified heading relative to current """
        relative_deg = max(-180, min(180, relative_deg))

        self.update_heading()
        self.target_heading = self.normalize_heading(self.current_heading + relative_deg)

        start = time.time()
        self.reset_pid()

        while time.time() - start < timeout:
            self.update_heading()

            heading_error = self.target_heading - self.current_heading

            if abs(heading_error) < tolerance:
                self.movement.stop()
                self.target_heading = self.current_heading
                return True

            if heading_error > 0: 
                self.movement.rotate(turn_speed, 'CCW')
            else:
                self.movement.rotate(turn_speed, 'CW')

            time.sleep(0.05)

        self.movement.stop()
        self.target_heading = self.current_heading

        return False


if __name__ == "__main__":
    r3m1 = Droid()

    print(f"Target: {r3m1.target_heading:6.2f}, Current: {r3m1.current_heading:6.2f}")

    #for i in range(100):
        #r3m1.movement.drive(0.5)
        #r3m1.update_heading()
        #print(f"Current: {r3m1.current_heading:6.2f}")
        #time.sleep(0.1)

    #print(f"Target: {r3m1.target_heading:6.2f}, Current: {r3m1.current_heading:6.2f}")


    #for i in range(300):
        #r3m1.update_heading()
        #print(f"Heading: {r3m1.current_heading:6.2f}")
        #time.sleep(0.1)

    try:
        while True:
            dist = r3m1.dist_sensors.center.distance * 100.0

            if dist < 40:
                r3m1.turn_deg(relative_deg=180, turn_speed=0.8)

            r3m1.maintain_heading(0.8)

            #error = -r3m1.current_heading
            #turn_deg = 180

            #print(f"\nturning around\n")

            #success = r3m1.turn_deg(error + turn_deg)
            #print(f"Success: {success}")

            time.sleep(.01)
    except KeyboardInterrupt:
        r3m1.movement.stop()
        print("\nended successfully")
