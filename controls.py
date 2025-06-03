import smbus
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

i2c = smbus.SMBus(1)

# MPU6050 registers
MPU_ADDR    = 0x68
PWR_MGMT_1  = 0x6B
SMPLRT_DIV  = 0x19
CONFIG      = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE  = 0x38
ACC_X_H     = 0x3B
ACC_Y_H     = 0x3D
ACC_Z_H     = 0x3F
GYRO_X_H    = 0x43
GYRO_Y_H    = 0x45
GYRO_Z_H    = 0x47

class MPU:
    def __init__(self):
        # set up registers
        i2c.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)
        i2c.write_byte_data(MPU_ADDR, SMPLRT_DIV, 7)
        i2c.write_byte_data(MPU_ADDR, CONFIG, 0)
        i2c.write_byte_data(MPU_ADDR, GYRO_CONFIG, 0)
            
        #places to store readings
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0

        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0

        # to track the amount the droid has rotated
        self.heading = 0.0
        self.last_time = time.time()

    # returns acc or gyro data in g or deg/s respectively 
    def read_one(self, addr):
        high = i2c.read_byte_data(MPU_ADDR, addr)
        low = i2c.read_byte_data(MPU_ADDR, addr+1)

        raw = (high << 8) | low

        if raw > 32767:
            raw -= 65536

        if addr in [0x3B, 0x3D, 0x3F]:     # acc reading
            data = raw / 16384.0
        elif addr in [0x43, 0x45, 0x47]:   # gyro reading
            data = raw / 131.0
        else:
            data = 0.0

        return data

    def read_acc(self):
        self.acc_x = self.read_one(ACC_X_H)
        self.acc_y = self.read_one(ACC_Y_H)
        self.acc_z = self.read_one(ACC_Z_H)

    def read_gyro(self):
        self.gyro_x = self.read_one(GYRO_X_H)
        self.gyro_y = self.read_one(GYRO_Y_H)
        self.gyro_z = self.read_one(GYRO_Z_H)

    def update_heading(self):
        now = time.time()
        dt = now - self.last_time

        self.read_gyro()

        if abs(self.gyro_z) < 1.28:
            data = 0.0
        else:
            data = self.gyro_z

        dh = data * dt       # heading change
        self.heading += dh 

        self.last_time = now

    def get_heading(self):
        self.update_heading()
        return self.heading

    def reset_heading(self):
        self.heading = 0.0
        self.last_time = time.time()


class DistanceSensor:
    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin 

        GPIO.setup(trigger_pin, GPIO.OUT)
        GPIO.setup(echo_pin, GPIO.IN)
        GPIO.output(trigger_pin, False)

    def get_reading(self, timeout=0.03):
        """ get distance in cm """
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00001) # 10us pulse to start
        GPIO.output(self.trigger_pin, False)

        start = time.time()
        pulse_start = start
        pulse_end = start
        timeout_time = start + timeout

        while GPIO.input(self.echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start > timeout_time:
                return -1

        while GPIO.input(self.echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end > timeout_time:
                return -1


        elapsed = pulse_end - pulse_start

        distance = elapsed * 34300 # speed of sound in cm/s
        distance = distance / 2

        return round(distance, 1)

class SensorArray:
    def __init__(self, left_trig, left_echo, center_trig, center_echo, right_trig, right_echo):
        self.left = DistanceSensor(left_trig, left_echo)
        self.center = DistanceSensor(center_trig, center_echo)
        self.right = DistanceSensor(right_trig, right_echo)

    def read_all(self):
        return {
            'left'   : -1,                            # sensor isn't working...
            'center' : self.center.get_reading(),
            'right'  : self.right.get_reading()
        }


if __name__ == "__main__":
    mpu = MPU()
    dist_sensor = DistanceSensor(trigger_pin=22, echo_pin=18)

    try:
        while True:
            #mpu.read_gyro()
            #print(f"Gx = {mpu.gyro_x:.2f}, \tGy = {mpu.gyro_y:.2f}, \tGz = {mpu.gyro_z:.2f}\n")

            distance = dist_sensor.get_reading()

            if distance > 0:
                print(f"distance: {distance}cm")
            else:
                print("no reading")

            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("\nended successfully")


            

