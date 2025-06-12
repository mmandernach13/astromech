import smbus
import time
from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory

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

    def update_heading(self, normalize_heading=False):
        now = time.time()
        dt = now - self.last_time

        self.read_gyro()

        if abs(self.gyro_z) < 1.6:
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

class SensorArray:
    def __init__(self, left_trig, left_echo, center_trig, center_echo, right_trig, right_echo):
        max_dist = 3
        factory = PiGPIOFactory()
        self.left = DistanceSensor(trigger=left_trig, echo=left_echo, max_distance=max_dist, pin_factory=factory)
        self.center = DistanceSensor(trigger=center_trig, echo=center_echo, max_distance=max_dist, pin_factory=factory)
        self.right = DistanceSensor(trigger=right_trig, echo=right_echo, max_distance=max_dist, pin_factory=factory)

    def read_all(self):
        return {
            'left'   : self.left.distance * 100.0,                         
            'center' : self.center.distance * 100.0,
            'right'  : self.right.distance * 100.0
        }


if __name__ == "__main__":
    mpu = MPU()
    dist_sensor = SensorArray(left_trig=18, left_echo=22,
                              center_trig=17, center_echo=27,
                              right_trig=23, right_echo=24)

    try:
        while True:
            #mpu.read_gyro()
            #print(f"Gx = {mpu.gyro_x:.2f}, \tGy = {mpu.gyro_y:.2f}, \tGz = {mpu.gyro_z:.2f}\n")

            r_distance = dist_sensor.right.distance * 100
            c_distance = dist_sensor.center.distance * 100

            
            print(f"center: {c_distance:6.2f}, right: {r_distance:6.2f}")

            time.sleep(1)
    except KeyboardInterrupt:
        print("\nended successfully")


            

