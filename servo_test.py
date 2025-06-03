from adafruit_servokit import ServoKit
import time

try: 
    print("init kit")
    kit = ServoKit(channels=16)
    #kit._pca.frequency = 50
    print("init success")

    print("testing ch 0 servo")
    kit.servo[0].angle = 90 
    time.sleep(30)
    print("cmd sent")

    print("testing cts servo")
    kit.continuous_servo[1].throttle = 0.5
    time.sleep(2)
    kit.continuous_servo[1].throttle = 0
    print("cts test complete")

except Exception as e:
    print(f"{e}")
