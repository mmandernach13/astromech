from controls import MPU 
import time

def test_heading():
    mpu = MPU()

    print("testing heading tracking")
    mpu.reset_heading()
    print("heading reset")

    try:
        while True:
            heading = mpu.get_heading()
            print(f"current heading: {heading:6.1f} deg\r")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print(f"\nfinal heading: {mpu.get_heading():.1f} deg")


if __name__ == "__main__":
    test_heading()
