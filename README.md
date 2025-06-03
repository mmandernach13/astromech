# Astromech

## Overview
Astromech is a Python-based project for controlling a robotic droid, designed to run on a Raspberry Pi with servo motors and sensors. The project includes functionality for movement control, heading maintenance using a PID controller, and obstacle detection via ultrasonic sensors. It leverages an MPU6050 for orientation tracking and the Adafruit ServoKit for servo motor control.

This repository, maintained by [mmandenach13](https://github.com/mmandenach13), is located at [github.com/mmandenach13/astromech](https://github.com/mmandenach13/astromech).

## Features
- **Movement Control**: Drive and rotate the droid using continuous servos for wheels and standard servos for shoulder mechanisms.
- **PID Heading Control**: Maintains a specified heading using a PID controller with data from an MPU6050 sensor.
- **Obstacle Detection**: Uses ultrasonic distance sensors to detect obstacles and adjust the droid's behavior.
- **Modular Design**: Organized into classes (`Droid`, `MovementServos`, `MPU`, `SensorArray`) for easy maintenance and extensibility.

## Hardware Requirements
- Raspberry Pi (e.g., Raspberry Pi 4)
- MPU6050 gyroscope/accelerometer
- Ultrasonic distance sensors (e.g., HC-SR04, with trigger and echo pins)
- Adafruit PCA9685 Servo Controller
- Continuous servos for wheels
- Standard servos for shoulder mechanisms
- Appropriate power supply and wiring for servos and sensors

## Software Requirements
See `requirements.txt` for Python dependencies. Key libraries include:
- `adafruit-circuitpython-servokit`
- `smbus`
- `RPi.GPIO`

## Installation
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/mmandenach13/astromech.git
   cd astromech
   ```

2. **Set Up a Virtual Environment** (optional but recommended):
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

3. **Install Dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

4. **Configure Hardware**:
   - Connect the MPU6050 to the Raspberry Pi I2C pins.
   - Connect ultrasonic sensors to the specified GPIO pins (left: 22/18, center: 17/27, right: 23/24).
   - Connect servos to the Adafruit PCA9685 channels (wheels: 0, 2; shoulders: 1, 3).

## Usage
1. **Run the Main Droid Program**:
   ```bash
   python3 droid.py
   ```
   The droid will move forward while maintaining a heading (0° or 180°, alternating), stop when an obstacle is closer than 20 cm, turn 180°, and continue. Press `Ctrl+C` to stop.

2. **Test Individual Components**:
   - **Servo Test**:
     ```bash
     python3 servo_test.py
     ```
     Test servo channel 0 (angle) and channel 1 (continuous).
   - **MPU Test**:
     ```bash
     python3 mpu_test.py
     ```
     Displays the current heading from the MPU6050.

3. **Key Files**:
   - `droid.py`: Main program controlling the droid's behavior.
   - `movement.py`: Handles servo motor control for wheels and shoulders.
   - `controls.py`: Manages MPU6050 and ultrasonic sensor interactions.
   - `servo_test.py`: Tests servo functionality.
   - `mpu_test.py`: Tests MPU6050 heading tracking.

## Notes
- Ensure proper GPIO cleanup by handling `KeyboardInterrupt` (Ctrl+C) to reset servos and GPIO pins.
- The left ultrasonic sensor is currently non-functional (returns -1 in `SensorArray.read_all()`).
- Adjust PID constants (`kp`, `ki`, `kd`) in `droid.py` for better heading control if needed.
- The shoulder servo range is limited to ±30° to prevent damage.

## Contributing
Contributions are welcome! Please submit issues or pull requests to [github.com/mmandenach13/astromech](https://github.com/mmandenach13/astromech).

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
