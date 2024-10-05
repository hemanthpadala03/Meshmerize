# Meshmerize

This repository contains various `.ino` (Arduino) and `.py` (Python) scripts for controlling and testing motors, encoders, line-followers, and more, used for robot motor control and testing. 

## Project Overview

The primary goal of this project is to build, test, and control different motors and robots using Arduino and Raspberry Pi. The scripts included in this repository are used for handling various aspects of the system, such as motor control, encoder testing, and implementing line following.

### Key Components:
- **Arduino motor control** using different conditions.
- **Encoder testing** for precise motor control.
- **Line Following** using PID controller.
- **Motor Testing** scripts.
- **Integration with Raspberry Pi**.

## Files in this Repository

- `Basic_conditions.ino` - Basic conditions to test motor movement and responses.
- `Complete_Final_Code.ino` - Final demo code for the project, handling the full motor setup.
- `Encoder values for 2 motors.ino` - Script for testing encoder values on two motors.
- `Encoder.ino` - Basic encoder test.
- `LSRB.ino` - Script for testing the LSRB module.
- `Line_Follower(PID).ino` - Implementing a PID controller for a line follower robot.
- `Motortester.ino` - Testing motor functionalities.
- `publisher(RPI).py` - Python script used on the Raspberry Pi for publishing data to the robot.
- `README.md` - This file, providing an overview of the repository.

## Getting Started

### Prerequisites

To work with these scripts, you'll need:
- An **Arduino** board (preferably an Uno or similar).
- Basic **motors** and **encoders**.
- A **Raspberry Pi** for running `publisher(RPI).py`.
- **Motor driver module** for Arduino.
- **Sensors** for line following (optional, depending on the setup).
  
### Installing

1. Clone this repository:
    ```bash
    git clone https://github.com/username/Meshmerize.git
    ```

2. Upload any of the `.ino` files to your Arduino board via the Arduino IDE.
3. For Python code, ensure you have the necessary packages installed on your Raspberry Pi:
    ```bash
    pip install -r requirements.txt  # If a requirements file is present
    ```

### Running the Code

- To upload any of the Arduino code, open the corresponding `.ino` file in the Arduino IDE and upload it to your board.
- For Raspberry Pi:
    ```bash
    python publisher(RPI).py
    ```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributions

Feel free to contribute to this project by submitting issues or pull requests.

## Contact

For any questions or inquiries, please contact the repository owner at `email@example.com`.
