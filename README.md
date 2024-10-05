# Meshmerize

This repository contains various `.ino` (Arduino) and `.py` (Python) scripts for controlling and testing motors, encoders, and line-following robots used for robot motor control and testing.

## Project Overview

The goal of this project is to build, test, and control different motors and robots using Arduino and Raspberry Pi. The scripts included in this repository manage various aspects of the system, such as motor control, encoder testing, and implementing line-following using a PID controller.

### Key Components:
- **Arduino motor control** with different test conditions.
- **Encoder testing** for accurate motor control.
- **Line Following** robot using a PID controller.
- **Motor Testing** functionality.
- **Integration with Raspberry Pi** for data publishing.

## Files in this Repository

- `Basic_conditions.ino` - Contains basic conditions for testing motor movement and responses.
- `Complete_Final_Code.ino` - Final, comprehensive code for the robot's motor control setup.
- `Encoder values for 2 motors.ino` - Code to test and print encoder values for two motors.
- `Encoder.ino` - Basic test script for motor encoders.
- `LSRB.ino` - Code for testing the LSRB module.
- `Line_Follower(PID).ino` - A script implementing a PID controller for line-following robots.
- `Motortester.ino` - Script for testing motor functionalities.
- `publisher(RPI).py` - Python script running on Raspberry Pi to publish data to the robot.
- `README.md` - This file, which provides an overview of the repository.

## Getting Started

### Prerequisites

To run these scripts, you'll need the following hardware and software:

- An **Arduino** board (preferably an Uno or similar).
- **DC motors** and **encoders**.
- A **Raspberry Pi** for running the `publisher(RPI).py` script.
- A **Motor driver module** compatible with Arduino.
- **Sensors** for line-following (optional for line-following tasks).
  
### Installing

1. Clone this repository to your local machine:
    ```bash
    git clone https://github.com/username/Meshmerize.git
    ```

2. Upload any of the `.ino` files to your Arduino board using the Arduino IDE.


### Running the Code

#### Arduino
- Open any of the Arduino `.ino` files in the Arduino IDE.
- Select the appropriate port and Arduino board, then click **Upload**.


## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

## Contributions

Contributions to this project are welcome! Please feel free to submit any issues or pull requests on the repository.

## Contact

For questions or inquiries, contact the project owner at `hemanth.padala@gmail.com`.
