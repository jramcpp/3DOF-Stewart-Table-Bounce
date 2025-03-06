# Precision Ball Control Platform

## Overview
The **Precision Ball Control Platform** is a **Stewart platform** designed to control and balance a ball on a tilting surface using a PID control system and computer vision. The platform is actuated by **three linear actuators**, each driven by a combination of **revolute, prismatic, and spherical joints**, allowing movement with **three degrees of freedom**. The goal of this project is to implement **real-time feedback control** to maintain and manipulate the ball’s position dynamically.

## Features
- **Inverse Kinematics**: Calculates the required actuator positions based on the desired platform tilt.
- **PID Control**: Implements a proportional-integral-derivative controller to maintain ball stability.
- **Computer Vision Feedback**: Uses a camera to track the ball’s position in real-time.
- **Mathematical Modeling**: Utilizes the Euler-Lagrange method for system dynamics.
- **Arduino Integration**: Communicates with actuators to adjust the platform based on control inputs.

## System Components
- **Stewart Platform** (Hexagonal surface, 210mm circumscribed radius)
- **Three Linear Actuators** (Driven by servo motors)
- **Camera System** (Mounted at a fixed height of 440mm)
- **Microcontroller** (Arduino-based control system)
- **Software** (PID control, inverse kinematics, and computer vision processing)

## Getting Started
### Prerequisites
Ensure you have the following dependencies installed:
- Arduino IDE
- Python 3 (for computer vision processing)
- OpenCV (for image processing)
- NumPy & SciPy (for mathematical calculations)
- Serial Communication Library (pyserial for Python-Arduino communication)



## Troubleshooting
- **Excessive oscillation**: Reduce the **derivative gain (Kd)** in the PID controller.
- **Platform not responding**: Check Arduino serial communication and power supply.
- **Ball tracking errors**: Ensure proper lighting conditions and adjust OpenCV threshold settings.
   ```
1. **Adjust PID gains** in `control_system.py` if necessary:
   ```python
   Kp = 1.0  # Proportional gain
   Ki = 0.1  # Integral gain
   Kd = 0.05 # Derivative gain
   ```

## Inverse Kinematics
The inverse kinematics model calculates the required actuator positions to achieve the desired platform tilt, considering the **offsets**:
- Platform center offsets: `xOffset = 158`, `yOffset = 102`
- Camera-coordinate inversion: `x <-> y` due to orientation



## Future Improvements
- Implement **Kalman filtering** for improved motion prediction.
- Enhance **actuator dynamics modeling** for better precision.
- Optimize **real-time processing** for faster response times.

## Contributing
Contributions are welcome! Feel free to submit pull requests or open issues for suggestions.

## License
This project is licensed under the **MIT License**.

## Acknowledgments
Special thanks to the **Open Source Robotics Community** and **Arduino** for their contributions to control systems and embedded programming.

