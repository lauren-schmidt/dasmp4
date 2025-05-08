# Project Report: Design of Autonomous Systems MP4
#### Authors: Lauren Schmidt and Liza Mozolyuk

## Approach
For this project, we implemented an autonomous rover control system including waypoint navigation, obstacle avoidance, and traffic sign detection. We began by analyzing the code provided in `controller.py` and `PID.py`, and followed the TODO comments to implement the required functionality. Then, we implemented ultrasound obstacle detection, stop sign detection, and traffic light color detection using the code base from Lab 9. Finally, we tested our implementation sequentially:
1. **Waypoint Navigation Testing**: Making sure the rover can autonomously drive in a square pattern using the waypoint navigation algorithm 
2. **Stop Sign Detection Testing**: Holding the rover in front of a stop sign and verifying that it pauses when the stop sign is detected. 
3. **Traffic Light Detection Testing**: Holding the rover in front of a traffic light and verifying that it stops when the light is red and continues when the light is green.
4. **Obstacle Detection Testing**: Placing an obstacle in front of the rover and verifying that it stops when the obstacle is detected.

## Algorithms Used
The files that we edited for this project are: 
- `controller.py`: This file has the main control logic for the autonomous system. We updated `algo()` to implement our new control algorithm, which included: 
    - Initializing and updating a counter to keep track of the iteration of the program, which is used to determine when to check for obstacles, stop signs, and trafic lights. 
    - A series of calculations to take the rover from waypoint to waypoint:
        - Calculating the angle betwen the waypoint and the rover's position.
        - Calculating the distance between the waypoint and the rover's position using sensor data. 
        - Calculating the speed in the x direction that the rover should travel to reach the waypoint. 
    - Calls to functions that detect obstacles, stop signs, and traffic lights: 
        - `stop_sign_detected()`: Sets the cascade path for stop sign detection, and uses the `cv2` library to capture frames from the camera. Converts the grames to grayscale, and then uses the `detectMultiScale()` function to detect stop signs in the frame. If a stop sign is detected, it returns `True`, otherwise it returns `False`.
            - The `algo()` function checks for stop signs every 400 iterations, and if a stop sign is detected, it sets the rover speed to 0, and pauses for 3 seconds before resuming navigation.
        - `detect_traffic_light_color()`: Loads the Haar cascade classifier for traffic light detection, captures frames from the camera, and uses the `cv2` library to detect traffic lights in the frame. It then determines the color of the detected traffic light by analyzing the pixels within the bounding box of the detected light. If a red light is detected, it returns `red`, otherwise it returns `green` if the light is green, and otherwise `unknown`.
            - The `algo()` function checks for traffic lights every 400 iterations, and if a red light is detected, it sets the rover speed to 0, and pauses until a green light is detected before resuming navigation.
        - `ultrasound_reading()`:
            - this function uses code from lab 2 to detect the distance of objects near our rover. Once an object is detected. The rover pauses until the obstacle is moved from 
- `PID.py`: This file contains the PID controller implementation. We used this to help control the rover's speed and direction as it navigates towards waypoints. The PID controller helps to minimize the error between the desired position and the actual position of the rover. Our implementation includes:
    - Initializing PID parameters (Kp, Ki, Kd) for both speed and steering control.
    - Updating the PID controller in the `algo()` function to adjust the rover's speed and direction based on the error between the current position and the target waypoint.
- `figure8.csv`:
    - program in `waypoints.py` that uses the paramteric equation for a figure 8 and generates waypoints from points that fall on the figure 8.
  
<img width="721" alt="Screenshot 2025-05-07 at 2 04 39 PM" src="https://github.com/user-attachments/assets/2ffaa43c-04aa-4e21-910e-cbb9aa05f64e" />

<img width="1442" alt="Screenshot 2025-05-07 at 2 21 44 PM" src="https://github.com/user-attachments/assets/7b94f0a2-3b5a-45a3-a16c-65ef456536c2" />

## How to Run the Code 
1. Make sure the rover is powered on and all components are connected properly
2. Make sure the radio-controller is paired and and set to your rover
3. Unplug and Re-plug Intel IMU's USB cable
4. Check if IMU is detected by running `lususb`
5. Pull the SF-Key on the radio-controllor to the DOWN position
6. Place rover on the ground, make sure it has a 2mx3m area to run 
7. Pull the SG-Key on the radio-controller to the UP position for `square`, or to the DOWN position for `eight`
8. Run the following command in the terminal:
    ```
    roslaunch gwurover simple_pid.launch
    ```
9. Pull the SF-Key on the radio-controller to the UP position to start the rover in autonomous mode 
10. When done, pull the SF-Key on the radio-controller to the DOWN position to stop the rover
11. Turn off the ESC BEFORE powering off anything else 
12. Power off the rover and radio-controller

## Assumptions & Limitations 
#### Assumptions
- The rover is assumed to be in an open area with enough space to navigate.
- Assume all IMU sensor data is accurate and reliable (it is sometimes noisy).

#### Limitations
- The rover can detect stop signs and traffic lights *within a certain range*, but may not be able to detect them in conditions with poor lighting or from far distances.
- We encountered issues with the hardware for this project, specifically with connecting to the SLAB Router and the radio-controller. We were frequently booted from the network, which caused delays in testing and debugging code. There was also a limited number of available radio-controllers, and our rover was only able to connect to one of them.
- object detection and 

