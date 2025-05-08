#!/usr/bin/env python3

import rospy
from gwurover.msg import RCIN
from gwurover.msg import PID
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gwurover.srv import *
import RPi.GPIO as GPIO

# Pin setup
TRIG = 17
ECHO = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

import os
import cv2
import numpy as np
import sys
import time

from time import sleep


from std_msgs.msg import Float32, String
from tf import euler_from_quaternion
from PID import pid
import math


def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Call backs are basically functions you call to update some variables when an event happens
# Here, we update  the `rcin_msg` object with the data received from the RCIN
# This is done asyncronously
def rcin_callback(data):
    global rcin_msg
    rcin_msg = data

# update the `pid_param_msg` object with the data received from the PID
# Ofc this data object can also be written to so that the PID knows the current state
# You'll see the pids being updated like   `linear_pos_pid.update_pid(0,dist)`
# This is done asyncronously
def pid_param_callback(data):
    if data.kp_steer >= 0: steer_pid.set_term_kp(data.kp_steer)
    if data.ki_steer >= 0: steer_pid.set_term_ki(data.ki_steer)
    if data.kd_steer >= 0: steer_pid.set_term_kd(data.kd_steer)
    if data.offset_steer >= 0: steer_pid.set_term_offset(data.offset_steer)
    if data.kp_speed >= 0: speed_pid.set_term_kp(data.kp_speed)
    if data.ki_speed >= 0: speed_pid.set_term_ki(data.ki_speed)
    if data.kd_speed >= 0: speed_pid.set_term_kd(data.kd_speed)
    if data.offset_speed >= 0: speed_pid.set_term_offset(data.offset_speed)


# Same as the above call backs
def t265_position_callback(data):
    global sensor_pos_msg
    sensor_pos_msg = data
    sensor_pos_msg.position.x *= -1
    sensor_pos_msg.position.z *= -1

# Same as the above call backs
def t265_velocity_callback(data):
    global sensor_vel_msg
    sensor_vel_msg = data

# Detect a stop sign
def stop_sign_detected(frame):
    cascade_path = os.path.join(os.path.dirname(__file__), 'stopsign_good.xml')

    stop_sign_cascade = cv2.CascadeClassifier(cascade_path)

    # Resize the ROI for faster processing
    frame = cv2.resize(frame,(640, 480))

    # Convert to grayscale
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect stop signs
    stop_signs = stop_sign_cascade.detectMultiScale(gray_img, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    if len(stop_signs) > 0:
        # cap.release()
        # cv2.destroyAllWindows()
        return True
    return False

def detect_traffic_light_color(frame):
    # Load the Haar Cascade file
    cascade_path = os.path.join(os.path.dirname(__file__), 'haar_xml_07_19-3.xml')

    traffic_light_cascade = cv2.CascadeClassifier(cascade_path)

    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect traffic lights
    traffic_lights = traffic_light_cascade.detectMultiScale(gray_img, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    
    for x, y, w, h in traffic_lights:
        roi = frame[y:y+h, x:x+w]
        
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # Define color ranges for red, green, and orange
    # Lower red range
    red_lower1 = np.array([0, 160, 160])
    red_upper1 = np.array([5, 255, 255])

    # Upper red range
    red_lower2 = np.array([175, 160, 160])
    red_upper2 = np.array([180, 255, 255])

    # Yellow range
    yellow_lower, yellow_upper = np.array([15, 100, 100]), np.array([35, 255, 255])

    # Green range
    green_lower = np.array([36, 80, 80])
    green_upper = np.array([85, 255, 255])

    # Create masks for each color
    mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    red_mask = cv2.bitwise_or(mask1, mask2)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    # Check for the presence of each color
    if cv2.countNonZero(red_mask) > 0:
        return 'red'
    elif cv2.countNonZero(yellow_mask) > 0:
        return 'yellow'
    elif cv2.countNonZero(green_mask) > 0:
        return 'green'
    return

def ultrasound_reading():
    GPIO.output(TRIG, True)
    sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        start_time = time.time()
    while GPIO.input(ECHO) == 1:
        end_time = time.time()

    duration = end_time - start_time
    distance = (duration * 34300) / 2
    return distance



def algo():
    rate = rospy.Rate(200) # ~200hz
    waypoint_po_x = 0
    waypoint_po_y = 0
    sensor_reset_key_on_flag = False
    traj_shape = ""
    counter = 0

    # Controller main loop, this is where the rover is controlled from
    while not rospy.is_shutdown():

        if rcin_msg.ch6 > 1000: # SF Key is UP (Means: start running in autonomous mode)

            # MP4-TODO:
            # Update flags detecting obstacles and stop signs/traffics light every few seconds
            # e.g.
            # if its__400_th_iteration: stop_flag = run_ultrasonic/traffioc_light
            if counter % 400 == 0:
                cap = cv2.VideoCapture(0)
                ret, frame = cap.read()
                if not ret:
                    print("ret failed")
                test_traffic_light = True
                test_stop_sign = False
                test_object_detection = False
                # putting code here to only get called once to save resources
                # if test_stop_sign:
                    
                if test_traffic_light:
                    if stop_sign_detected(frame):
                        # If stop sign is detected, we stop the rover
                        speed_pub.publish(map(0,-1000,1000,100,-100))
                        rospy.loginfo("Stop sign detected: Stopping rover")
                        sleep(3) # Sleep for 3 seconds to simulate stopping at the stop sign
                    # Capture for traffic light detection
                    frame = cv2.resize(frame, (640, 480))
                    color = detect_traffic_light_color(frame)
                    while color == 'red':
                        ret, frame = cap.read()
                        if not ret:
                            print("no ret")
                        frame = cv2.resize(frame, (640, 480))
                        color = detect_traffic_light_color(frame)
                        # Loop every 0.2 seconds until the traffic light is green
                        speed_pub.publish(map(0,-1000,1000,100,-100))
                        rospy.loginfo(f"{color} traffic light detected: Stopping rover")
                        sleep(0.2)
                    rospy.loginfo(f"{color} traffic light detected: Starting rover")
                # object detection code
                if test_object_detection:
                    # if we have an object closer than 100 cm, stop and wait for object to move
                    object_detected = ultrasound_reading()
                    if object_detected < 100:
                        rospy.loginfo(f"object detected {object_detected} cm away: Stopping rover")
                        while object_detected < 100:
                            rospy.loginfo(f"object detected {object_detected} cm away: Stopping rover")
                            speed_pid_value = 0
                            speed_pub.publish(map(0,-1000,1000,100,-100))
                            object_detected = ultrasound_reading()
                            sleep(0.2)
                    rospy.loginfo(f"object no longer detected: starting rover")
                cap.release()
                cv2.destroyAllWindows()
            counter += 1

            # MP4-TODO: You can add a flag to check for the flag and skip the iteration if it's true


            # The two variables below are used to figure out the angle and distance between the waypoint and the rover's position
            # This helps the PID to figure out how to steer the rover, you'll need to figure out the angle and distance

            # MP4-TODO: Find out the angle between the waypoint and the rover's position
            # You're given : (waypoint_po_x, waypoint_po_y) & (sensor_pos_msg.position.x, sensor_pos_msg.position.z)
            theta = 0 # Figure it out and reaplce the zero with the angle's formula

            diffx = waypoint_po_x - sensor_pos_msg.position.x
            diffy = waypoint_po_y - sensor_pos_msg.position.z

            theta = math.atan2(diffy, diffx) * (180/math.pi)



            # MP4-TODO: Find out the distance between the waypoint and the rover's position
            # You're given : (waypoint_po_x, waypoint_po_y) & (sensor_pos_msg.position.x, sensor_pos_msg.position.z)
            dist = 0 # Figure it out and reaplce the zero with the distance's formula

            diffx = waypoint_po_x - sensor_pos_msg.position.x
            diffy = waypoint_po_y - sensor_pos_msg.position.z

            dist = math.sqrt((diffx)**2 + (diffy)**2)


            # These 3 blocks below check the SG-Key state and set the waypoint file
            # You can't switch waypoint in midst of following another trajectory (way-point) as each waypoint file
            # should and does assume the starting point to be 0,0,0.
            # and while being in the midst of following another trajectory, you're not on 0,0,0
            # so make sure that you restart ROS (the roslaunch thingy) if you wanna switch the trajectory
            # Also after starting the ROS, don't move the rover because then the intial point won't be 0,0,0
            if rcin_msg.ch8 > 2000:
                if traj_shape != 'square':
                    traj_shape = 'square'
                    dist = 0
                    rospy.loginfo("Trajectory shape set to: Square")
            elif rcin_msg.ch8 < 1000:
                if traj_shape != 'eight':
                    traj_shape = 'eight'
                    dist = 0
                    rospy.loginfo("Trajectory shape set to: Figure eight")
            else:
                if traj_shape != 'MP':
                    traj_shape = 'MP'
                    dist = 0
                    rospy.loginfo("Trajectory shape set to: MP")

            if dist < 0.25: # If we're around 20cm away from the desired postion, we fetch the next weaypoint
                # MP4-TODO: Play with the parameter above to to tune the the algorithm

                # Get next waypoint
                resp = get_waypoint(traj_shape)
                waypoint_po_x = resp.x
                waypoint_po_y = resp.y
                print('New point recived:',waypoint_po_x,waypoint_po_y)


            (roll, pitch, yaw) = euler_from_quaternion (
                [
                    sensor_pos_msg.orientation.x,
                    sensor_pos_msg.orientation.y,
                    sensor_pos_msg.orientation.z,
                    sensor_pos_msg.orientation.w
                ]
            )

            # Note from intel IMU's prespective,
            #   -> sensor_vel_msg.linear.Z is x (Forwards)
            #   -> sensor_vel_msg.linear.X is y (side)
            #   -> sensor_vel_msg.linear.Y is z (up/down) (irrelevent)
            #   -> pitch is actually yaw (the turning ange)
            #   -> yaw is actually roll
            #   -> roll is actually pitch (irrelevent)


            # Here we're trying to figure out the speed of the rover in x direction
            # It's speed in x direction depends on both x and y and you should multiply both with proper trig ratios to account for the components
            # The variables you should work with are: sensor_vel_msg.linear.z, pitch & sensor_vel_msg.linear.x, pitch
            # Make sure to use the absolute value when adding speeds
            # x_speed=  0 # // Figure it out and replace zero with equation

            x_speed = abs(sensor_vel_msg.linear.z * math.cos(pitch) + sensor_vel_msg.linear.x * math.sin(pitch))

            pitch = -pitch * (180/math.pi)
            yaw = yaw * (180/math.pi)

            if abs(yaw) > 100:pitch = (180 - abs(pitch)) * abs(pitch)/pitch

            print(
                "Pos-Z(x): {:3.1f}, ".format(sensor_pos_msg.position.z),
                "Pos-X(y): {:3.1f}, ".format(-sensor_pos_msg.position.x),
                "Theta: {:3.1f}, ".format(theta),
                "Pitch: {:3.1f}, ".format(pitch),
                end='\r'
            )

            # The neg variable is used to determine the direction of the steering angle
            if abs(pitch) > 90 and abs(theta) > 90 and pitch*theta < 0: neg = -1
            else: neg = 1

            # This pid is used to keep track of the distance, it tries to minimise the distance 
            # between the rover and the waypoint
            # All PIDs are instances of the same PID class but control different params and have different gains
            # The PID class is defined in PID.py (You'll need to populate the class algorithm as well)
            # The first arg is the desired position (0) and the second arg is the current position (dist)
            linear_pos_pid_value = linear_pos_pid.update_pid(0,dist)

            # If there's distance to cover, we update the speed pid
            # Setting the speed PID is is a bit triky, note that the current position is 0.4 and the desired position is changing
            # Think of why that is and what that accomplishes?
            # Also, play with the 0.4 value; (you need to write PID code first)
            # If the rover goes too fast, it might not be able to turn in time and will overshoot the waypoint
            if linear_pos_pid_value >= 0:
                    speed_pid_value = speed_pid.update_pid(x_speed,0.15)

            # Update the pid value for the steering angle and see what it decides to do next
            steer_pid_value = steer_pid.update_pid(pitch,theta)

            # Publish the steering angle (bounded by some values) (this physically changes the steering angle)
            # The motor's PWM controller only expects a certain range of values
            # The negative sign accounts for the direction of steering angle
            steer_pub.publish(map(neg * steer_pid_value,-1000,1000,-100,100))

            # Publish the speed (bounded by some values) (this physically changes the speed)
            # The motor's PWM controller only expects a certain range of values
            speed_pub.publish(map(speed_pid_value,-1000,1000,100,-100))

        else: # SF Key is DOWN (Means: drone is working in manual mode)
            print( "Pos-Z(x): {:3.1f}, ".format(sensor_pos_msg.position.z) , "Pos-X(y): {:3.1f}, ".format(-sensor_pos_msg.position.x),  end="\r")
            steer_pub.publish(map(rcin_msg.ch1,800,2100,-100,100))
            speed_pub.publish(map(rcin_msg.ch2,800,2100,100,-100))

        rate.sleep()



if __name__ == '__main__':
    try:
        rcin_msg = RCIN()
        pid_param_msg = PID()
        sensor_pos_msg = Pose()
        sensor_vel_msg = Twist()

        rospy.init_node('controller', anonymous=True)

        # All the topics controller subscribes to
        # Arguments are:
        # Topic name (Format: "[node_name]/[topic_name]")
        # Data class (type of the message)
        # Callback function (the function that will be called when the data is received, it'll update the object asyncronously)
        rospy.Subscriber("controller/pid_params", PID, pid_param_callback)
        rospy.Subscriber("rcinput/data", RCIN, rcin_callback)
        rospy.Subscriber("realsense/pose", Pose, t265_position_callback)
        rospy.Subscriber("realsense/velocity", Twist, t265_velocity_callback)

        # All the topics it publishes (Just speed and steering angle)
        # The controller node is pulishing the steering angle and speed to the controller node
        # to which the rcout node is subscribed to (See def listener(): in rcout.py)
        steer_pub = rospy.Publisher("controller/steer", Float32, queue_size=10)
        speed_pub = rospy.Publisher("controller/speed", Float32, queue_size=10)

        # // I'm planning to use this in future, dw abt it for now (irrelavent for MP4)
        sensor_reset_pub = rospy.Publisher("realsense/reset", String, queue_size=10)

        rospy.wait_for_service('trajectory/next_waypoint')
        get_waypoint = rospy.ServiceProxy('trajectory/next_waypoint', WayPoint)

        # Initializing and setting the gains for steering PID
        # All PIDs are instances of the same PID class but control different params and have different gains
        steer_pid = pid(50,0.01,5)
        steer_pid.set_pid_limit(1000)
        steer_pid.set_I_limit(100)

        # Initializing and setting the gains for PID for the linear position
        # All PIDs are instances of the same PID class but control different params and have different gains
        linear_pos_pid = pid(1.0,0,0.1)
        linear_pos_pid.set_pid_limit(0.4)

        # Initializing and setting the gains for PID for the speed
        # All PIDs are instances of the same PID class but control different params and have different gains
        speed_pid = pid(200,0,1500,offset=200)
        speed_pid.set_pid_limit(1000)
        speed_pid.set_I_limit(100)

        # Run the main loop when exerything is setup
        algo()
    except rospy.ROSInterruptException:
        pass