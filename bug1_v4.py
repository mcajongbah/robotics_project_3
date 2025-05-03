# -*- coding: utf-8 -*-
import math
import pickle
import time

import cv2
import numpy as np
import yaml
from picamera2 import Picamera2
from picarx import Picarx

import utils

# Initialize the Picarx robot instance.
px = Picarx()

# Set the grayscale reference value for your sensor calibration.
# Modify this value for an optimal result.
px.set_grayscale_reference(0)

# -------------------------------
# Global Calibration Constants
# -------------------------------
# IR sensor calibration constants:
GRAYSCALE_COEF = 1000      # Coefficient used for scaling sensor indices (i.e., sensor spacing)
IR_THRESHOLD_HIGH = 200    # Threshold to detect if the robot is "on" the line (adjust as needed)
IR_THRESHOLD_LOW = 50      # Noise threshold below which sensor reading is ignored

# Controller constant for line following using IR sensor.
# Adjust these values based on your calibration.
PID_DIVISOR = 25           # Divisor for the proportional term
DERIVATIVE_DIVISOR = 100   # Divisor for the derivative term

# -------------------------------
# Utility Classes and Functions
# -------------------------------
class Point(object):
    """
    Represents a point in 2D space.
    Attributes:
        x (float): The x-coordinate.
        z (float): The z-coordinate.
    """
    def __init__(self):
        self.x = None
        self.z = None

def stop(px):
    """
    Stops the robot by setting both motor speeds to zero.
    
    Args:
        px (Picarx): Instance controlling the robot.
    """
    px.set_motor_speed(1, 0)
    px.set_motor_speed(2, 0)

# Define additional movement functions, e.g.:
# - turn(direction, angle)
# - move_left()
# - move_right()
# - move_forward()
# - move_backward()
# These methods should control the robot's motors accordingly.

def turn(px, direction, angle):
    """
    Turns the robot in the specified direction by the given angle.
    
    Args:
        px (Picarx): Instance controlling the robot.
        direction (str): Direction to turn ('left' or 'right').
        angle (float): Angle to turn in degrees.
    """
    if direction == 'left':
        px.set_dir_servo_angle(angle)
        px.forward(30)
    elif direction == 'right':
        px.set_dir_servo_angle(-angle)
        px.forward(30)

def move_left(px, speed=30):
    """
    Moves the robot to the left.
    
    Args:
        px (Picarx): Instance controlling the robot.
        speed (int): Motor speed.
    """
    px.set_dir_servo_angle(30)
    px.forward(speed)

def move_right(px, speed=30):
    """
    Moves the robot to the right.
    
    Args:
        px (Picarx): Instance controlling the robot.
        speed (int): Motor speed.
    """
    px.set_dir_servo_angle(-30)
    px.forward(speed)

def move_forward(px, speed=30):
    """
    Moves the robot forward.
    
    Args:
        px (Picarx): Instance controlling the robot.
        speed (int): Motor speed.
    """
    px.set_dir_servo_angle(0)
    px.forward(speed)

def move_backward(px, speed=30):
    """
    Moves the robot backward.
    
    Args:
        px (Picarx): Instance controlling the robot.
        speed (int): Motor speed.
    """
    px.set_dir_servo_angle(0)
    px.backward(speed)

def readLine(white_line=0):
    """
    Reads the infrared (IR) line sensor values and calculates an estimated position
    of the detected line based on a weighted average of sensor readings.
    
    Returns:
        float: A position value where 0 indicates the line is directly below sensor 0, 
               1000 indicates the line is below sensor 1, and 2000 indicates the line is 
               below sensor 2. Intermediate values indicate the line is between sensors.
               
    Formula:
        (0*value0 + GRAYSCALE_COEF*value1 + 2*GRAYSCALE_COEF*value2 + ...)
        ---------------------------------------------------------------
          (value0 + value1 + value2 + ...)
    """
    sensor_values = px.get_grayscale_data()  # Get IR sensor readings
    weighted_sum = 0       # Numerator of weighted average
    total = 0              # Denominator sum of sensor values
    on_line = 0            # Flag indicating if the line is detected
    numSensors = 3         # Number of sensors in use
    last_value = 0
    
    for i in range(numSensors):
        value = sensor_values[i]

        # Check if this sensor value exceeds the high-threshold.
        if value > IR_THRESHOLD_HIGH:
            on_line = 1

        # Use only values above the noise threshold.
        if value > IR_THRESHOLD_LOW:
            weighted_sum += value * (i * GRAYSCALE_COEF)
            total += value

    if on_line != 1:
        # If no sensor sees the line, use the last estimated value to determine the side.
        if last_value < (numSensors - 1) * GRAYSCALE_COEF / 2:
            return 0  # Line is to the left.
        else:
            return (numSensors - 1) * GRAYSCALE_COEF  # Line is to the right.

    last_value = weighted_sum / total
    return last_value

def go_to_goal(px, cam, goal_id, goal, hit, deg_eps, dist_eps, last_proportional, angle_to_goal):
    """
    Moves the robot toward a goal using both IR line sensing and ArUco marker detection.
    
    Args:
        px (Picarx): Robot control instance.
        cam (Picamera2): Picamera2 instance for capturing frames.
        goal_id (int): ID of the goal marker.
        goal (Point): Goal point data.
        hit (Point): Point where the line is hit.
        deg_eps (float): Angular epsilon tolerance.
        dist_eps (float): Distance epsilon tolerance.
        last_proportional (float): Previous proportional error from line following.
        angle_to_goal (float): Angle toward the goal, if already determined.
        
    Returns:
        Tuple: (state, goal, hit, last_proportional, angle_to_goal)
    """
    print("Warming up the line track sensors...")
    time.sleep(0.5)
    
    # Calibration parameters for go_to_goal.
    coef = 2000      # Constant used for control computations.
    threshold = 500  # Derivative threshold (adjust as needed).
    
    # Warm-up loop to stabilize line sensor baseline.
    for i in range(100):
        position = readLine()
        proportional = position - coef  
        last_proportional = proportional
    time.sleep(0.5)
    print("Finished warmup")
    
    while True:
        # Capture a frame using Picamera2.
        frame = cam.capture_array()
        # Convert from BGRA (default) to BGR.
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        
        position = readLine()
        proportional = position - coef
        derivative = proportional - last_proportional
        last_proportional = proportional
        
        # Process the frame for ArUco marker detection.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        
        # Initialize p_gc for potential line hit detection
        p_gc = None
        
        if corners is not None and len(corners) != 0:
            # Estimate marker pose.
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            for i, _ in enumerate(rvecs):
                if ids[i] == goal_id:
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_gc = utils.cvdata2transmtx(rvecs[i], tvecs[i])[0]
                    p_gc = g_gc[:, 3]  # Extract translation vector.
                    th = utils.transmtx2twist(g_gc)[2]  # Marker orientation.
                    
                    # Set initial goal point if not already set.
                    if angle_to_goal is None:
                        goal.z = 0.1  # Example distance from the marker.
                        angle_to_goal = math.atan(p_gc[0] / p_gc[2])
                        goal.x = goal.z * math.tan(angle_to_goal)
                        print("Set Goal Point: x:{}  z:{}".format(goal.x, goal.z))
                    
                    # Calculate error relative to the goal.
                    xdiff = p_gc[0] - goal.x
                    zdiff = p_gc[2] - goal.z
                    cur_dist = utils.distance(xdiff, zdiff)
                    
                    if cur_dist <= dist_eps:
                        print("Reached goal point!")
                        cv2.destroyAllWindows()
                        stop(px)
                        state = 3
                        return state, goal, hit, last_proportional, angle_to_goal
                    
                    # Implement robot movements based on conditions
                    if zdiff < 0:
                        move_backward(px)
                    else:
                        move_forward(px)
                    
                    # Check for line hit after we have p_gc
                    if abs(derivative) >= threshold:
                        state = 1
                        print("Hit line detected!")
                        print("Running find_leave")
                        hit.x = p_gc[0]
                        hit.z = p_gc[2]
                        
                        # Insert code for a turning maneuver.
                        turn(px, 'left', 90)
                        
                        return state, goal, hit, last_proportional, angle_to_goal
                else:
                    # handle detection of other markers.
                    pass
        else:
            # If no markers detected, still check for line hit
            if abs(derivative) >= threshold and p_gc is not None:
                state = 1
                print("Hit line detected!")
                print("Running find_leave")
                hit.x = p_gc[0]
                hit.z = p_gc[2]
                
                # Insert code for a turning maneuver.
                turn(px, 'left', 90)
                
                return state, goal, hit, last_proportional, angle_to_goal
            
            # robot does not detect the line on the ground or an ArUco marker
            # it checks if it has previously detected an ArUco marker. 
            # If it has, it will continue moving towards the target. 
            #
            # determine which movement method(s) the robot should use 
            # to move toward the target based on its current position relative to the target.
            # =====================
            if 'th' in locals():
                # Use the current heading compared to the goal angle
                if th - angle_to_goal > 0:
                    # Insert code to adjust robot's direction (e.g., slight left turn).
                    turn(px, 'left', 10)
                else:
                    # Insert code to adjust robot's direction (e.g., slight right turn).
                    turn(px, 'right', 10)
            else:
                # No orientation data available; move forward to explore
                move_forward(px, 25)
                print("No markers detected, moving forward to explore")
        # Display the annotated frame (for debugging).
        # cv2.imshow('aruco', frame)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            stop(px)
            state = 3
            return state, goal, hit, last_proportional, angle_to_goal

def find_leave(px, cam, goal_id, helper1_id, helper2_id, goal, hit, leave, dist_eps, g_gh1, g_gh2, last_proportional):
    """
    Searches for an optimal "leave" point along the path after hitting the line.
    While following the line, this function uses proportional control to steer the robot
    and updates the leave point based on proximity to the goal and previous hit point.
    
    Args:
    px (Picarx): Robot instance.
    cam (Picamera2): Picamera2 instance for capturing frames.
    goal_id (int): Marker id for the goal.
    helper1_id (int): Marker id for the first helper marker.
    helper2_id (int): Marker id for the second helper marker.
    goal (Point): Goal point.
    hit (Point): The point where the line was hit.
    leave (Point): The current best leave point.
    dist_eps (float): Distance tolerance.
    g_gh1 (np.ndarray): Calibration matrix for helper marker 1.
    g_gh2 (np.ndarray): Calibration matrix for helper marker 2.
    last_proportional (float): Previous proportional error.

    Returns:
        Tuple: (state, leave, last_proportional)
    """
    maximum = 35
    count = 0

    while True:
        # Read the line sensor position.
        position = readLine()
        # Calculate the proportional error assuming the desired position is centered at 1000.
        # Note: Changed from 2000 to 1000 to center the line better
        proportional = position - 1000
        
        # Calculate the change in error (derivative).
        derivative = proportional - last_proportional
        last_proportional = proportional
        
        # Compute power difference using a simple PD controller.
        power_difference = proportional / PID_DIVISOR + derivative / DERIVATIVE_DIVISOR
        if power_difference > maximum:
            power_difference = maximum
        if power_difference < -maximum:
            power_difference = -maximum
        
        # Print sensor values for debugging
        sensor_values = px.get_grayscale_data()
        print("sensor values", sensor_values)
        
        # Adjust motor speeds based on computed power difference for line following
        # Instead of just moving backward, we'll follow the line by adjusting left/right
        if power_difference < 0:
            # Line is to the left, turn left while moving
            turn(px, 'left', abs(power_difference)/5)  # Scale down the turning angle
        else:
            # Line is to the right, turn right while moving
            turn(px, 'right', abs(power_difference)/5)  # Scale down the turning angle
        
        time.sleep(0.05)
        stop(px)

        
        # Capture a frame from Picamera2.
        frame = cam.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        
        # Compute the current distance between the goal and the candidate leave point.
        gl_dist = utils.distance(goal.x - leave.x, goal.z - leave.z)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        
        if corners is not None and len(corners) != 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            for i, _ in enumerate(rvecs):
                if ids[i] == goal_id:
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_gc = utils.cvdata2transmtx(rvecs[i], tvecs[i])[0]
                    p_gc = g_gc[:, 3]
                    cur_dist_goal = utils.distance(p_gc[0] - goal.x, p_gc[2] - goal.z)
                    cur_dist_hit = utils.distance(p_gc[0] - hit.x, p_gc[2] - hit.z)
                    if cur_dist_goal <= gl_dist:
                        gl_dist = cur_dist_goal
                        leave.x = p_gc[0]
                        leave.z = p_gc[2]
                        count += 1
                    # Once the robot has been following the line for a sufficient duration,
                    # if the current distance to the hit point is below threshold, use the leave point. 
                    if cur_dist_hit <= dist_eps and count > 100:
                        stop(px)
                        print("Finished tracking; now going to the leave point")
                        state = 2
                        return state, leave, last_proportional
                elif ids[i] == helper1_id:
                    # Process helper1 marker for additional localization
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_ch1 = utils.cvdata2transmtx2(rvecs[i], tvecs[i])[0]
                    # Calculate position relative to helper1
                    p_h1c = g_ch1[:, 3]
                    # Use the helper1 marker to refine the robot's position estimate
                    robot_pos_from_h1 = np.dot(g_gh1, p_h1c)
                    print("Position from Helper1: x:{}, z:{}".format(robot_pos_from_h1[0], robot_pos_from_h1[2]))
                elif ids[i] == helper2_id:
                    # Process helper2 marker for additional localization
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_ch2 = utils.cvdata2transmtx2(rvecs[i], tvecs[i])[0]
                    # Calculate position relative to helper2
                    p_h2c = g_ch2[:, 3]
                    # Use the helper2 marker to refine the robot's position estimate
                    robot_pos_from_h2 = np.dot(g_gh2, p_h2c)
                    print("Position from Helper2: x:{}, z:{}".format(robot_pos_from_h2[0], robot_pos_from_h2[2]))
        # cv2.imshow('aruco', frame)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            stop(px)
            state = 3
            return state, leave, last_proportional

def go_to_leave(px, cam, goal_id, helper1_id, helper2_id, goal, leave, dist_eps, g_gh1, g_gh2, last_proportional):
    """
    Guides the robot from the leave point back toward the goal.
    Uses IR sensor data for line following and ArUco marker detection for pose estimation.
    
    Args:
        px (Picarx): Robot control instance.
        cam (Picamera2): Picamera2 instance for capturing frames.
        goal_id (int): ID of the goal marker.
        helper1_id (int): ID for helper marker 1.
        helper2_id (int): ID for helper marker 2.
        goal (Point): Goal point.
        leave (Point): Leave point.
        dist_eps (float): Distance epsilon tolerance.
        g_gh1 (np.ndarray): Calibration data for helper marker 1.
        g_gh2 (np.ndarray): Calibration data for helper marker 2.
        last_proportional (float): Last proportional error value.
    
    Returns:
        int: The new state (e.g., 0 for continuing toward goal, 3 for termination, etc.)
    """
    maximum = 35

    while True:
        # Insert command to move the robot backward (e.g., move_backward(px)).
        move_backward(px)
        
        coef = 2000  # Calibration constant.
        
        # Read line
        position = readLine()
        
        proportional = position - coef
        derivative = proportional - last_proportional
        last_proportional = proportional
        power_difference = proportional / PID_DIVISOR + derivative / DERIVATIVE_DIVISOR

        if power_difference > maximum:
            power_difference = maximum
        if power_difference < -maximum:
            power_difference = -maximum
                    
        if power_difference < 0:
            px.set_motor_speed(1, maximum + power_difference)
            px.set_motor_speed(2, maximum)
        else:
            px.set_motor_speed(1, maximum)
            px.set_motor_speed(2, maximum - power_difference)
            
        time.sleep(0.05)
        stop(px)
        
        # Capture frame from Picamera2.
        frame = cam.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if corners is not None and len(corners) != 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            for i, _ in enumerate(rvecs):
                if ids[i] == goal_id:
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_gc = utils.cvdata2transmtx(rvecs[i], tvecs[i])[0]
                    p_gc = g_gc[:, 3]
                    cur_dist_leave = utils.distance(p_gc[0] - leave.x, p_gc[2] - leave.z)
                    if cur_dist_leave <= dist_eps:
                        print("Reached leave point, resuming approach to goal.")
                        # Move forward a few steps after reaching the leave point.
                        for i in range(5):
                            # Insert call to your forward movement function (e.g., move_forward(px))
                            move_forward(px)
                        state = 0
                        return state
                elif ids[i] == helper1_id:
                    # Process helper1 marker for additional localization
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_ch1 = utils.cvdata2transmtx2(rvecs[i], tvecs[i])[0]
                    # Calculate position relative to helper1
                    p_h1c = g_ch1[:, 3]
                    # Use the helper1 marker to refine the robot's position estimate
                    robot_pos_from_h1 = np.dot(g_gh1, p_h1c)
                    print("Position from Helper1: x:{}, z:{}".format(robot_pos_from_h1[0], robot_pos_from_h1[2]))
                elif ids[i] == helper2_id:
                    # Process helper2 marker for additional localization
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_ch2 = utils.cvdata2transmtx2(rvecs[i], tvecs[i])[0]
                    # Calculate position relative to helper2
                    p_h2c = g_ch2[:, 3]
                    # Use the helper2 marker to refine the robot's position estimate
                    robot_pos_from_h2 = np.dot(g_gh2, p_h2c)
                    print("Position from Helper2: x:{}, z:{}".format(robot_pos_from_h2[0], robot_pos_from_h2[2]))
        # cv2.imshow('aruco', frame)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            stop(px)
            state = 3
            return state

# -------------------------------
# Main Execution Block
# -------------------------------
if __name__ == "__main__":
    # Variable to track the proportional error for the line following algorithm.
    last_proportional = 0

    # Initialize ArUco marker detection parameters.
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
    aruco_params = cv2.aruco.DetectorParameters_create()
    aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

    # Marker side length in meters.
    marker_length = 0.05

    # Initialize key points.
    goal = Point()
    hit = Point()
    leave = Point()
    
    angle_to_goal = None

    # Set an initially very distant leave point.
    leave.x = 1000000
    leave.z = 1000000

    # Load camera calibration parameters from YAML.
    with open('calib_data_bug1.yaml') as file:
        calib_data = yaml.load(file, Loader=yaml.FullLoader)
    mtx = np.asarray(calib_data["camera_matrix"])
    dist = np.asarray(calib_data["distortion_coefficients"])
    g_gh1 = np.asarray(calib_data["g_gh1"])
    g_gh2 = np.asarray(calib_data["g_gh2"])

    # Initialize Picamera2 instead of cv2.VideoCapture.
    cam = Picamera2()
    config = cam.create_preview_configuration(main={"size": (640, 480)})
    cam.configure(config)
    cam.start()

    # Define marker IDs.
    goal_id = 2
    helper1_id = 1
    helper2_id = 0

    # Define tolerances.
    deg_eps = 0.1
    dist_eps = 0.2  

    # Initial state.
    state = 0

    try:
        # Main loop: cycle through different states of the algorithm.
        while True:
            if state == 0:
                state, goal, hit, last_proportional, angle_to_goal = go_to_goal(
                    px, cam, goal_id, goal, hit, deg_eps, dist_eps, last_proportional, angle_to_goal
                )
            elif state == 1:
                state, leave, last_proportional = find_leave(
                    px, cam, goal_id, helper1_id, helper2_id, goal, hit, leave, dist_eps, g_gh1, g_gh2, last_proportional
                )
            elif state == 2:
                state = go_to_leave(
                    px, cam, goal_id, helper1_id, helper2_id, goal, leave, dist_eps, g_gh1, g_gh2, last_proportional
                )
            elif state == 3:
                print("Final Points:")
                print("Goal point: x: {}, z: {}".format(goal.x, goal.z))
                print("Hit point: x: {}, z: {}".format(hit.x, hit.z))
                print("Leave point: x: {}, z: {}".format(leave.x, leave.z))
                print("Finished Bug 1!")
                break

    except KeyboardInterrupt:
        cam.stop()
        cv2.destroyAllWindows()
        stop(px)
