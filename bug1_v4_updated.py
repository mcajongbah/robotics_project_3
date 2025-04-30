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

# ======= TODO =======
# Define additional movement functions, e.g.:
# - turn(direction, angle)
# - move_left()
# - move_right()
# - move_forward()
# - move_backward()
# These methods should control the robot's motors accordingly.
# =====================
def turn(px, direction, speed=30):
    """Spin in place. direction: 'left' or 'right'."""
    if direction.lower() == 'left':
        px.set_motor_speed(1, -speed)
        px.set_motor_speed(2,  speed)
    else:
        px.set_motor_speed(1,  speed)
        px.set_motor_speed(2, -speed)


def move_forward(px, speed=20):
    """Drive straight ahead."""
    px.set_motor_speed(1,  speed)
    px.set_motor_speed(2,  speed)


def move_backward(px, speed=20):
    """Back up in a straight line."""
    px.set_motor_speed(1, -speed)
    px.set_motor_speed(2, -speed)


def move_left(px, speed=15):
    """Drift left: slow left wheel, fast right wheel."""
    px.set_motor_speed(1, speed // 2)
    px.set_motor_speed(2, speed)


def move_right(px, speed=15):
    """Drift right: fast left wheel, slow right wheel."""
    px.set_motor_speed(1, speed)
    px.set_motor_speed(2, speed // 2)


def readLine(white_line=0):
    """
    Reads the infrared (IR) line sensor values and calculates an estimated position
    of the detected line based on a weighted average of sensor readings.
    
    Returns:
        float: A position value where 0 indicates the line is directly below sensor 0, 
               1000 indicates the line is below sensor 1, and 2000 indicates the line is 
               below sensor 2. Intermediate values indicate the line is between sensors.
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
        cam (Picamera2): Picamera2 instance.
        goal_id (int): ID of the goal ArUco marker.
        goal (Point): Goal point object.
        hit (Point): Point object where the line was hit.
        deg_eps (float): Angular tolerance.
        dist_eps (float): Distance tolerance.
        last_proportional (float): Previous proportional error.
        angle_to_goal (float): Previously calculated goal angle.
    
    Returns:
        Tuple: (state, goal, hit, last_proportional, angle_to_goal)
    """
    print("Warming up the line track sensors...")
    time.sleep(0.5)
    
    coef = 2000      # Constant used for control computations.
    threshold = 500  # Derivative threshold (adjust as needed).
    
    # Warm-up loop to stabilize line sensor baseline.
    for _ in range(100):
        position = readLine()
        proportional = position - coef
        last_proportional = proportional
    time.sleep(0.5)
    print("Finished warmup")
    
    while True:
        # Capture a frame using Picamera2.
        frame = cam.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        
        # Line-following PD control.
        position = readLine()
        proportional = position - coef
        derivative = proportional - last_proportional
        last_proportional = proportional
        
        # If a sudden change indicates hitting the line (obstacle).
        if abs(derivative) >= threshold:
            state = 1
            print("Hit line detected!")
            hit.x = goal.x
            hit.z = goal.z

            # ======= TODO =======
            # TODO: Insert code for a turning maneuver.
            stop(px)
            turn(px, 'left')
            time.sleep(0.5)
            stop(px)
            # =====================
            return state, goal, hit, last_proportional, angle_to_goal
        
        # Process frame for ArUco markers.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        
        if corners is not None and len(corners) != 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            for i in range(len(rvecs)):
                if ids[i] == goal_id:
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_gc = utils.cvdata2transmtx(rvecs[i], tvecs[i])[0]
                    p_gc = g_gc[:, 3]
                    th = utils.transmtx2twist(g_gc)[2]
                    
                    # Initialize goal point if not set.
                    if angle_to_goal is None:
                        goal.z = 0.1
                        angle_to_goal = math.atan(p_gc[0] / p_gc[2])
                        goal.x = goal.z * math.tan(angle_to_goal)
                        print("Set Goal Point: x:{}  z:{}".format(goal.x, goal.z))
                    
                    xdiff = p_gc[0] - goal.x
                    zdiff = p_gc[2] - goal.z
                    cur_dist = utils.distance(xdiff, zdiff)
                    
                    if cur_dist <= dist_eps:
                        print("Reached goal point!")
                        cv2.destroyAllWindows()
                        stop(px)
                        return 3, goal, hit, last_proportional, angle_to_goal
                    
                    # ======= TODO =======
                    # Implement robot movements (left, right, forward, etc.) based on conditions
                    if abs(th) > deg_eps:
                        turn(px, 'left' if th > 0 else 'right')
                    elif cur_dist > dist_eps:
                        move_forward(px)
                    else:
                        stop(px)
                    # =====================
                else:
                    # handle detection of other markers.
                    pass
        else:
            # ======= TO DO =======
            # robot does not detect the line on the ground or an ArUco marker
            # it checks if it has previously detected an ArUco marker. 
            # If it has, it will continue moving towards the target.
            if angle_to_goal is not None:
                # determine which movement method(s) the robot should use
                if th - angle_to_goal > 0:
                    # TODO: Insert code to adjust robot's direction (e.g., slight left turn).
                    move_left(px)
                else:
                    # TODO: Insert code to adjust robot's direction (e.g., slight right turn).
                    move_right(px)
            else:
                # No orientation data available; maintain the current course.
                move_forward(px)
            # =====================
        
        # Display the annotated frame for debugging.
        cv2.imshow('aruco', frame)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            stop(px)
            return 3, goal, hit, last_proportional, angle_to_goal


def find_leave(px, cam, goal_id, helper1_id, helper2_id, goal, hit, leave, dist_eps, g_gh1, g_gh2, last_proportional):
    """
    Searches for an optimal "leave" point along the path after hitting the line.
    """
    maximum = 35
    count = 0

    while True:
        # ======= TODO =======
        # Insert code here to move the robot backward (for example: move_backward(px)).
        move_backward(px)
        time.sleep(0.3)
        stop(px)
        # =====================
        
        # Calibration constant for find_leave control loop.
        coef = 2000
        
        # Read the line sensor position.
        position = readLine()
        proportional = position - 2000
        
        # Calculate the change in error (derivative).
        derivative = proportional - last_proportional
        last_proportional = proportional
        
        # Compute power difference using a simple PD controller.
        power_difference = proportional / PID_DIVISOR + derivative / DERIVATIVE_DIVISOR
        power_difference = max(-maximum, min(maximum, power_difference))
        
        # Adjust motor speeds based on computed power difference.
        if power_difference < 0:
            px.set_motor_speed(1, maximum + power_difference)
            px.set_motor_speed(2, maximum)
        else:
            px.set_motor_speed(1, maximum)
            px.set_motor_speed(2, maximum - power_difference)
        
        time.sleep(0.05)
        stop(px)

        # Capture a frame from Picamera2.
        frame = cam.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        
        # Compute distances to goal and hit points.
        gl_dist = utils.distance(goal.x - leave.x, goal.z - leave.z)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        
        if corners is not None and len(corners) != 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            for i in range(len(rvecs)):
                if ids[i] == goal_id:
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_gc = utils.cvdata2transmtx(rvecs[i], tvecs[i])[0]
                    p_gc = g_gc[:, 3]
                    cur_dist_goal = utils.distance(p_gc[0] - goal.x, p_gc[2] - goal.z)
                    cur_dist_hit = utils.distance(p_gc[0] - hit.x, p_gc[2] - hit.z)
                    if cur_dist_goal <= gl_dist:
                        gl_dist = cur_dist_goal
                        leave.x, leave.z = p_gc[0], p_gc[2]
                        count += 1
                    if cur_dist_hit <= dist_eps and count > 100:
                        stop(px)
                        print("Finished tracking; now going to the leave point")
                        return 2, leave, last_proportional
                elif ids[i] == helper1_id:
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    T_cam_h1 = utils.cvdata2transmtx2(rvecs[i], tvecs[i])
                    T_goal_h1 = g_gh1 @ np.linalg.inv(T_cam_h1)
                    p_h1 = T_goal_h1[:, 3]
                    d_h1 = math.hypot(p_h1[0], p_h1[2])
                    if d_h1 < gl_dist:
                        gl_dist = d_h1
                        leave.x, leave.z = p_h1[0], p_h1[2]
                elif ids[i] == helper2_id:
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    T_cam_h2 = utils.cvdata2transmtx2(rvecs[i], tvecs[i])
                    T_goal_h2 = g_gh2 @ np.linalg.inv(T_cam_h2)
                    p_h2 = T_goal_h2[:, 3]
                    d_h2 = math.hypot(p_h2[0], p_h2[2])
                    if d_h2 < gl_dist:
                        gl_dist = d_h2
                        leave.x, leave.z = p_h2[0], p_h2[2]
        cv2.imshow('aruco', frame)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            stop(px)
            return 3, leave, last_proportional


def go_to_leave(px, cam, goal_id, helper1_id, helper2_id, goal, leave, dist_eps, g_gh1, g_gh2, last_proportional):
    """
    Guides the robot from the leave point back toward the goal.
    """
    maximum = 35

    while True:
        # ======= TODO =======
        # Insert command to move the robot backward (e.g., move_backward(px)).
        move_backward(px)
        time.sleep(0.3)
        stop(px)
        # =====================
        
        coef = 2000
        position = readLine()
        proportional = position - coef
        derivative = proportional - last_proportional
        last_proportional = proportional
        power_difference = proportional / PID_DIVISOR + derivative / DERIVATIVE_DIVISOR
        power_difference = max(-maximum, min(maximum, power_difference))
                    
        if power_difference < 0:
            px.set_motor_speed(1, maximum + power_difference)
            px.set_motor_speed(2, maximum)
        else:
            px.set_motor_speed(1, maximum)
            px.set_motor_speed(2, maximum - power_difference)
        
        time.sleep(0.05)
        stop(px)
        
        frame = cam.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if corners is not None and len(corners) != 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            for i in range(len(rvecs)):
                if ids[i] == goal_id:
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_gc = utils.cvdata2transmtx(rvecs[i], tvecs[i])[0]
                    p_gc = g_gc[:, 3]
                    cur_dist_leave = utils.distance(p_gc[0] - leave.x, p_gc[2] - leave.z)
                    if cur_dist_leave <= dist_eps:
                        print("Reached leave point, resuming approach to goal.")
                        # Move forward a few steps after reaching the leave point.
                        for _ in range(5):
                            move_forward(px)
                            time.sleep(0.1)
                        stop(px)
                        return 0
                elif ids[i] == helper1_id:
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    T_cam_h1 = utils.cvdata2transmtx2(rvecs[i], tvecs[i])
                    T_goal_h1 = g_gh1 @ np.linalg.inv(T_cam_h1)
                    p_h1 = T_goal_h1[:, 3]
                    angle_leave = math.degrees(math.atan2(p_h1[0], p_h1[2]))
                    dist_leave = math.hypot(p_h1[0], p_h1[2])
                elif ids[i] == helper2_id:
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    T_cam_h2 = utils.cvdata2transmtx2(rvecs[i], tvecs[i])
                    T_goal_h2 = g_gh2 @ np.linalg.inv(T_cam_h2)
                    p_h2 = T_goal_h2[:, 3]
                    angle_leave = math.degrees(math.atan2(p_h2[0], p_h2[2]))
                    dist_leave = math.hypot(p_h2[0], p_h2[2])
        cv2.imshow('aruco', frame)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            stop(px)
            return 3

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
                print(f"Goal point: x: {goal.x}, z: {goal.z}")
                print(f"Hit point: x: {hit.x}, z: {hit.z}")
                print(f"Leave point: x: {leave.x}, z: {leave.z}")
                print("Finished Bug 1!")
                break

    except KeyboardInterrupt:
        cam.stop()
        cv2.destroyAllWindows()
        stop(px)
