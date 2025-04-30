import cv2
import numpy as np
import yaml
import utils
import math
from picamera2 import Picamera2

# -------------------------------
# ArUco Marker and Calibration Setup
# -------------------------------

# Retrieve the predefined ArUco dictionary and create detector parameters.
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters_create()
aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

# Side length of the ArUco marker in meters.
marker_length = 0.05

# Load calibration parameters from the YAML file.
with open(r'calib_data.yaml') as file:
    calib_data = yaml.load(file, Loader=yaml.FullLoader)
mtx = np.asarray(calib_data["camera_matrix"])
dist = np.asarray(calib_data["distortion_coefficients"])

# -------------------------------
# Camera Initialization with Picamera2
# -------------------------------
cam = Picamera2()
config = cam.create_preview_configuration(main={"size": (640, 480)})
cam.configure(config)
cam.start()

# Marker IDs.
goal_id = 2
helper1_id = 1
helper2_id = 0

# -------------------------------
# First Calibration Loop:
# Capture transformation between Goal and Helper 1.
# -------------------------------
print("Press q to save the transformation between Goal and Helper 1")
while True:
    # Capture a frame from Picamera2.
    frame = cam.capture_array()
    # Convert BGRA (default format) to BGR.
    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    
    # Convert the frame to grayscale.
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect ArUco markers.
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
    if corners is not None and len(corners) != 0:
        # Estimate poses of the detected markers.
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
        cv2.aruco.drawDetectedMarkers(frame, corners)
        for i, _ in enumerate(rvecs):
            if ids[i] == goal_id:
                cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                g_gc1 = utils.cvdata2transmtx(rvecs[i], tvecs[i])[0]
            elif ids[i] == helper1_id:
                cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                g_ch1 = utils.cvdata2transmtx2(rvecs[i], tvecs[i])[0]
    
    cv2.imshow('aruco', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# -------------------------------
# Second Calibration Loop:
# Capture transformation between Goal and Helper 2.
# -------------------------------
print("Press q to save the transformation between Goal and Helper 2")
while True:
    frame = cam.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
    if corners is not None and len(corners) != 0:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
        cv2.aruco.drawDetectedMarkers(frame, corners)
        for i, _ in enumerate(rvecs):
            if ids[i] == goal_id:
                cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                g_gc2 = utils.cvdata2transmtx(rvecs[i], tvecs[i])[0]
            elif ids[i] == helper2_id:
                cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                g_ch2 = utils.cvdata2transmtx2(rvecs[i], tvecs[i])[0]
    
    cv2.imshow('aruco', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# Release resources.
cam.stop()
cv2.destroyAllWindows()

# -------------------------------
# Compute and Save Transformations
# -------------------------------
# Compute the transformation between Goal and Helper markers.
g_gh1 = g_gc1.dot(g_ch1)
print("Helper1 location x:{}, z:{}".format(g_gh1[0, 3], g_gh1[2, 3]))
g_gh2 = g_gc2.dot(g_ch2)
print("Helper2 location x:{}, z:{}".format(g_gh2[0, 3], g_gh2[2, 3]))

# Save calibration data back to YAML.
calib_data = {
    'camera_matrix': np.asarray(mtx).tolist(),
    'distortion_coefficients': np.asarray(dist).tolist(),
    'g_gh1': np.asarray(g_gh1).tolist(),
    'g_gh2': np.asarray(g_gh2).tolist()
}

with open(r'calib_data.yaml', 'w') as file:
    yaml.dump(calib_data, file)
