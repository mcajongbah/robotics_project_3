import math

import cv2
import numpy as np


def vec2hat(x):
    return np.array([[  0,     -x[2][0],   x[1][0]],
                     [ x[2][0],     0,     -x[0][0]],
                     [-x[1][0],  x[0][0],   0]])

def cvdata2transmtx(rvec,tvec):
    # Find the transformation matrix gab (B is the camera;  A is the marker)
    R_temp = cv2.Rodrigues(rvec)[0]
    p_temp = tvec.reshape(-1,1)
    R = R_temp.T
    p = -R.dot(p_temp)
    g = np.vstack((np.hstack((R,p)), [0, 0, 0 ,1]))
    return g, R, p

# def cvdata2transmtx2(rvec,tvec):
#     # Find the transformation matrix gba
#     R = cv2.Rodrigues(rvec)[0]
#     p = tvec.reshape(-1,1)
#     g = np.vstack((np.hstack((R,p)), [0, 0, 0 ,1]))
#     return g, R, p


def cvdata2transmtx2(rvec, tvec):
    """
    Build a 4×4 homogeneous transform from the CAMERA frame into the marker frame,
    directly computing the inverse of the marker→camera pose.
    """
    # 1. Convert Rodrigues vector to rotation matrix R (marker→camera)
    R, _ = cv2.Rodrigues(rvec)
    # 2. Build the inverse rotation (camera→marker)
    R_inv = R.T
    # 3. Compute the inverse translation
    p = tvec.reshape(-1, 1)
    # p is the translation from the camera to the marker, so we need to invert it
    p_inv = -R_inv.dot(p)
    # 4. Assemble into a 4×4 homogeneous matrix
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_inv
    T[:3,  3] = p_inv
    return T

def transmtx2twist(g):
    R = g[0:3,0:3]
    p = g[0:3,3]
    
    rot_exp_coord = cv2.Rodrigues(R)[0]
    th = np.linalg.norm(rot_exp_coord)
    w = rot_exp_coord/th
    v = np.linalg.inv((np.identity(3)-R).dot(vec2hat(w))+w.dot(w.T)*th).dot(p).reshape(-1,1)
    return v, w, th

def twist2screw(v,w,th):
    q = np.cross(w.reshape(-1),v.reshape(-1)).reshape(-1,1)
    h = w.T.dot(v)
    u = w
    M = th
    return q, h, u, M

def distance(xdiff,zdiff):
    dist = math.sqrt(xdiff**2 + zdiff**2)
    return dist



print("Test transmtx2twist function:")
gMatrix = np.array( [ [  0.5555,  0.5274,  0.6429,  6.0000 ],
                      [ -0.3906,  0.8480, -0.3581,  1.4015 ],
                      [ -0.7341, -0.0522,  0.6771, -1.3978 ],
                      [       0,       0,       0,  1.0000 ] ] )

v, w, th = transmtx2twist(gMatrix)

print("test",w.dot(w.T))

print("  v = " + str(v))
print("  w = " + str(w))
print("  th = " + str(th))


