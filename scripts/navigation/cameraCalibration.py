#!/usr/bin/env python
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import cv2 as cv
import rospy
import cv2

def callback_img(img):
    bridge = CvBridge()

    image = bridge.imgmsg_to_cv2(img, "bgr8")
    image = cv2.flip(image, -1) # Just for temporal solution for simulations

    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    img = np.array(image)

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,6), None)
    # If found, add object points, image points (after refining them)
    print(ret)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,6), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(10000)

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("Return value: ")
    print(ret)

    print("Camera matrix: ")
    print(mtx)

    print("Distortion coefficients: ")
    print(dist)

    print("Rotation vector: ")
    print(rvecs)

    print("Translation vector: ")
    print(tvecs)


def calibration():
    rospy.init_node('calibration')
    rospy.Subscriber("/frontr200/camera/color/image_raw", Image, callback_img)
    rospy.spin()

if __name__ == '__main__':
    calibration()