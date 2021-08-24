import numpy as np
import cv2
import glob

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(11,8,0)
objp = np.zeros((8*11,3), np.float32)
objp[:,:2] = np.mgrid[0:11, 0:8].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d points in real world space
imgpoints = [] # 2d points in image plane.

# Make a list of calibration images
images = glob.glob('calib_image/*.jpg')

# Step through the list and search for chessboard corners
for idx, fname in enumerate(images):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chessboard corners 11*8
    ret, corners = cv2.findChessboardCorners(gray, (11,8), None)
    # If found, add object points, image points
    if ret == True:
        objpoints.append(objp)
        imgpoints.append(corners)
        # Draw and display the corners 11*8
        cv2.drawChessboardCorners(img, (11,8), corners, ret)
        cv2.imshow('img', img)
        cv2.waitKey(100)

# Test undistortion on an image
img = cv2.imread('./calib_image/26.jpg')
cv2.imshow('img', img)
img_size = (img.shape[1], img.shape[0])

# Do camera calibration given object points and image points
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size,None,None)

print("mtx")
print(mtx)

print("dist")
print(dist)

dst = cv2.undistort(img, mtx, dist, None, mtx)
cv2.imwrite('./result_undist.jpg',dst)
cv2.imshow('result_undist', dst)

cv2.waitKey(5000)
cv2.destroyAllWindows()
