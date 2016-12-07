import numpy as np
import cv2
import glob

width = 6
height = 9

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((height*width,3), np.float32)
objp[:,:2] = np.mgrid[0:width,0:height].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = ["chess.png"]

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(img, (width,height),None)
    for corner in corners:
        im = img.copy()
        cv2.circle(im, (corner[0][0],corner[0][1]), 3, (0,0,255) , thickness=3)
        #print(corner[0])

    print(ret)
    imgpoints.append(corners)
    objpoints.append(objp)
    # If found, add object points, image points (after refining them)
    # if ret == True:
    #     objpoints.append(objp)

    #     corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    #     imgpoints.append(corners2)

    #     # Draw and display the corners
    #     img = cv2.drawChessboardCorners(img, (width,height), corners2,ret)
    #     cv2.imshow('img',img)
    #     cv2.waitKey(500)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
print "\n\n", ret
print "\n\n", dist
print "\n\n", tvecs

cv2.destroyAllWindows()