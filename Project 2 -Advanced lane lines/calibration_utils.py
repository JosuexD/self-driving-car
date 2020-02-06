import cv2
import numpy as np
import glob
import matplotlib.pyplot as plt
import os.path as path
import pickle

def cache_calibration(func):
    """
    Decorator checks if we have an existing pickle file that holds calibration information.
    If we already have calibrated information, we simply return it and avoid extra computation.
    If we do not have calibration data, we will go through the calibration cycle and dump it as a pickle file for future use.
    """

    calibration_cache = 'calibration_cached_data.pickle'

    def wrapper(*args, **kwargs):
        # we check if the pickle file with the calibration information exists
        if path.exists(calibration_cache):
            print("Loading cached camera calibration...", end= ' ')
            with open(calibration_cache, 'rb') as dump_file:
                # returns the data if found. This avoid an extra call to calibrate_camera
                calibration = pickle.load(dump_file)
        
        else:
            # if we don't find the calibration data already computed, then we go ahead and call the original calibrate_camera function
            # afterwards we make sure to save the returned file by serializing it as cache
            print('Computing camera calibration...', end = ' ')
            calibration = func(*args, **kwargs)
            with open(calibration_cache, 'wb') as dump_file:
                pickle.dump(calibration, dump_file)
        
        print('Done.')
        return calibration
    
    return wrapper

@cache_calibration
def calibrate_camera(calib_images_dir, verbose = False):

    # specifying grid size. TODO - should be a function parameter
    gridX = 6
    gridY = 9

    # utilizing an assert to ensure that the path directory for calibration actually exists
    assert path.exists(calib_images_dir), '"{}" must exist and contain calibration images.'.format(calib_images_dir)

    # initializing the obj points array with zeros. We're using 6*9 since that is the checkerboard size that the images contain
    objp = np.zeros((gridX * gridY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:gridY, 0:gridX].T.reshape(-1, 2)

    # initializing empty list that will hold the obj and img points
    objpoints = [] 
    imgpoints = []

    # utilizing glob to read all the images that start with the name 'calibration'
    images = glob.glob(path.join(calib_images_dir, 'calibration*.jpg'))

    # looping through each image found 
    for filename in images:

        # opening the image using opencv
        img = cv2.imread(filename)
        # grayscale conversion of the image. Notice we're usign BGR as opposed to RGB since we used imread and not matplot's version
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # running the chessboard corners algorithm for a 6x9 chessboard
        pattern_found, corners = cv2.findChessboardCorners(gray, (gridY, gridX), None)

        if pattern_found is True:
            objpoints.append(objp)
            imgpoints.append(corners)

            if verbose:
                # Draw and display corners 
                img = cv2.drawChessboardCorners(img, (gridX, gridY), corners, pattern_found)
                cv2.imshow('img', img)
                cv2.waitKey(1500)

    if verbose:
        # if we are showing images due to the flag being on. After showing all the images and having pressed a key we'll close everything
        cv2.destroyAllWindows()
    
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    return ret, mtx, dist, rvecs, tvecs

def undistort(frame, mtx, dist, verbose = False):

    frame_undistorted = cv2.undistort(frame, mtx, dist, newCameraMatrix = mtx)

    if verbose:
        ax = plt.subplots(nrows = 1, ncols = 2)
        ax[0].imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        ax[1].imshow(cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2RGB))
        plt.show()

    return frame_undistorted

if __name__ == '__main__':

    ret, mtx, dist, rvecs, tvecs = calibrate_camera(calib_images_dir='camera_cal', verbose = True)

    img = cv2.imread('test_images/test2.jpg')

    img_undistorted = undistort(img, mtx, dist)

    # Specifying the output directory
    cv2.imwrite('output_images/calibration/test_calibration_before.jpg', img)
    cv2.imwrite('output_images/calibration/test_calibration_after.jpg', img_undistorted)