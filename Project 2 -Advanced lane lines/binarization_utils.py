import cv2
import numpy as np
import glob
import matplotlib.pyplot as plt

# global parameters used for obtaining yellow lane in HSV color space
HSV_yellow_threshold_min = np.array([0, 70, 70])
HSV_yellow_threshold_max = np.array([50, 255, 255])

def get_HSV_frame(frame, min_values, max_values, verbose = False):
    
    HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #TODO - Find better variable names
    min_th_ok = np.all(HSV > min_values, axis = 2)
    max_th_ok = np.all(HSV < max_values, axis = 2)

    # returning true whether the they're both the same value
    out = np.logical_and(min_th_ok, max_th_ok)

    if verbose:
        plt.imshow(out, cmap='gray')
        plt.show()

    return out

def get_equalized_grayscale_frame(frame):
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    eq_global = cv2.equalizeHist(gray)

    _, th = cv2.threshold(eq_global, thresh = 250, maxval = 250, type = cv2.THRESH_BINARY)

    return th

def get_sobel_frame(frame, kernel_size):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize = kernel_size)
    sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize = kernel_size)

    # squaring the value of sobel x and sobel y in order to take the magnitude of both before scaling it
    sobel_mag = np.sqrt(sobel_x ** 2 + sobel_y ** 2)
    # scaling sobel to be 8bit or 0-255 in order to use the threshold parameters and obtain better results
    sobel_mag = np.uint8(sobel_mag / np.max(sobel_mag) * 255)
    

    # TODO - Figure out what the return values for this are
    _, sobel_mag = cv2.threshold(sobel_mag, 50, 1, cv2.THRESH_BINARY)

    return sobel_mag.astype(bool)

def get_binarized_frame(img, verbose = False):

    # obtaining the height and width of image
    h, w = img.shape[:2]

    binary = np.zeros(shape = (h, w), dtype = np.uint8)

    # applying HSV threshold to the V colorspace to obtain the yellow lane.
    HSV_yellow_mask = get_HSV_frame(img, HSV_yellow_threshold_min, HSV_yellow_threshold_max, verbose = False)
    binary = np.logical_or(binary, HSV_yellow_mask)

    eq_white_mask = get_equalized_grayscale_frame(img)
    binary = np.logical_or(binary, eq_white_mask)

    sobel_mask = get_sobel_frame(img, kernel_size = 9)
    binary = np.logical_or(binary, sobel_mask)

    kernel = np.ones((5, 5), np.uint8)
    closing = cv2.morphologyEx(binary.astype(np.uint8), cv2.MORPH_CLOSE, kernel)

    if verbose:
        f, ax = plt.subplots(2, 3)
        f.set_facecolor('white')
        ax[0, 0].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        ax[0, 0].set_title('Original Image')
        ax[0, 0].set_axis_off()

        ax[0, 0].set_facecolor('red')
        ax[0, 1].imshow(eq_white_mask, cmap='gray')
        ax[0, 1].set_title('Histogram Equalization')
        ax[0, 1].set_axis_off()

        ax[0, 2].imshow(HSV_yellow_mask, cmap='gray')
        ax[0, 2].set_title('HSV (Yellow Lane)')
        ax[0, 2].set_axis_off()

        ax[1, 0].imshow(sobel_mask, cmap='gray')
        ax[1, 0].set_title('Sobel (Edge Detection)')
        ax[1, 0].set_axis_off()

        ax[1, 1].imshow(binary, cmap='gray')
        ax[1, 1].set_title('Before Morph Transformation')
        ax[1, 1].set_axis_off()

        ax[1, 2].imshow(closing, cmap='gray')
        ax[1, 2].set_title('After Morph Transformation')
        ax[1, 2].set_axis_off()
        plt.show()

    return closing

if __name__ == '__main__':
    test_images = glob.glob('test_images/*.jpg')
    for test_image in test_images:
        img = cv2.imread(test_image)
        get_binarized_frame(img = img, verbose = True)
