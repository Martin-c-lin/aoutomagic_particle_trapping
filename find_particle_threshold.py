import cv2
import numpy as np
from matplotlib import pyplot as plt
import scipy.ndimage as ndi
from skimage import measure
import time
def find_single_particle_center(img,threshold=127):
    """
    Locate the center of a single particle in an image.
    Obsolete now, find_particle_centers is surperior in every way
    """
    img_temp = cv2.medianBlur(img,5)
    ret,th1 = cv2.threshold(img_temp,threshold,255,cv2.THRESH_BINARY)
    cy, cx = ndi.center_of_mass(th1)
    # if np.isnan(cx) return inf?
    return cx,cy,th1
def threshold_image(image,threshold=120,bright_particle=True):
        img_temp = cv2.medianBlur(image,5)
        if bright_particle:
            ret,thresholded_image = cv2.threshold(img_temp,threshold,255,cv2.THRESH_BINARY)
            return thresholded_image
        else:
            ret,thresholded_image = cv2.threshold(img_temp,threshold,255,cv2.THRESH_BINARY_INV)
            return thresholded_image
def find_particle_centers(image,threshold=120,particle_size_threshold=200,particle_upper_size_threshold=5000,bright_particle=True):
    """
    Function which locates particle centers using thresholding.
    Parameters :
        image - Image with the particles
        threshold - Threshold value of the particle
        particle_size_threshold - minimum area of particle in image measured in pixels
        bright_particle - If the particle is brighter than the background or not
    Returns :
        x,y - arrays with the x and y coordinates of the particle in the image in pixels.
            Returns empty arrays if no particle was found
    """

    # Do thresholding of the image
    img_temp = cv2.medianBlur(image,5)
    if bright_particle:
        ret,thresholded_image = cv2.threshold(img_temp,threshold,255,cv2.THRESH_BINARY)
    else:
        ret,thresholded_image = cv2.threshold(img_temp,threshold,255,cv2.THRESH_BINARY_INV)

    # Separate the thresholded image into different sections
    separate_particles_image = measure.label(thresholded_image)
    # Count the number of pixels in each section
    counts = np.bincount(np.reshape(separate_particles_image,(np.shape(separate_particles_image)[0]*np.shape(separate_particles_image)[1])))
    x = []
    y = []
    group = 0

    #tmp_image = np.zeros()
    # Check for pixel sections which are larger than particle_size_threshold.

    for pixel_count in counts: # First will be background
        if particle_upper_size_threshold>pixel_count>particle_size_threshold:
            # Particle found, locate center of mass of the particle
            cy, cx = ndi.center_of_mass(separate_particles_image==group)
            x.append(cx)
            y.append(cy)

        group +=1
    #cv2.imwrite(str(threshold)+"threshold-" + time.strftime("%d-%m-%Y-%H-%M-%S") + ".jpg", cv2.cvtColor(thresholded_image, cv2.COLOR_RGB2BGR))

    return x,y
