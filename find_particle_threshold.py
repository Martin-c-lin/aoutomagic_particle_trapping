import cv2
import numpy as np
from matplotlib import pyplot as plt
import scipy.ndimage as ndi

def find_single_particle_center(img,threshold=127):
    img_temp = cv2.medianBlur(img,5)
    ret,th1 = cv2.threshold(img_temp,threshold,255,cv2.THRESH_BINARY)
    cy, cx = ndi.center_of_mass(th1)
    # if np.isnan(cx) return inf?
    return cx,cy,th1
