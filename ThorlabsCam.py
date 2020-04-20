import clr
import sys
from System import Int32
import nicelib
import win32com.client as win32
sys.path.append('C:/Program Files/Thorlabs/Scientific Imaging/DCx Camera Support/Develop/Lib')
sys.path.append("C:/Program Files/Thorlabs/Scientific Imaging/DCx Camera Support/Develop/DotNet/signed")
from instrumental import instrument, list_instruments,drivers,u,Q_
import time
# https://instrumental-lib.readthedocs.io/en/stable/uc480-cameras.html -> Link to website with the camera package

def get_camera():
    list_instruments()
    return drivers.cameras.uc480.UC480_Camera()
def number_to_millisecond(nbr):
    return str(nbr)+'ms'
def distance_2D(P1,P2):
    from numpy import sqrt
    return sqrt((P1[0]-P2[0])**2+(P1[1]-P2[1])**2)
def find_closest(centers,target):
    from numpy import inf
    index=0
    distance = inf
    for i in range(len(centers)):
        tmpDistance = distance_2D(centers[i],target)
        if tmpDistance<distance:
            distance = tmpDistance
            index = i
    return index,distance
def find_exposure_time(cam,targetIntensity=100,margin=5):
    """
    Function for finding a exposure which yields a suitable exposure for experiments

    # Potential future update, replace np.mean() with a function taken as input
    """
    from numpy import mean

    if targetIntensity<0 or targetIntensity>255:
        print("Invalid target intensity")
        return 1

    minExposure = 0.01 # Smallest value in ms
    maxExposure = 80
    counter = 0

    # Calculate exposures at the different end
    Image=cam.grab_image(timeout='1s', copy=True,exposure_time=number_to_millisecond(minExposure))
    minIntensity = mean(Image)

    Image=cam.grab_image(timeout='1s', copy=True,exposure_time=number_to_millisecond(maxExposure))
    maxIntensity = mean(Image)

    midIntensity=1
    while (midIntensity<targetIntensity-margin)or(midIntensity>targetIntensity+margin) and counter<100 :
        # Set exposure, take a picture and check how good it was
        counter=counter+1

        midExposure=(maxExposure+minExposure)/2
        Image=cam.grab_image(timeout='1s', copy=True,exposure_time=number_to_millisecond(midExposure))
        midIntensity = mean(Image)

        if(midIntensity>targetIntensity):# Exposure time too short
            maxExposure=midExposure
            maxIntensity=midIntensity
        else:                           # Exposure time too long
            minExposure=midExposure
            minIntensity=midIntensity
    if(counter==100):
        print("WARNING: Find exposure function ran max number of iterations! No really suitable exposure setting found")
    # Update the exposure time of the camera and return the target exposure
    cam.set_defaults(exposure_time=number_to_millisecond(midExposure))
    return number_to_millisecond(midExposure)
