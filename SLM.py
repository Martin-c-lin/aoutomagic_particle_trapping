import numpy as np
import matplotlib.pyplot as plt
from math import ceil,pi
from random import random
from time import time
from math import atan2
# TODO make all this into functions and investigate if we can change to smaller
# datatypes to improve performance
# See if smaller datatypes and upscaling can be used to improve performance

# random mask encoding algorithm  (RM)

def atan2_vec_2d(Y,X):
    '''
    Function for calculating atan2 of 2 2d arrays of coordinate positions(X,Y)

    Parameters
    ----------
    Y : 2-d array of y coordinates
    X : 2-d array of x-coordinates

    Returns
    -------
    2d-array of same shape as x and y.
    None if x and y are not of the same shape

    '''
    shape = np.shape(X)

    if np.shape(X)==np.shape(Y):

        Y = np.reshape(Y,shape[0]*shape[1])
        X = np.reshape(X,shape[0]*shape[1])
        res = np.zeros(len(X))
        for idx in range(len(X)):
            res[idx] = atan2(Y[idx],X[idx])
        return np.reshape(res,shape)
    else:
        print('Error, vectors not of equal length', len(X),' is not equal to ',len(Y))
        return None

def get_LGO(image_width=1080,order = -8):
    '''
    Parameters
    ----------
    image_width : TYPE, optional
        DESCRIPTION. The default is 1080.

    Returns
    -------
    LGO : TYPE
        LGO, phase shift required to delta to get a laguerre gaussian instead of a gaussian.

    '''

    xc = image_width/2
    yc = image_width/2
    xxx,yyy = np.meshgrid(np.linspace(1,image_width,image_width),
                    np.linspace(1,image_width,image_width))
    LGO = np.mod((order * atan2_vec_2d(yyy-yc,xxx-xc)) ,(2*pi)) # Ther should maybe be a +pi before mod 2pi

    return LGO

def RM(N,M,Delta,image_width):
    return Delta[np.random.randint(0,M,N),range(N)]
# Random Superposition Algorithm (SR)
def RS(N,M,Delta):
    RN = np.transpose(np.random.uniform(low=0.0, high=2*pi, size=(1,M)))*np.ones((1,N))
    return np.angle(np.sum(np.exp(1j*(Delta+RN)),axis=0))+pi
# Weighted Gerchberg-Saxton Algorithm (GSW)
def GSW(N,M,Delta=None,image_width=1080,nbr_iterations=30):
    if Delta is None:
        Delta = SLM.get_delta(image_width=image_width)
    Phi = RS(N,M,Delta) # Initial guess
    W = np.ones((M,1))
    I_m =np.uint8(np.ones((M,1)))
    I_N = np.uint8(np.ones((1,N)))
    Delta_J = np.exp(1j*Delta)
    for J in range(nbr_iterations):
        V = np.reshape(np.mean((np.exp(1j*(I_m*Phi)-Delta)),axis=1),(M,1))
        V_abs = abs(V)
        W = np.mean(V_abs)*np.divide(W,V_abs)
        Phi = np.angle(sum(np.multiply(Delta_J, np.divide(np.multiply(W,V),V_abs)*I_N)))
        print('Iteration: ', J+1, 'of ', nbr_iterations)

    return np.reshape(128+Phi*255/(2*pi),(image_width,image_width))
def  GS(N,M,Delta=None,image_width=1080,nbr_iterations=30):
    if Delta is None:
        Delta = SLM.get_delta(image_width=image_width)
    Phi = RS(N,M,Delta) # Initial guess
    W = np.ones((M,1))
    I_m =np.uint8(np.ones((M,1)))
    I_N = np.uint8(np.ones((1,N)))
    Delta_J = np.exp(1j*Delta)
    for J in range(nbr_iterations):
        V = np.reshape( np.transpose( np.mean((np.exp(1j*(I_m*Phi)-Delta)),axis=1) ),(M,1))
        Phi = np.angle(sum(np.multiply(Delta_J,np.divide(V,abs(V)))*I_N ))
        print('Iteration: ', J+1, 'of ', nbr_iterations)
    result = np.reshape(128+Phi*255/(2*pi),(image_width,image_width))
    return  result
def get_default_xm_ym():
    '''
    Generates default x,y positions for particle.
    Legacy funciton, use get_xm_ym_rect instead
    '''
    M = 9 # Changed to 9 from 16
    xm = np.zeros((M))
    ym = np.zeros((M))

    d = 65e-6
    d0x=-115e-6
    d0y=-115e-6

    fac = 3
    for i in range(fac):
        xm[i*fac+0] = d0x-d/2
        xm[i*fac+1] = d0x
        xm[i*fac+2] = d0x+d/2
        ym[i*fac+0] = d0y+d/2*(i-1)
        ym[i*fac+1] = d0y+d/2*(i-1)
        ym[i*fac+2] = d0y+d/2*(i-1)

    return xm,ym
def get_xm_ym_rect(nbr_rows,nbr_columns, dx=30e-6,dy=30e-6, d0x=-115e-6, d0y=-115e-6):
    '''
    Generates xm,ym in a rectangular grid with a particle-particle distance of d.
    '''
    if nbr_rows < 1 or nbr_columns < 1:
        return [],[]
    xm = np.zeros((nbr_rows * nbr_columns))
    ym = np.zeros((nbr_rows * nbr_columns))

    for i in range(nbr_rows):
        for j in range(nbr_columns):
            xm[i*nbr_columns+j] = d0x + dx*j
            ym[i*nbr_columns+j] = d0y + dy*i
    return xm,ym

def get_xm_ym_E_L_Triangle(d=30e-6, d0x=-115e-6, d0y=-115e-6):
    '''
    Generates xm,ym in a equilateral triangle with sidelength d and first
    corner placed on (d0x,d0y)
    '''
    xm = np.zeros(3)
    ym = np.zeros(3)

    xm[0] = d0x
    ym[0] = d0y
    xm[1] = d0x
    ym[1] = d0y + d
    xm[2] = d0x + np.sqrt(3/4) * d
    ym[2] = d0y + d/2
    return xm, ym

def get_xm_ym_triangle_with_center(d=30e-6, d0x=-115e-6, d0y=-115e-6):
        '''
        Generates xm,ym in a equilateral triangle with sidelength d and a
        particle in the center at d0x,d0y
        '''
        xm = np.zeros(4)
        ym = np.zeros(4)

        xm[0] = d0x
        ym[0] = d0y

        xm[1] = d0x - d / np.sqrt(12)
        ym[1] = d0y + d / 2
        xm[2] = d0x - d / np.sqrt(12)
        ym[2] = d0y - d / 2
        xm[3] = d0x + d * (np.sqrt(3/4) - np.sqrt(1/12))
        ym[3] = d0y

        return xm, ym
def get_Isaac_xm_ym(d=30e-6, d0x=-115e-6, d0y=-115e-6):
    '''
    Two particles, first placed at [d0x, d0y] other at [d0x, d0y + d]
    '''


    xm = np.zeros((2))
    ym = np.zeros((2))

    xm[0] = d0x
    xm[1] = d0x

    ym[0] = d0y
    ym[1] = d0y+d

    return xm, ym



def get_delta(image_width = 1080, xm=[], ym=[], use_LGO=[False], order=-8):
    """
    Calculates delta in paper. I.e the phase shift of light when travelling from
    the SLM to the trap position for a specific set of points
    Default parameters copied from Allessandros script
    """
    x = np.linspace(1,image_width,image_width)
    y = np.reshape(np.transpose(np.linspace(1,image_width,image_width)),(image_width,1))

    I = np.ones((1,image_width))
    N = image_width**2
    p = 9e-6 # pixel size
    f = np.sqrt(2e-4*0.4) # Focal length of imaging system. Empirically found value
    z = 0
    lambda_ = 532e-9


    if len(xm)<1 or len(ym)<1:
        xm,ym = get_default_xm_ym()
        use_LGO = [False for i in range(len(xm))]
    # TODO make the order into a list
    if True in use_LGO:
        LGO = get_LGO(image_width,order=order)
    M = len(xm) # Total number of traps
    zm = np.zeros((M))
    Delta=np.zeros((M,N))
    for m in range(M):

        # Calculate delta according to eq : in paper
        # Using python "%" instead of Matlabs "rem"
        Delta[m,:]=np.reshape(2*pi*p/lambda_/f*((np.transpose(I)*x*xm[m]+(y*I)*ym[m]) + 1/(2*f)*zm[m] * ( (np.transpose(I)*x)**2 + (y*I)**2 )) % (2*pi),(1,N))
        if len(use_LGO)>m and use_LGO[m]: # TODO, check if this is the way to add this
            Delta[m,:] += np.reshape(LGO,(N))
            Delta[m,:] = Delta[m,:] % (2*pi)
        # TODO Add z-dependence to to ensuere that this works also in 3d
    return Delta,N,M
def setup_fullscreen_plt_image():
    '''
    This script magically sets up pyplot lib so it displays an image on a secondary display
    in full screen.
    Legacy function. USe the SLM_controller script with tkinter windows instead
    '''
    plt.switch_backend('QT4Agg')

    # a little hack to get screen size; from here [1]
    mgr = plt.get_current_fig_manager()
    mgr.full_screen_toggle()
    py = mgr.canvas.height()
    px = mgr.canvas.width()
    mgr.window.close()
    # hack end

    plt.figure()
    plt.rcParams['toolbar'] = 'None'
    fig = plt.gcf()
    fig.canvas.window().statusBar().setVisible(False)
    fig.set_size_inches(8,8)
    figManager = plt.get_current_fig_manager()
    plt.subplots_adjust(left=0, right=1, top=1, bottom=0)

    # if px=0, plot will display on 1st screen
    figManager.window.move(px, 0)
    figManager.window.showMaximized()
    figManager.window.setFocus()
