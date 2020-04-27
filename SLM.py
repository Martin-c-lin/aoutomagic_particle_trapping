import numpy as np
import matplotlib.pyplot as plt
from math import ceil,pi
from random import random
from time import time
# TODO make all this into functions and investigate if we can change to smaller
# datatypes to improve performance
# See if smaller datatypes and upscaling can be used to improve performance

# random mask encoding algorithm  (RM)
def RM(N,M,Delta,image_width):
    return Delta[np.random.randint(0,M,N),range(N)]
# Random Superposition Algorithm (SR)
def RS(N,M,Delta,image_width):
    RN = np.transpose(np.random.uniform(low=0.0, high=2*pi, size=(1,M)))*np.ones((1,N))
    return np.angle(np.sum(np.exp(1j*(Delta+RN)),axis=0))+pi
# Weighted Gerchberg-Saxton Algorithm (GSW)
def GSW(N,M,Delta=None,image_width=1080,nbr_iterations=30):
    if Delta is None:
        Delta = SLM.get_delta(image_width=image_width)
    Phi = RS(N,M,Delta,image_width) # Initial guess
    W = np.ones((M,1))
    I_m =np.uint8(np.ones((M,1)))
    I_N = np.uint8(np.ones((1,N)))
    Delta_J = np.exp(1j*Delta)
    for J in range(nbr_iterations):
        V = np.reshape(np.mean((np.exp(1j*(I_m*Phi)-Delta)),axis=1),(M,1)) # axis = 0
        V_abs = abs(V)
        W = np.mean(V_abs)*np.divide(W,V_abs)
        Phi = np.angle(sum(np.multiply(Delta_J, np.divide(np.multiply(W,V),V_abs)*I_N))) # axis = 0
        print(J,np.shape(Phi),np.shape(V),np.shape(W))

    return np.reshape(128+Phi*255/(2*pi),(image_width,image_width))

def get_delta(image_width = 1080,d = 60e-6,d0x=-120e-6,d0y=-115e-6):
    """
    Calculates delta in paper. I.e the phase shift of light when travelling from
    the SLM to the trap position for a specific set of points
    Default parameters copied from Allessandros script


    """
    x = np.linspace(1,image_width,image_width)
    y = np.reshape(np.transpose(np.linspace(1,image_width,image_width)),(image_width,1))

    I = np.ones((1,image_width))
    N = len(x)**2
    p = 9e-6 # pixel size?
    f = np.sqrt(2e-4*0.4) # Focal length of imaging system. Empirically tested value
    z = 0
    lambda_ = 532e-9

    M = 16#n1*n2 # Total number of traps

    xm = np.zeros((M))
    ym = np.zeros((M))

    zm = np.zeros((M))


    #
    # ym[0] = d0y-0.5*d*np.sqrt(3/4)
    # ym[1] = d0y-0.5*d*np.sqrt(3/4)
    # ym[2] = d0y-0.5*d*np.sqrt(3/4)
    # ym[3] = d0y-0.5*d*np.sqrt(3/4)
    #
    # ym[4] = d0y
    # ym[5] = d0y
    # ym[6] = d0y
    # ym[7] = d0y
    #
    # ym[8] = d0y+0.5*d*np.sqrt(3/4)
    # ym[9] = d0y+0.5*d*np.sqrt(3/4)
    # ym[10] = d0y+0.5*d*np.sqrt(3/4)
    # ym[11] = d0y+0.5*d*np.sqrt(3/4)
    #
    # ym[12] = d0y + (0.5*d*np.sqrt(3/4)*2)
    # ym[13] = d0y + (0.5*d*np.sqrt(3/4)*2)
    # ym[14] = d0y + (0.5*d*np.sqrt(3/4)*2)
    # ym[15] = d0y + (0.5*d*np.sqrt(3/4)*2)

    # Hexagonal grid
    # xm[0] = d0x-d/2
    # xm[1] = d0x
    # xm[2] = d0x+d/2
    # xm[3] = d0x-d/2 +d/4
    # xm[4] = d0x+d/4
    # xm[5] = d0x+d/2+d/4
    # xm[6] = d0x-d/2
    # xm[7] = d0x
    # xm[8] = d0x+d/2
    #
    # ym[0] = d0y-0.5*d*np.sqrt(3/4)
    # ym[1] = d0y-0.5*d*np.sqrt(3/4)
    # ym[2] = d0y-0.5*d*np.sqrt(3/4)
    # ym[3] = d0y
    # ym[4] = d0y
    # ym[5] = d0y
    # ym[6] = d0y+0.5*d*np.sqrt(3/4)
    # ym[7] = d0y+0.5*d*np.sqrt(3/4)
    # ym[8] = d0y+0.5*d*np.sqrt(3/4)

    for i in range(4):
        xm[i*4+0] = d0x-d/2
        xm[i*4+1] = d0x
        xm[i*4+2] = d0x+d/2
        xm[i*4+3] = d0x+d
        ym[i*4+0] = d0y+d/2*(i-1)
        ym[i*4+1] = d0y+d/2*(i-1)
        ym[i*4+2] = d0y+d/2*(i-1)
        ym[i*4+3] = d0y+d/2*(i-1)
        '''
    xm[0] = d0x-d/2
    xm[1] = d0x
    xm[2] = d0x+d/2
    xm[3] = d0x-d/2
    xm[4] = d0x
    xm[5] = d0x+d/2
    xm[6] = d0x-d/2
    xm[7] = d0x
    xm[8] = d0x+d/2

    ym[0] = d0y-0.5*d
    ym[1] = d0y-0.5*d
    ym[2] = d0y-0.5*d
    ym[3] = d0y
    ym[4] = d0y
    ym[5] = d0y
    ym[6] = d0y+0.5*d
    ym[7] = d0y+0.5*d
    ym[8] = d0y+0.5*d
        '''

    Delta=np.zeros((M,N))
    for m in range(M):


        # Calculate delta according to eq : in paper
        # Using python "%" instead of Matlabs "rem"
        Delta[m,:]=np.reshape(2*pi*p/lambda_/f*((np.transpose(I)*x*xm[m]+(y*I)*ym[m]) + 1/(2*f)*zm[m] * ( (np.transpose(I)*x)**2 + (y*I)**2 )) % (2*pi),(1,N))
        # TODO Add z-dependence to to ensuere that this works also in 3d
        # Can we remove the %2pi and * 2 *pi?
    return Delta,N,M
def setup_fullscreen_plt_image():
    '''
    This script magically sets up pyplot lib so it displays an image on a secondary display
    in full screen.
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
    #plt.imshow(image,cmap='gist_gray')
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
'''
image_width = int(1080)
Delta,N,M = get_delta(image_width)
start = time()
image = np.reshape(GSW(N,M,Delta,image_width,nbr_iterations=20)*255/(2*pi),(image_width,image_width))
print("Execution time GSW:",time()-start)

setup_fullscreen_plt_image()
plt.imshow(image,cmap='gist_gray')
plt.pause(1)
'''
