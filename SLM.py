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
def GSW(N,M,Delta,image_width,nbr_iterations=30):
    Phi = RS(N,M,Delta,image_width) # Initial guess
    W = np.ones((M,1))
    I_m =np.uint8(np.ones((M,1)))
    I_N = np.uint8(np.ones((1,N)))
    Delta_J = np.exp(1j*Delta)
    for J in range(nbr_iterations):
        # New slightly faster implementaition
        V = np.abs(np.mean((np.exp(1j*(I_m*Phi-Delta))),axis=0))
        V_abs = np.abs(V)
        W = np.mean(V_abs)*(W/V_abs)
        Phi = np.angle(np.sum(Delta_J*((W*V/V_abs)*I_N),axis=0))

        print("Iteration no:",J)
    return Phi
def Get_Delta(image_width = 1080,d = 20e-6,d0x=-120e-6,d0y=30e-6):
    """
    Default parameters copied from Allessandros script
    """
    x = np.linspace(1,image_width,image_width)
    y = np.reshape(np.transpose(np.linspace(1,image_width,image_width)),(image_width,1))

    I = np.ones((1,image_width))
    N = len(x)**2
    p = 9e-6 # pixel size?
    f = np.sqrt(2e-4*0.4)
    z = 0
    lambda_ = 532e-9

    n1 = 3 # point-rows not used now
    n2 = 1  # #point-columns
    # d = 20e-6 # Distance between particles
    # d0x=-120e-6 # From allessandros matlabs script
    # d0y=30e-6
    M = n1*n2 # Total number of particles


    xm = np.zeros((n1,n2))
    ym = np.zeros((n1,n2))

    zm = np.zeros((n1,n2))

    for j in range(n1):
        for k in range(n2):
            xm[j,k] = (((-(n1-1)*d/2)+(j)*d)+d0x)
            ym[j,k] = (((-(n2-1)*d/2)+(k)*d)+d0y)
            zm[j,k] = z


    # d=15e-6;
    # dd=1e-7;
    #
    xm[0][0] = d0x
    xm[1][0] = d0x
    xm[2][0] = d0x+0.7*d*np.sqrt(3)
    ym[0][0] = 0.7*d+d0y
    ym[1][0] = -0.7*d+d0y
    ym[2][0] = d0y
    Delta=np.zeros((M,N))
    for m in range(M):
        # Calculate delta according to eq : in paper
        # Using python "%" instead of Matlabs "rem"
        i2,i1 = divmod(m,n1)
        # TODO Add z-dependence to improve the possibility to move the traps in 3d
        Delta[m,:]=np.reshape(2*pi*p/lambda_/f*(np.transpose(I)*x*xm[i1,i2]+(y*I)*ym[i1,i2])%(2*pi),(1,N))
    return Delta,N,M
def setup_fullscreen_plt_image():
    plt.switch_backend('QT4Agg')

    # a little hack to get screen size; from here [1]
    mgr = plt.get_current_fig_manager()
    mgr.full_screen_toggle()
    py = mgr.canvas.height()
    px = mgr.canvas.width()
    mgr.window.close()
    # hack end

    x = [i for i in range(0,10)]
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
image_width = int(1080)
Delta,N,M = Get_Delta(image_width)
start = time()
image = np.reshape(GSW(N,M,Delta,image_width,nbr_iterations=20)*255/(2*pi),(image_width,image_width))
print("Execution time GSW:",time()-start)

setup_fullscreen_plt_image()
plt.imshow(image,cmap='gist_gray')
plt.pause(1)
# for i in range(30):
#     Delta,N,M = Get_Delta(image_width,d=i*1e-6)
#     start = time()
#     image = np.reshape(GSW(N,M,Delta,image_width,nbr_iterations=0)*255/(2*pi),(image_width,image_width))
#     plt.imshow(image,cmap='gist_gray')
#     plt.pause(0.01)
# plt.show()
