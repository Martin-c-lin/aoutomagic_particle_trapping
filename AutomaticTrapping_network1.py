# Script for controlling the whole setup automagically
import ThorlabsCam as TC
import ThorlabsMotor as TM
# import control_threads
import find_particle_threshold as fpt
from instrumental import u
import matplotlib.pyplot as plt
import numpy as np
from keras.models import load_model
import threading,time,cv2,queue,copy,sys,tkinter
from tkinter import messagebox
from functools import partial
def get_default_control_parameters():
    control_parameters = {
    'trapped_counter': 0, # Maybe does not need to be here
    'serial_num_X': '27502438',
    'serial_num_Y': '27502419',
    'network_path': 'C:/Martin/Networks/',
    'polling_rate': 100,
    'continue_capture': True, # True if camera and dispaly should keep updating
    'motor_running': True, # Should the motor thread keep running?
    'zoomed_in': False, # Keeps track of whether the image is cropped or not
    'record': False, # Default
    'half_image_width':500,
    'AOI':[0,1000,0,1000],
    'new_AOI_camera': False,
    'new_AOI_display': False,
    'movement_threshold': 40,
    'record':False,
    'trap_1_absolute': [500,500],# Change to aritrary number of traps
    #'trap_1_relative': [trap_1_absolute[0],trap_1_absolute[1]],
    'trap_1_relative': [500,500],
    'center': [400,400],
    'camera_lock': threading.Lock(),
    'motor_locks': [threading.Lock(),threading.Lock()]
    }
    return control_parameters
def terminate_threads():
    '''
    Function for killing all the threads
    '''
    control_parameters['continue_capture'] = False # All threds exits their main loop once this parameter is changed
    control_parameters['motor_running'] = False
    messagebox.showinfo('Terminating threads')
    print('Terminating threads \n')
    time.sleep(3)
    global thread_list
    for thread in thread_list:
        thread.join()
    for thread in thread_list:
        del thread
def start_threads():
    """
    Function for starting all the threads, can only be called once
    """
    camera_thread = CameraThread(1, 'Thread-camera')
    motor_X_thread = MotorThread(2,'Thread-motorX',motor_X,0) # Last argument is to indicate that it is the x-motor and not the y
    motor_Y_thread = MotorThread(3,'Thread-motorY',motor_Y,1)
    display_thread = DisplayThread(4,'Thread-display')
    tracking_thread = TrackingThread(5,'Tracker_thread')
    camera_thread.start()
    motor_X_thread.start()
    motor_Y_thread.start()
    display_thread.start()
    tracking_thread.start()
    print('Camera,display, motor_X and motor_Y threads created')
    global thread_list
    thread_list.append(camera_thread)
    thread_list.append(motor_X_thread)
    thread_list.append(motor_Y_thread)
    thread_list.append(display_thread)
    thread_list.append(tracking_thread)
def create_buttons(top):
    exit_button = tkinter.Button(top, text ='Exit program', command = terminate_threads)
    start_button = tkinter.Button(top, text ='Start program', command = start_threads)
    up_button = tkinter.Button(top, text ='Move up', command = partial(move_button,0))
    down_button = tkinter.Button(top, text ='Move down', command = partial(move_button,1))
    right_button = tkinter.Button(top, text ='Move right', command = partial(move_button,2))
    left_button = tkinter.Button(top, text ='Move left', command = partial(move_button,3))
    start_record_button = tkinter.Button(top, text ='Start recording', command = start_record)
    stop_record_button = tkinter.Button(top, text ='Stop recording', command = stop_record)
    # TODO add button for zoom in
    exit_button.pack()
    start_button.pack()
    up_button.pack()
    down_button.pack()
    right_button.pack()
    left_button.pack()
    start_record_button.pack()
    stop_record_button.pack()
class DisplayThread(threading.Thread):
    '''
    Thread class for plotting the result in the BG.
    InitateMotorreally appreciated by python
    '''
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.setDaemon(True)

    def run(self):
        global control_parameters
        while control_parameters['continue_capture']:

            # Initiate interactive pyplot display
            ax = plt.axes() #axis([xmin, xmax, ymin, ymax])
            im = ax.imshow(image) # Image is global
            plt.ion()

            # Acknowledge that the display has been created
            control_parameters['new_AOI_display'] = False # AOI has been acknowledged

            # Start measuring the FPS
            start =  time.time() # For calculating framerate
            frame_counter = 0
            while control_parameters['continue_capture'] and not control_parameters['new_AOI_display']:

               im.set_data(image) #
               if not np.isnan(control_parameters['center'][0]):
                   plt.cla()
                   im = ax.imshow(image,cmap='Greys',vmin=0, vmax=255) # Repainting to prevent old center position from staying on screen
                   if control_parameters['center'][0]<(control_parameters['AOI'][1]-control_parameters['AOI'][0]) and control_parameters['center'][1]<(control_parameters['AOI'][3]-control_parameters['AOI'][2]) : # Accidentally doing stuff wrong here

                       plt.plot(control_parameters['center'][0],control_parameters['center'][1],'x',color='r')
               if control_parameters['record']:
                   plt.title('Recording ON')
               else:
                   plt.title('Recording OFF')
               plt.pause(0.0001)
               frame_counter +=1
            # CLose down the livedisplay and print FPS
            end = time.time()
            print('Fps for livefeed is ',frame_counter/(end-start))
            plt.ioff()
            plt.close()
class MotorThread(threading.Thread):
    '''
    Thread in which a motor is controlled. The motor object is available globally.
    '''
    def __init__(self, threadID, name,motor,axis):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.motor = motor
      self.axis = axis # 0 = x-axis, 1 = y axis
      self.setDaemon(True)
    def run(self):
       print('Running motor thread')
       global control_parameters
       while control_parameters['motor_running']:
           # Acquire lock to ensure that it is safe to move the motor
           control_parameters['motor_locks'][self.axis].acquire()

           if np.abs(control_parameters['center'][self.axis]-control_parameters['trap_1_relative'][self.axis])>=control_parameters['movement_threshold'] and not np.isnan(control_parameters['center'][self.axis]):
                maxPixel = np.abs(control_parameters['AOI'][1]-control_parameters['AOI'][0])
                TM.MoveMotorToPixel(self.motor ,targetPixel=control_parameters['center'][self.axis],currentPixel=control_parameters['trap_1_relative'][self.axis],maxPixel=maxPixel)
           control_parameters['motor_locks'][self.axis].release()
           time.sleep(0.1) # To give other threads some time to work
       return
class CameraThread(threading.Thread):
   def __init__(self, threadID, name,batch_size=100):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.batch_size = batch_size
      self.setDaemon(True)
   def run(self):

       print('Initiating threaded capture sequence')
       image_count = 0
       number_images_saved = 0 # counts
       # TODO - set framreate so we have a proper framerate in the videos!
       global control_parameters
       while control_parameters['continue_capture']:
           # Set defaults for camera, aknowledge that this has been done
           cam.set_defaults(left=control_parameters['AOI'][0],right=control_parameters['AOI'][1],top=control_parameters['AOI'][2],bot=control_parameters['AOI'][3])
           control_parameters['new_AOI_camera'] = False

           # Grab one example image
           global image
           image = cam.grab_image(n_frames=1)

           # Start livefeed from the camera
           cam.start_live_video() # Maximum framerate, shoul probably cap it
           start = time.time()

           # Create an array to store the images which have been captured in
           saved_images = np.zeros([self.batch_size ,np.abs(control_parameters['AOI'][0]-control_parameters['AOI'][1]),np.abs(control_parameters['AOI'][3]-control_parameters['AOI'][2])])

           # Start continously capturin images now that the camera parameters have been set
           while control_parameters['continue_capture'] and not control_parameters['new_AOI_camera']:
               cam.wait_for_frame(timeout=None)

               # Capture an image and update the image count
               control_parameters['camera_lock'].acquire()
               image[:][:][:] = cam.latest_frame()
               control_parameters['camera_lock'].release()
               image_count = image_count+1

               if control_parameters['record']:
                   # TODO Create option for recording video instead of simple images
                   saved_images[number_images_saved%self.batch_size ] = copy.copy(image) # Do not want a reference but a copy of its own
                   number_images_saved+=1
                   if number_images_saved%self.batch_size ==0 and number_images_saved>=1:
                       np.save('test_images/frames'+str(number_images_saved-self.batch_size )+'_'+str(number_images_saved),np.uint8(saved_images))
           # Close the livefeed and calculate the fps of the captures
           end = time.time()
           cam.stop_live_video()
           print('Capture sequence finished',image_count, 'Images captured in ',end-start,'seconds. \n FPS is ',image_count/(end-start))
class TrackingThread(threading.Thread):
   '''
   Thread which does the tracking
   '''
   def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.setDaemon(True)
   def run(self):
       global image
       global control_parameters
       network1 = load_model(control_parameters['network_path']+'network1.h5')
       network2 = load_model(control_parameters['network_path']+'network_101x101_2.h5')
       i = 0
       zoom_counter = 0
       while control_parameters['continue_capture']: # Change to continue tracking?
           if not control_parameters['zoomed_in']:
               '''
               We are in full frame mode looking for a particle
               '''
               control_parameters['center'][0],control_parameters['center'][1],picture = fpt.find_single_particle_center(copy.copy(image),threshold = 200) # Do we need a copy of this or move it to another thread?s

               if is_trapped():
                   '''
                   We are not zoomed in but have found a particle.
                   Time to zoom in on it.
                   '''
                   control_parameters['zoomed_in'] =  True
                   half_image_width = 50
                   control_parameters['movement_threshold'] = 10
                   set_AOI(half_image_width=half_image_width)
           elif control_parameters['zoomed_in']:
               '''
               We are in zoomed in mode ready to predict the position using a network
               '''
               # predict_particle_position(network2,print_position=False) # Network cannot yet be trusted. Was fit to differen objectve magnification
               control_parameters['center'][0],control_parameters['center'][1],picture = fpt.find_single_particle_center(copy.copy(image),threshold = 200)
               print('Center is',control_parameters['center'])

               if is_trapped():
                   zoom_counter = 0 # particle is trapped do not want to increase the counter
               else:
                  '''
                  We are in zoomed in mode but have lost the particle
                  '''
                  zoom_counter += 1 # Particle might not be trapped, lets wait and see
                  if zoom_counter>10:
                      #
                      control_parameters['zoomed_in'] =  False
                      half_image_width = 500
                      control_parameters['movement_threshold'] = 20
                      set_AOI(half_image_width=half_image_width)
                      zoom_counter = 0
           time.sleep(0.15) # Needed to prevent this thread from running too fast
           i+=1
def set_AOI(half_image_width=50):
    '''
    Function for changing the Area Of Interest for the camera to the box specified by
    left,right,top,bottom
    Assumes global access to
    '''
    global control_parameters

    # Do not want motors to be moving when changing AOI!
    control_parameters['motor_locks'][0].acquire()
    control_parameters['motor_locks'][1].acquire()

    # Update the area of interest
    if control_parameters['zoomed_in']:
        # Zoom in on particle
        control_parameters['AOI'] = [control_parameters['trap_1_relative'][0]-half_image_width, control_parameters['trap_1_relative'][0]+half_image_width, control_parameters['trap_1_relative'][1]-half_image_width, control_parameters['trap_1_relative'][1]+half_image_width]# +1 due to deeptrack oddity
    else:
        # Use defult center
        control_parameters['AOI'] = [0,2*half_image_width,0,2*half_image_width]
    print('Setting AOI to ',control_parameters['AOI'])

    # Inform the camera and display thread about the updated AOI
    control_parameters['new_AOI_camera'] = True
    control_parameters['new_AOI_display'] = True

    # Update trap relative position
    control_parameters['trap_1_relative'][0] = control_parameters['trap_1_absolute'][0]- control_parameters['AOI'][0]
    control_parameters['trap_1_relative'][1] = control_parameters['trap_1_absolute'][1]- control_parameters['AOI'][2]
    control_parameters['center'] = [control_parameters['trap_1_relative'][0],control_parameters['trap_1_relative'][1]] # Don't want the motors to move just yet
    print('trap 1 rel , center',control_parameters['trap_1_relative'],control_parameters['center'])

    time.sleep(0.2) # Give motor threads time to catch up
    control_parameters['motor_locks'][0].release()
    control_parameters['motor_locks'][1].release()
def predict_particle_position(network,half_image_width=50,network_image_width=101,print_position=False):
    '''
    Function for making g a prediciton with a network and automatically updating the center position array.
    inputs :
        network - the network to predict with. Trained with deeptrack
        half_image_width - half the width of the image. needed to convert from deeptrack output to pixels
        network_image_width - Image width that the newtwork expects
        print_position - If the predicted positon should be printed in the console or not
    Outputs :
        Updates the center position of the particle
    '''
    global image
    global control_parameters
    #  TODO - Read half_image_width from the image
    resized = cv2.resize(copy.copy(image), (network_image_width,network_image_width), interpolation = cv2.INTER_AREA)
    pred = network.predict(np.reshape(resized/255,[1,network_image_width,network_image_width,1]))

    control_parameters['camera_lock'].acquire() # Might not need the lock
    control_parameters['center'][0] = half_image_width+pred[0][1]*half_image_width
    control_parameters['center'][1] = half_image_width+pred[0][0]*half_image_width
    control_parameters['camera_lock'].release()

    if print_position:
        print('Predicted posiiton is ',control_parameters['center'])
def is_trapped(threshold_distance=50):
    '''
    Function for detemining if a particle has been trapped or not
    '''
    global control_parameters
    distance = np.sqrt((control_parameters['center'][0]-control_parameters['trap_1_relative'][0])**2 + (control_parameters['center'][1]-control_parameters['trap_1_relative'][1])**2)

    if distance<threshold_distance:
        control_parameters['trapped_counter'] += 1 # TODO put uppper limit on trapped zoom_counter
        control_parameters['trapped_counter'] = np.minimum(10,control_parameters['trapped_counter'])
    else:
        control_parameters['trapped_counter'] -= 1
    if control_parameters['trapped_counter']<0:
        control_parameters['trapped_counter']=0
    return control_parameters['trapped_counter']>=8
def move_button(move_direction):
    '''
    Button function for manually moving the motors a bit
    The direction refers to the direction a particle in the fiel of view will move on the screen
    move_direction = 0 => move up
    move_direction = 1 => move down
    move_direction = 2 => move right
    move_direction = 3 => move left
    '''
    move_dist = 0.05 # Quite arbitrary movemement amount
    global control_parameters
    global motor_Y
    global motor_X
    if control_parameters['zoomed_in']:
        move_dist = 0.02
    if move_direction==0:
        # Move up (Particles in image move up on the screen)
        control_parameters['motor_locks'][1].acquire()
        TM.MoveMotor(motor_Y,-move_dist)
        control_parameters['motor_locks'][1].release()
    elif move_direction==1:
        # Move down
        control_parameters['motor_locks'][1].acquire()
        TM.MoveMotor(motor_Y,move_dist)
        control_parameters['motor_locks'][1].release()
    elif move_direction==2:
        # Move right
        control_parameters['motor_locks'][0].acquire()
        TM.MoveMotor(motor_X,-move_dist)# minus is right
        control_parameters['motor_locks'][0].release()
    elif move_direction==3:
        # Move left
        control_parameters['motor_locks'][0].acquire()
        TM.MoveMotor(motor_X,move_dist)
        control_parameters['motor_locks'][0].release()
    else:
        print('Invalid move direction')
def start_record():
    '''
    Button function for starting of recording
    '''
    control_parameters['record']= True
    print('Recording is on')
def stop_record():
    '''
    Button function for starting of recording
    '''
    control_parameters['record']= False
    print('Recording is off')
############### Main script starts here ####################################

control_parameters = get_default_control_parameters()
# Initate contact with motors
#TODO move these to the threads or not?
motor_X = TM.InitateMotor(control_parameters['serial_num_X'],pollingRate=control_parameters['polling_rate'])
motor_Y = TM.InitateMotor(control_parameters['serial_num_Y'],pollingRate=control_parameters['polling_rate'])

# Create camera and set defaults
cam = TC.get_camera()
cam.set_defaults(left=control_parameters['AOI'][0],right=control_parameters['AOI'][1],top=control_parameters['AOI'][2],bot=control_parameters['AOI'][3],n_frames=1)
exposure_time = TC.find_exposure_time(cam) # automagically finds a decent exposure time
print('Exposure time = ',exposure_time)

# Capture an example image to work with
image = cam.grab_image()

control_parameters['center'][0],control_parameters['center'][1],picture = fpt.find_single_particle_center(copy.copy(image),threshold = 150) # Do we need a copy of this or move it to another thread?s
control_parameters['center'][0],control_parameters['center'][1] = control_parameters['trap_1_relative'][0],control_parameters['trap_1_relative'][1] # Do not want it wandering on its own

# Create a empty list to put the threads in
thread_list = []

# Create a TK
top = tkinter.Tk()
# Create some buttons for basic control
create_buttons(top)
top.mainloop()

# Close the threads and the camera
control_parameters['continue_capture'] = False # All threds exits their main loop once this parameter is changed
control_parameters['motor_running'] = False

# Shut down camera and motors safely
cam.close()

TM.DisconnectMotor(motor_X)
TM.DisconnectMotor(motor_Y)
print(thread_list)
sys.exit()
