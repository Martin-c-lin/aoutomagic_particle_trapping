# Script for controlling the whole setup automagically
import ThorlabsCam as TC
import ThorlabsMotor as TM
import find_particle_threshold as fpt
from instrumental import u
import matplotlib.pyplot as plt
import numpy as np
from keras.models import load_model
import threading,time,cv2,queue,copy,sys
import tkinter
from tkinter import messagebox


def terminate_threads():
    """
    Function for killing all the threads
    """
    global continue_capture
    global motor_running

    continue_capture = False # All threds exits their main loop once this parameter is changed
    motor_running = False
    messagebox.showinfo("Terminating threads")
    print("Terminating threads \n")
    time.sleep(3)
    global thread_list
    for thread in thread_list:
        thread.join()
def start_threads():
    camera_thread = CameraThread(1, "Thread-camera")
    motor_X_thread = MotorThread(2,"Thread-motorX",motor_X,0) # Last argument is to indicate that it is the x-motor and not the y
    motor_Y_thread = MotorThread(3,"Thread-motorY",motor_Y,1)
    display_thread = DisplayThread(4,"Thread-display")
    tracking_thread = TrackingThread(5,"Tracker_thread")
    camera_thread.start()
    motor_X_thread.start()
    motor_Y_thread.start()
    display_thread.start()
    tracking_thread.start()
    print("Camera,display, motor_X and motor_Y threads created")
    global thread_list
    thread_list.append(camera_thread)
    thread_list.append(motor_X_thread)
    thread_list.append(motor_Y_thread)
    thread_list.append(display_thread)
    thread_list.append(tracking_thread)
class DisplayThread(threading.Thread):
    """
    Thread class for plotting the result in the BG.
    InitateMotorreally appreciated by python
    """
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.setDaemon(True)

    def run(self):
        while continue_capture:

            # Initiate interactive pyplot display
            ax = plt.axes()
            im = ax.imshow(image) # Image is global
            plt.ion()

            # Acknowledge that the display has been created
            global new_AOI_display
            new_AOI_display = False # AOI has been acknowledged

            # Start measuring the FPS
            start =  time.time() # For calculating framerate
            frame_counter = 0
            while continue_capture and not new_AOI_display:

               im.set_data(image) #
               if not np.isnan(center[0]):
                   plt.cla()
                   im = ax.imshow(image) # Repainting to prevent old center position from staying on screen
                   plt.plot(center[0],center[1],'x',color='r')
               plt.pause(0.0001)
               frame_counter +=1
            # CLose down the livedisplay and print FPS
            end = time.time()
            print("Fps for livefeed is ",frame_counter/(end-start))
            plt.ioff()
            plt.close()
class MotorThread(threading.Thread):
    """
    Thread in which a motor is controlled. The motor object is available globally.
    """
    def __init__(self, threadID, name,motor,axis):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.motor = motor
      self.axis = axis # 0 = x-axis, 1 = y axis
      self.setDaemon(True)
    def run(self):
       print("Running motor thread")
       global motor_running
       while motor_running:
           # Acquire lock to ensure that it is safe to move the motor
           motor_locks[self.axis].acquire()
           # Move motor to specified position
           # Check if trap should be moved
           global center
           if np.abs(center[self.axis]-trap_1_relative[self.axis])>=movement_threshold and not np.isnan(center[self.axis]):
                TM.MoveMotorToPixel(self.motor ,targetPixel=center[self.axis],currentPixel=trap_1_relative[self.axis])
           motor_locks[self.axis].release()
           time.sleep(0.05) # To give other threads some time to work
       return
class CameraThread(threading.Thread):
    # Add camera to init method here?
   def __init__(self, threadID, name):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.setDaemon(True)
   def run(self):

       print("Initiating threaded capture sequence")
       image_count = 0
       batch_size = 100 # Pretty good size for high performance
       number_images_saved = 0 # counts

       while continue_capture:
           # Set defaults for camera, aknowledge that this has been done
           cam.set_defaults(left=AOI[0],right=AOI[1],top=AOI[2],bot=AOI[3])
           global new_AOI_camera
           new_AOI_camera = False

           # Grab one example image
           global image
           image = cam.grab_image(n_frames=1)

           # Start livefeed from the camera
           cam.start_live_video() # Maximum framerate
           start = time.time()

           # Create an array to store the images which have been captured in
           saved_images = np.zeros([batch_size,np.abs(AOI[0]-AOI[1]),np.abs(AOI[3]-AOI[2])])

           while continue_capture and not new_AOI_camera:

               cam.wait_for_frame(timeout=None)

               # Capture an image and update the image count
               camera_lock.acquire()
               image[:][:][:] = cam.latest_frame()
               camera_lock.release()
               image_count = image_count+1

               if record:
                   # TODO Create option for recording video instead of simple images
                   saved_images[number_images_saved%batch_size] = copy.copy(image) # Do not want a reference but a copy of its own
                   number_images_saved+=1
                   if number_images_saved%batch_size==0 and number_images_saved>1:
                       np.save("test_images/frames"+str(number_images_saved-batch_size)+"_"+str(number_images_saved),np.uint8(saved_images))
           # Close the livefeed and calculate the fps of the captures
           end = time.time()
           cam.stop_live_video()
           print("Capture sequence finished",image_count, "Images captured in ",end-start,"seconds. \n FPS is ",image_count/(end-start))
class TrackingThread(threading.Thread):
   """
   Thread which does the tracking
   """
   def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.setDaemon(True)
   def run(self):
       global camera_lock
       global center
       global image
       global zoomed_in
       network1 = load_model(network_path+'network1.h5')
       network2 = load_model(network_path+'network_101x101_2.h5')
       i = 0
       while continue_capture:

           if i<100:
               camera_lock.acquire() # Might not need the lock
               center[0],center[1],picture = fpt.find_single_particle_center(copy.copy(image),threshold = 150) # Do we need a copy of this or move it to another thread?s
               camera_lock.release()

           if i>=100 and not zoomed_in and is_trapped(): # Change to a proper lock-on check
               zoomed_in =  True
               half_image_width = 50
               movement_threshold = 10
               set_AOI(half_image_width=half_image_width)
               #record = False
           if i>100 and zoomed_in:
               predict_particle_position(network2,print_position=False)

           time.sleep(0.1) # Needed to prevent this thread from running too fast
           i+=1
def set_AOI(half_image_width=50):
    """
    Function for changing the Area Of Interest for the camera to the box specified by
    left,right,top,bottom
    Assumes global access to
    """

    # Ensure that all global variables needed are available
    global motor_locks
    global new_AOI_camera
    global new_AOI_display
    global AOI
    global trap_1_relative

    # Do not want motors to be moving when changing AOI!
    motor_locks[0].acquire()
    motor_locks[1].acquire()

    # Update the area of interest
    AOI = [trap_1_relative[0]-half_image_width, trap_1_relative[0]+half_image_width, trap_1_relative[1]-half_image_width, trap_1_relative[1]+half_image_width]# +1 due to deeptrack oddity
    print("Setting AOI to ",AOI)

    # Inform the camera and display thread about the updated AOI
    new_AOI_camera = True
    new_AOI_display = True

    # Update trap relative position
    trap_1_relative[0] = trap_1_absolute[0]- AOI[0]
    trap_1_relative[1] = trap_1_absolute[1]- AOI[2]
    center = [trap_1_relative[0],trap_1_relative[1]] # Don't want the motors to move just yet
    print("trap 1 rel , center",trap_1_relative,center)

    time.sleep(0.2) # Give motor threads time to catch up
    motor_locks[0].release()
    motor_locks[1].release()
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
    #  TODO - Read half_image_width from the image
    resized = cv2.resize(copy.copy(image), (network_image_width,network_image_width), interpolation = cv2.INTER_AREA)
    pred = network.predict(np.reshape(resized/255,[1,network_image_width,network_image_width,1]))

    camera_lock.acquire() # Might not need the lock
    center[0] = half_image_width+pred[0][1]*half_image_width
    center[1] = half_image_width+pred[0][0]*half_image_width
    camera_lock.release()

    if print_position:
        print("Predicted posiiton is ",center)
def is_trapped(threshold_distance=50):
    """
    Function for detemining if a particle has been trapped or not
    """
    global center
    global trap_1_relative
    distance = np.sqrt((center[0]-trap_1_relative[0])**2 + (center[1]-trap_1_relative[1])**2)
    return distance<threshold_distance

############### Main script starts here ####################################

# Serual numbers for motors
serial_num_X = '27502438'
serial_num_Y = '27502419'
network_path = 'C:/Martin/Networks/'
inputQueue = queue.Queue()
polling_rate = 100 # How often to speak to the motor(ms)

# Initate contact with motors
#TODO move these to the thread
motor_X = TM.InitateMotor(serial_num_X,pollingRate=polling_rate)
motor_Y = TM.InitateMotor(serial_num_Y,pollingRate=polling_rate)

# Create camera and set defaults
cam = TC.get_camera()
half_image_width =500
AOI = [0,2*half_image_width,0,2*half_image_width]
cam.set_defaults(left=AOI[0],right=AOI[1],top=AOI[2],bot=AOI[3],n_frames=1)
exposure_time = TC.find_exposure_time(cam) # automagically finds a decent exposure time
print("Exposure time = ",exposure_time)

# Capture an example image to work with
image = cam.grab_image()

# Set parameters for the various threads

# TODO replace this with a dictionary perhaps
continue_capture = True # True if camera and dispaly should keep updating
motor_running = True # Should the motor thread keep running?
zoomed_in = False # Keeps track of whether the image is cropped or not
record = False # Default
new_AOI_camera = False
new_AOI_display = False
movement_threshold = 40

trap_1_absolute = [520,580]# Position of trap relative to (0,0) of full camera frame
trap_1_relative = [trap_1_absolute[0],trap_1_absolute[1]] # position of trap when image is cropped, initally it is not cropped


center = [400,400]
center[0],center[1],picture = fpt.find_single_particle_center(copy.copy(image),threshold = 150) # Do we need a copy of this or move it to another thread?s
center[0],center[1] = trap_1_relative[0],trap_1_relative[1] # Do not want it wandering on its own

camera_lock = threading.Lock()
motor_locks = [threading.Lock(),threading.Lock()]

# Create a empty list to put the threads in
thread_list = []

# Create some buttons for basic control
top = tkinter.Tk()
B = tkinter.Button(top, text ="Exit program", command = terminate_threads)
B2 = tkinter.Button(top, text ="Start program", command = start_threads)
# TODO add buttons for left right up and down movement as well as zoom in
B.pack()
B2.pack()
top.mainloop()

# Close the threads and the camera
continue_capture = False # All threds exits their main loop once this parameter is changed
motor_running = False

# Shut down camera and motors safely
cam.close()

TM.DisconnectMotor(motor_X)
TM.DisconnectMotor(motor_Y)
print(thread_list)
sys.exit()
