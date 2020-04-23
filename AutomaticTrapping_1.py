# Script for controlling the whole setup automagically
import ThorlabsCam as TC
import ThorlabsMotor as TM
import find_particle_threshold as fpt
from instrumental import u
import matplotlib.pyplot as plt
import numpy as np
import threading,time
import copy

new_AOI_camera = False # Is there a new Area Of Interest in the image so that
new_AOI_display = False
# the camera options and livefeed should be updated?

class DisplayThread(threading.Thread):
    """
    Thread class for plotting the result in the BG.
    InitiateMotorreally appreciated by python
    """
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name

    def run(self):
        while continue_capture:
            ax = plt.axes()
            im = ax.imshow(image) # Image is global
            global new_AOI_display
            new_AOI_display = False # AOI has been acknowledged
            plt.ion()
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
    def run(self):
       print("Running motor thread")
       while motor_running:

           # Acquire lock to ensure that it is safe to move the motor
           motor_locks[self.axis].acquire()
           # Move motor to specified position
           # Check if trap should be moved
           global center
           if np.abs(center[self.axis]-trap_1_relative[self.axis])>=movement_threshold and not np.isnan(center[self.axis]):
                #print("Moving motor",self.name,center[self.axis],trap_1_relative[self.axis])
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

   def run(self):

       print("Initiating threaded capture sequence")
       image_count = 0
       batch_size = 100 # Pretty good size for high performance
       number_images_saved = 0 # counts

       while continue_capture:
           # Set defaults for camera, aknowledge that this has been done
           cam.set_defaults(left=AOI[0],right=AOI[1],top=AOI[2],bot=AOI[3])
           global new_AOI_camera
           new_AOI_camera = False # Need to broadcast this
           global image
           image = cam.grab_image(n_frames=1)
           # Start livefeed from the camera
           cam.start_live_video() # Maximum framerate
           start = time.time()

           saved_images = np.zeros([batch_size,np.abs(AOI[0]-AOI[1]),np.abs(AOI[3]-AOI[2])])
           print(np.shape(saved_images))

           while continue_capture and not new_AOI_camera:
               cam.wait_for_frame(timeout=None)
               camera_lock.acquire()
               image[:][:][:] = cam.latest_frame()
               camera_lock.release()
               image_count = image_count+1
               if record:
                   # TODO There seem to be an error here with not all images being saved and the same images getting saved multiple times
                   # Needs to be investigated ASAP
                   saved_images[number_images_saved%batch_size] = copy.copy(image) # Believe that the error was the usage of i instead of number_images_saved here
                   # Test and verify this please
                   number_images_saved+=1
                   if number_images_saved%batch_size==0 and number_images_saved>1:
                       np.save("test_images/frames"+str(number_images_saved-batch_size)+"_"+str(number_images_saved),np.uint8(saved_images))
           # Close the livefeed and calculate the fps of the captures
           end = time.time()
           cam.stop_live_video()
           print("Capture sequence finished",image_count, "Images captured in ",end-start,"seconds. \n FPS is ",image_count/(end-start))
def set_AOI(left,right,top,bottom):
    """
    Function for changing the Area Of Interest for the camera to the box specified by
    left,right,top,bottom
    Assumes global access to
    """
    motor_locks.acquire() # Do not want motors to be moving at this stage!
    AOI = [left,right,top,bottom]
    print("Setting AOI to ",AOI)
    global new_AOI_camera
    global new_AOI_display
    new_AOI_camera = True
    new_AOI_display = True
    # Update trap relative position
    trap_1_relative[0] = trap_1_absolute[0]- left
    trap_1_relative[1] = trap_1_absolute[1]- top
    center = [trap_1_relative[0],trap_1_relative[1]] # Don't want the motors to move just yet, therfore setting this to center
    print("trap 1 rel , center",trap_1_relative,center)
    movement_threshold = 5

    motor_locks.release()
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
polling_rate = 100 # How often to speak to the motor(ms)

# Initate contact with motors
motor_X = TM.InitiateMotor(serial_num_X,pollingRate=polling_rate)
motor_Y = TM.InitiateMotor(serial_num_Y,pollingRate=polling_rate)

# Create camera and set defaults
cam = TC.get_camera()
timeout = 3000*u.ms
image_width =1000
AOI = [0,image_width,0,image_width]

cam.set_defaults(left=AOI[0],right=AOI[1],top=AOI[2],bot=AOI[3],n_frames=1)
# Find and set a suitable exposure time for the camera
exposure_time = TC.find_exposure_time(cam)
print("Exposure time = ",exposure_time)

# Capture an example image to work with
image = cam.grab_image()
cx,cy,picture_tmp = fpt.find_single_particle_center(image,threshold = 120) # Initiate default values
continue_capture = True
motor_running = True
record = True # Default

nbr_frames = 200
trap_1_absolute = [520,580]# Position of trap in absolute terms
trap_1_relative = [trap_1_absolute[0],trap_1_absolute[1]] # position of trap when image si cropped

movement_threshold = 40
center = [400,400]
center[0],center[1],picture = fpt.find_single_particle_center(copy.copy(image),threshold = 150) # Do we need a copy of this or move it to another thread?s

camera_lock = threading.Lock()
motor_locks = [threading.Lock(),threading.Lock()]
# Create and start a camera thread
camera_thread = CameraThread(1, "Thread-camera")
camera_thread.start()

# Create and start motor threads
motor_X_thread = MotorThread(2,"Thread-motorX",motor_X,0) # Last argument is to indicate that it is the x-motor and not the y
motor_Y_thread = MotorThread(3,"Thread-motorY",motor_Y,1)
display_thread = DisplayThread(4,"Thread-display")

motor_X_thread.start()
motor_Y_thread.start()

# Set the number of frames to capture
start =  time.time() # For calculating framerate
display_thread.start()
for i in range(nbr_frames):
    camera_lock.acquire() # Might not need the lock
    center[0],center[1],picture = fpt.find_single_particle_center(copy.copy(image),threshold = 150) # Do we need a copy of this or move it to another thread?s
    camera_lock.release()
    print(center)

    if i==100 : # Change to a proper lock-on check

        # set_AOI(trap_1_absolute[0]-new_image_width,trap_1_absolute[0]+new_image_width,trap_1_absolute[1]-new_image_width,trap_1_absolute[1]+new_image_width)

        motor_locks[0].acquire() # Do not want motors to be moving when changing AOI!
        motor_locks[1].acquire()
        new_image_width = 50
        AOI = [trap_1_relative[0]-new_image_width,trap_1_relative[0]+new_image_width,trap_1_relative[1]-new_image_width,trap_1_relative[1]+new_image_width]
        print("Setting AOI to ",AOI)
        new_AOI_camera = True
        new_AOI_display = True
        # Update trap relative position
        trap_1_relative[0] = trap_1_absolute[0]- AOI[0]
        trap_1_relative[1] = trap_1_absolute[1]- AOI[2]
        center = [trap_1_relative[0],trap_1_relative[1]] # Don't want the motors to move just yet
        print("trap 1 rel , center",trap_1_relative,center)
        movement_threshold = 5
        time.sleep(0.2) # Give motor threads time to catch up
        motor_locks[0].release()
        motor_locks[1].release()

        # Set record true if you are to capture data
        record = False
    time.sleep(0.1) # Needed to prevent this thread from running too fast
end = time.time()

# Close the threads and the camera
continue_capture = False # All threds exits their main loop once this parameter is changed
motor_running = False

camera_thread.join()
motor_X_thread.join()
motor_Y_thread.join()
display_thread.join()
cam.close()

plt.show()

# Shut down camera and motors safely
TM.DisconnectMotor(motor_X)
TM.DisconnectMotor(motor_Y)
