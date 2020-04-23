# Script for controlling the whole setup automagically
import ThorlabsCam as TC
import SLM
import ThorlabsMotor as TM
import find_particle_threshold as fpt
from instrumental import u
import matplotlib.pyplot as plt
import numpy as np
# from keras.models import load_model # Not curently in use
import threading,time,cv2,queue,copy,sys,tkinter,os
from tkinter import messagebox
from functools import partial
import datetime
from cv2 import VideoWriter, VideoWriter_fourcc
import PIL.Image, PIL.ImageTk

def get_default_control_parameters(recording_path=None):
    # TODO : change this into a class
    if recording_path == None:
        now = datetime.datetime.now()
        recording_path = 'F:/Martin/D'+str(now.year)+'-'+str(now.month)+'-'+str(now.day)
        try:
            os.mkdir(recording_path)
        except:
            print('Directory already exist')
    control_parameters = {
    'serial_num_X': '27502438',
    'serial_num_Y': '27502419',
    'serial_no_piezo':'97100532',
    'channel':1,
    'network_path': 'C:/Martin/Networks/',
    'recording_path':recording_path,
    'polling_rate': 100,
    'continue_capture': True, # True if camera, dispaly, etc should keep updating
    'motor_running': True, # Should the motor thread keep running?
    'zoomed_in': False, # Keeps track of whether the image is cropped or not
    'record': False, # Default
    'half_image_width':500, # TODO remove this parameter, should not be needed
    'AOI':[0,1200,0,1000],
    'new_AOI_camera': False,
    'new_AOI_display': False,
    'movement_threshold': 40,
    'record':False,
    'tracking_on':True,
    'target_temperature':25,

    'traps_absolute_pos':[[923,925,1048],[650,506,583]], #[500,500],# Change to aritrary number of traps
    'traps_relative_pos':[[923,925,1048],[650,506,583]], #[500,500],
    'particle_centers': [[500],[500]], #[400,400],
    'target_particle_center':[500,500], # Position of the particle we currently are trying to trap
    # Used to minimize changes in code when updating to multiparticle tracking
    'target_trap_pos':[500,500],# Position of the trap we currently are trying to trap in
    'motor_movements':[0,0], # How much x and y motor should be moved
    'z_starting_position':0, # Where the experiments starts in z position
    'z_movement':0, # Target z-movement in "ticks" positive for up, negative for down
    'return_z_home':False,
    'particle_threshold':120,
    'particle_size_threshold':200, # Parcticle detection threshold
    'bright_particle':True, # Is particle brighter than the background?
    'jog_motor_in_direction':[False,False,False,False], # TODO Remove this thiug
    # # TODO, make this from the number of traps
    'camera_lock': threading.Lock(),
    'motor_locks': [threading.Lock(),threading.Lock()]
    }
    control_parameters['traps_occupied'] = [False for i in range(len(control_parameters['traps_absolute_pos'][0]))]
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
    global motor_X
    global motor_Y
    global thread_list
    # global temperature_controller
    camera_thread = CameraThread(1, 'Thread-camera')
    motor_X_thread = MotorThread(2,'Thread-motorX',0) # Last argument is to indicate that it is the x-motor and not the y
    motor_Y_thread = MotorThread(3,'Thread-motorY',1)
    slm_thread =SLMThread(4,'Thread-SLM')
    tracking_thread = TrackingThread(5,'Tracker_thread')
    #temperature_thread = TemperatureThread(6,'Temperature_thread')
    z_thread = z_movement_thread(6, 'z-thread',serial_no=control_parameters['serial_no_piezo'],channel=control_parameters['channel'])

    camera_thread.start()
    motor_X_thread.start()
    motor_Y_thread.start()
    tracking_thread.start()
    slm_thread.start()
    z_thread.start()
    # temperature_thread.start()
    print('Camera, SLM, tracking, motor_X and motor_Y threads created')
    # thread_list.append(temperature_thread)
    thread_list.append(camera_thread)
    thread_list.append(motor_X_thread)
    thread_list.append(motor_Y_thread)
    thread_list.append(tracking_thread)
    thread_list.append(slm_thread)
    thread_list.append(z_thread)
def create_buttons(top):
    global control_parameters
    exit_button = tkinter.Button(top, text ='Exit program', command = terminate_threads)
    #start_button = tkinter.Button(top, text ='Start program', command = start_threads)
    up_button = tkinter.Button(top, text ='Move up', command = partial(move_button,0))
    down_button = tkinter.Button(top, text ='Move down', command = partial(move_button,1))
    right_button = tkinter.Button(top, text ='Move right', command = partial(move_button,2))
    left_button = tkinter.Button(top, text ='Move left', command = partial(move_button,3))
    start_record_button = tkinter.Button(top, text ='Start recording', command = start_record)
    stop_record_button = tkinter.Button(top, text ='Stop recording', command = stop_record)
    toggle_bright_particle_button = tkinter.Button(top, text ='Toggle particle brightness', command = toggle_bright_particle)
    threshold_entry = tkinter.Entry(top, bd =5)
    toggle_tracking_button = tkinter.Button(top, text ='Toggle particle tracking', command = toggle_tracking)
    def set_threhold():
        entry = threshold_entry.get()
        try:
            threshold = int(entry)
            if 0<threshold<255:
                control_parameters['particle_threshold'] = threshold
                print("Threshold set to ",threshold)
            else:
                print('Threshold out of bounds')
        except:
            print('Cannot convert entry to integer')
        threshold_entry.delete(0,last=5000)
    threshold_button = tkinter.Button(top, text ='Set threshold', command = set_threhold)
    # Idea - Use radiobutton for the toggle
    # TODO add button for zoom in
    x_position = 1220
    exit_button.place(x=x_position, y=50)
    #start_button.place(x=x_position, y=50)
    up_button.place(x=x_position, y=90)
    down_button.place(x=x_position, y=130)
    right_button.place(x=x_position, y=170)
    left_button.place(x=x_position, y=210)
    start_record_button.place(x=x_position, y=250)
    stop_record_button.place(x=x_position, y=290)
    toggle_bright_particle_button.place(x=x_position, y=330)
    threshold_entry.place(x=x_position,y=400)
    threshold_button.place(x=x_position+100,y=400)
    toggle_tracking_button.place(x=x_position,y=440)
class SLMThread(threading.Thread):
    def __init__(self,threadID,name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.setDaemon(True)
    def run(self):
        global control_parameters
        Delta,N,M = SLM.get_delta()
        SLM_image = SLM.GSW(N,M,Delta)
        SLM.setup_fullscreen_plt_image()
        plt.imshow(SLM_image,cmap='gist_gray')
        while control_parameters['continue_capture']:
            plt.pause(50000)
class TemperatureThread(threading.Thread):
        '''
        Class for running the temperature controller in the background
        '''
        def __init__(self, threadID, name,temperature_controller):
            threading.Thread.__init__(self)
            self.threadID = threadID
            self.name = name
            self.temperature_controller = temperature_controller
            self.setDaemon(True)
        def run(self):
            global control_parameters

            # Turn on output and continuosly set and query the temperature.
            self.temperature_controller.turn_on_output()
            while control_parameters['continue_capture']:
                self.temperature_controller.set_setpoint_temperature(control_parameters['target_temperature'])
                print('Current temperature is ',self.temperature_controller.measure_temperature())
                time.sleep(1) # We do not need to update the temperature very often
            self.temperature_controller.turn_off_output()
class TkinterDisplay:

    def __init__(self, window, window_title,):
         self.window = window
         self.window.title(window_title)

         # Create a canvas that can fit the above video source size
         self.canvas = tkinter.Canvas(window, width = 1200, height = 1000)
         self.canvas.place(x=0, y=0)

         # Button that lets the user take a snapshot
         self.btn_snapshot=tkinter.Button(window, text="Snapshot", width=50, command=self.snapshot)
         self.btn_snapshot.place(x=1300, y=0)
         create_buttons(self.window)
         self.window.geometry('1500x1000')
         # After it is called once, the update method will be automatically called every delay milliseconds
         self.delay = 50
         self.update()
         start_threads()
         self.window.mainloop()

    def snapshot(self):
         global image
         global control_parameters
         cv2.imwrite(control_parameters['recording_path']+"/frame-" + time.strftime("%d-%m-%Y-%H-%M-%S") + ".jpg", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

    def update(self):
         # Get a frame from the video source
         global image
         # global control_parameters
         # #TODO? Might wanna do some rescaling of the image so we can zoom in

         # tmp_image = fpt.threshold_image(copy.copy(image),
         #                                  threshold=control_parameters['particle_threshold'],
         #                                  bright_particle=control_parameters['bright_particle'])
         self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(image))
         self.canvas.create_image(0, 0, image = self.photo, anchor = tkinter.NW) # need to use a compatible image type

         self.window.after(self.delay, self.update)
class DisplayThread(threading.Thread):
    '''
    Thread class for plotting the result in the BG.
    InitiateMotorreally appreciated by python
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
               if not np.isnan(control_parameters['target_particle_center'][0]):
                   plt.cla()
                   im = ax.imshow(image,cmap='gray',vmin=0, vmax=255) # Repainting to prevent old center position from staying on screen
                   if control_parameters['target_particle_center'][0]<(control_parameters['AOI'][1]-control_parameters['AOI'][0]) and control_parameters['target_particle_center'][1]<(control_parameters['AOI'][3]-control_parameters['AOI'][2]) : # Accidentally doing stuff wrong here

                       plt.plot(control_parameters['particle_centers'][0],control_parameters['particle_centers'][1],'x',color='r')
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
    def __init__(self, threadID, name,axis):

      threading.Thread.__init__(self)
      global control_parameters
      self.threadID = threadID
      self.name = name
      self.axis = axis # 0 = x-axis, 1 = y axis
      if self.axis==0:
          self.motor = TM.InitiateMotor(control_parameters['serial_num_X'],pollingRate=control_parameters['polling_rate'])
      elif self.axis==1:
          self.motor = TM.InitiateMotor(control_parameters['serial_num_Y'],pollingRate=control_parameters['polling_rate'])
      else:
          print('Invalid choice of axis, no motor available')

      self.setDaemon(True)
    def run(self):
       print('Running motor thread')
       global control_parameters
       while control_parameters['motor_running']:
            # Acquire lock to ensure that it is safe to move the motor
            control_parameters['motor_locks'][self.axis].acquire()

            # Check if user is jogging the motor
            if control_parameters['jog_motor_in_direction'][self.axis*2]:
               TM.MoveMotor(self.motor,0.03)
               control_parameters['jog_motor_in_direction'][self.axis*2] = False
               print("User jogging motor")
            elif control_parameters['jog_motor_in_direction'][self.axis*2+1]:
               TM.MoveMotor(self.motor,-0.03)
               control_parameters['jog_motor_in_direction'][self.axis*2+1] = False
               print("User jogging motor")
            # Check that there is a target particle and if so where to move

            elif np.abs(control_parameters['motor_movements'][self.axis]):
                TM.MoveMotorPixels(self.motor,control_parameters['motor_movements'][self.axis])
                control_parameters['motor_movements'][self.axis] = 0
            '''
            elif len(control_parameters['target_particle_center'])>0:
               if np.abs(control_parameters['target_particle_center'][self.axis]-control_parameters['target_trap_pos'][self.axis])>=control_parameters['movement_threshold']:
                    maxPixel = np.abs(control_parameters['AOI'][1]-control_parameters['AOI'][0])
                    TM.MoveMotorToPixel(self.motor ,targetPixel=control_parameters['target_particle_center'][self.axis],currentPixel=control_parameters['target_trap_pos'][self.axis],maxPixel=maxPixel)
            '''
            control_parameters['motor_locks'][self.axis].release()
            time.sleep(0.1) # To give other threads some time to work

       TM.DisconnectMotor(self.motor)
class z_movement_thread(threading.Thread):
    '''
    Thread for controling movement of the objective in z-direction
    '''
    def __init__(self, threadID, name,serial_no,channel,polling_rate=250):
        threading.Thread.__init__(self)
        global control_parameters
        self.threadID = threadID
        self.name = name
        self.piezo = TM.PiezoMotor(serial_no,channel=channel,pollingRate=polling_rate)
        control_parameters['z_starting_position'] = self.piezo.get_position()
        self.setDaemon(True)
    def run(self):
        global control_parameters

        while control_parameters['continue_capture']:

            # Check if the objective should be moved
            if control_parameters['z_movement'] is not 0:
                try:
                    control_parameters['z_movement'] = int(control_parameters['z_movement'])
                    # Move up if we are not already up
                    if self.piezo.get_position()<control_parameters['z_starting_position']+300:
                        self.piezo.move_relative(control_parameters['z_movement'])
                    control_parameters['z_movement'] = 0
                except:
                    print('Cannot move objective to',control_parameters['z_movement'] )
                    print('Resetting target z movement.')
                    control_parameters['z_movement'] = 0
            elif control_parameters['return_z_home']:
                self.piezo.move_to_position(control_parameters['z_starting_position'])
                control_parameters['return_z_home'] = False
                print('homing z')
            time.sleep(0.2)

        self.piezo.move_to_position(control_parameters['z_starting_position'])
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
       video_created = False
       # TODO - set framreate so we have a proper framerate in the videos!
       # Also need to record the framerate etc
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
           if not video_created:
               now = datetime.datetime.now()
               fourcc = VideoWriter_fourcc(*'MP42')
               image_width = control_parameters['AOI'][1]-control_parameters['AOI'][0]
               image_height = control_parameters['AOI'][3]-control_parameters['AOI'][2]
               video = VideoWriter(control_parameters['recording_path']+'/moive-'+str(now.hour)+'-'+str(now.minute)+'-'+str(now.second)+'.avi', fourcc, float(10), (image_width, image_height),isColor=False)
               video_created = True
           # Start continously capturin images now that the camera parameters have been set
           while control_parameters['continue_capture'] and not control_parameters['new_AOI_camera']:
               cam.wait_for_frame(timeout=None)

               # Capture an image and update the image count
               control_parameters['camera_lock'].acquire()
               image[:][:][:] = cam.latest_frame()
               control_parameters['camera_lock'].release()
               image_count = image_count+1
               if control_parameters['record']:
                   video.write(image)
                   # Try to do this in the background

           video.release()

           del video
           video_created = False
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
       # network1 = load_model(control_parameters['network_path']+'network1.h5')
       # network2 = load_model(control_parameters['network_path']+'network_101x101_2.h5')
       zoom_counter = 0
       while control_parameters['continue_capture']: # Change to continue tracking?

            if control_parameters['tracking_on']:
               if not control_parameters['zoomed_in']:
                   '''
                   We are in full frame mode looking for a particle
                   '''
                   x,y = fpt.find_particle_centers(copy.copy(image),
                                                    threshold=control_parameters['particle_threshold'],
                                                    particle_size_threshold=control_parameters['particle_size_threshold'],
                                                    bright_particle=control_parameters['bright_particle'])
                   control_parameters['particle_centers'] = [x,y]

                   # Find the closest particles
                   if len(x)>0: # Check that there are particles present
                       # Todo add motor lock here?
                       min_index_trap,min_index_particle = find_closest_unoccupied()
                       if min_index_particle is not None:
                           # Get motor locks?
                           control_parameters['target_trap_pos'] = [control_parameters['traps_relative_pos'][0][min_index_trap],control_parameters['traps_relative_pos'][1][min_index_trap]]
                           control_parameters['target_particle_center'] = [control_parameters['particle_centers'][0][min_index_particle],control_parameters['particle_centers'][1][min_index_particle]]
                           control_parameters['motor_movements'][0] = control_parameters['target_trap_pos'][0] - control_parameters['target_particle_center'][0]
                           control_parameters['motor_movements'][1] = control_parameters['target_trap_pos'][1] - control_parameters['target_particle_center'][1]

                           # If there is a trapped particle then we do not want to move very far so we accidentally lose it
                           if True in control_parameters['traps_occupied']:
                               limit_motor_movement(limit=40)
                       if False in control_parameters['traps_occupied']:
                           control_parameters['return_z_home'] = True
                       else:
                           print("All the traps have been occupied")
                           control_parameters['z_movement'] = 10
                           control_parameters['return_z_home'] = False
                   else:
                       control_parameters['target_particle_center'] = []
               #print("Centers are",control_parameters['particle_centers'])
            time.sleep(0.1) # Needed to prevent this thread from running too fast
def limit_motor_movement(limit=100):
   if control_parameters['motor_movements'][0]>limit:
       control_parameters['motor_movements'][0] = limit
   elif control_parameters['motor_movements'][0]<-limit:
       control_parameters['motor_movements'][0] = -limit
   if control_parameters['motor_movements'][1]>limit:
       control_parameters['motor_movements'][1] = limit
   elif control_parameters['motor_movements'][1]<-limit:
       control_parameters['motor_movements'][1] = -limit
def set_AOI(half_image_width=50,left=None,right=None,up=None,down=None):
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

        # If exact values have been provided for all the
        # TODO change so that this syntax is default
        if left is not None and right is not None and up is not None and down is not None:
            if 0<=left<=1279 and left<=right<=1280 and 0<=up<=1079 and up<=down<=1080:
                control_parameters['AOI'][0] = left
                control_parameters['AOI'][1] = right
                control_parameters['AOI'][2] = up
                control_parameters['AOI'][3] = down
            else:
                print("Trying to set invalid area")
        else:
            control_parameters['AOI'] = [control_parameters['traps_relative_pos'][0]-half_image_width, control_parameters['traps_relative_pos'][0]+half_image_width, control_parameters['traps_relative_pos'][1]-half_image_width, control_parameters['traps_relative_pos'][1]+half_image_width]# +1 due to deeptrack oddity
    else:
        # Use defult center
        control_parameters['AOI'] = [0,2*half_image_width,0,2*half_image_width]
    print('Setting AOI to ',control_parameters['AOI'])

    # Inform the camera and display thread about the updated AOI
    control_parameters['new_AOI_camera'] = True
    control_parameters['new_AOI_display'] = True

    # Update trap relative position
    control_parameters['traps_relative_pos'][0] = control_parameters['traps_absolute_pos'][0]- control_parameters['AOI'][0]
    control_parameters['traps_relative_pos'][1] = control_parameters['traps_absolute_pos'][1]- control_parameters['AOI'][2]
    control_parameters['target_particle_center'] = [control_parameters['traps_relative_pos'][0],control_parameters['traps_relative_pos'][1]] # Don't want the motors to move just yet
    #print('trap 1 rel , center',control_parameters['traps_relative_pos'],control_parameters['particle_centers'])

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
    control_parameters['target_particle_center'][0] = half_image_width+pred[0][1]*half_image_width
    control_parameters['target_particle_center'][1] = half_image_width+pred[0][0]*half_image_width
    control_parameters['camera_lock'].release()

    if print_position:
        print('Predicted posiiton is ',control_parameters['particle_centers'])
def get_particle_trap_distances():
    '''
    Calcualtes the distance between all particles and traps and returns a distance matrix,
    ordered as distances(traps,particles),
        To clarify the distance between trap n and particle m is distances[n][m
        ]

    '''
    global control_parameters
    nbr_traps = len(control_parameters['traps_absolute_pos'][0])
    nbr_particles = len(control_parameters['particle_centers'][0])
    distances = np.ones((nbr_traps,nbr_particles))
    for i in range(nbr_traps):
        for j in range(nbr_particles):
            dx = (control_parameters['traps_relative_pos'][0][i]-control_parameters['particle_centers'][0][j])
            dy = (control_parameters['traps_relative_pos'][1][i]-control_parameters['particle_centers'][1][j])
            distances[i,j] = np.sqrt(dx*dx+dy*dy)
            #distances[i][j] = np.sqrt((control_parameters['traps_relative_pos'][0][i]-control_parameters['particle_centers'][0][j])**2+(control_parameters['traps_relative_pos'][1][i]-control_parameters['particle_centers'][1][j])**2)
    return distances
def trap_occupied(distances,trap_index):
    '''
    Checks if a specific trap is occupied by a particle. If so set that trap to occupied
    '''
    global control_parameters

    # Check that trap index is ok
    if trap_index>len(control_parameters['traps_occupied']):
        print('Trap index out of range')
        return None
    for i in range(len(distances[trap_index,:])):
        dist_to_trap = distances[trap_index,i]
        if dist_to_trap<=control_parameters['movement_threshold']:
            control_parameters['traps_occupied'][trap_index] = True
            return i
    control_parameters['traps_occupied'][trap_index] = False
    return None
def check_all_traps(distances=None):
    '''
    Updates all traps to see if they are occupied.
    Returns the indices of the particles which are trapped. Indices refers to their
    position in the control_parameters['particle_centers'] array.
    Returns an empty array if there are no trapped particles
    '''
    if distances is None:
        distances = get_particle_trap_distances()
    trapped_particle_indices = []
    for trap_index in range(len(distances)):
        trapped_particle_index = trap_occupied(distances,trap_index)
        if trapped_particle_index is not None:
            trapped_particle_indices.append(trapped_particle_index)
    return trapped_particle_indices
def find_closest_unoccupied():
    '''
    Function for finding the paricle and (unoccupied) trap which are the closest
    Returns : min_index_trap,min_index_particle.
        Index of the untrapped particle closest to an unoccupied trap.
    '''

    distances = get_particle_trap_distances()
    trapped_particles = check_all_traps(distances)
    distances[:,trapped_particles] = 1e6 # These indices are not ok, put them very large

    min_distance = 2000 # If the particle is not within 2000 pixels then it is not within the frame
    min_index_particle = None # Index of particle which is closes to an unoccupied trap
    min_index_trap = None # Index of unoccupied trap which is closest to a particle

    # Check all the traps
    for trap_idx in range(len(control_parameters['traps_occupied'])):
        trapped = control_parameters['traps_occupied'][trap_idx]

        # If there is not a particle trapped in the trap check for the closest particle
        if not trapped:
            particle_idx = np.argmin(distances[trap_idx])

            # If particle is within the threshold then update min_index and trap index as well as min distance
            if distances[trap_idx,particle_idx]<min_distance:
                min_distance = distances[trap_idx,particle_idx]
                min_index_trap = trap_idx
                min_index_particle = particle_idx

    return min_index_trap,min_index_particle
def move_button(move_direction):
    '''
    Button function for manually moving the motors a bit
    The direction refers to the direction a particle in the fiel of view will move on the screen
    move_direction = 0 => move up
    move_direction = 1 => move down
    move_direction = 2 => move right
    move_direction = 3 => move left
    '''
    global control_parameters

    if move_direction==0:
        # Move up (Particles in image move up on the screen)
        control_parameters['jog_motor_in_direction'][3]=True

    elif move_direction==1:
        control_parameters['jog_motor_in_direction'][2]=True
        # Move down
    elif move_direction==2:
        control_parameters['jog_motor_in_direction'][0]=True
        # Move right
    elif move_direction==3:
        control_parameters['jog_motor_in_direction'][1]=True
        # Move left
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
def toggle_bright_particle():
    '''
    Function for switching between bright and other particle
    '''
    control_parameters['bright_particle'] = not control_parameters['bright_particle']
    print("control_parameters['bright_particle'] set to",control_parameters['bright_particle'])
def set_particle_threshold():
    threshold=control_parameters['particle_threshold'] = threshold
    return
def toggle_tracking():
    control_parameters['tracking_on'] = not control_parameters['tracking_on']
    print("Tracking is ",control_parameters['tracking_on'])
############### Main script starts here ####################################
control_parameters = get_default_control_parameters()
# Create camera and set defaults
cam = TC.get_camera()
cam.set_defaults(left=control_parameters['AOI'][0],right=control_parameters['AOI'][1],top=control_parameters['AOI'][2],bot=control_parameters['AOI'][3],n_frames=1)
exposure_time = TC.find_exposure_time(cam) # automagically finds a decent exposure time
print('Exposure time = ',exposure_time)

# Capture an example image to work with
image = cam.grab_image()

#control_parameters['particle_centers'][0],control_parameters['particle_centers'][1] = fpt.find_particle_centers(copy.copy(image),threshold = 150) # Do we need a copy of this or move it to another thread?s
control_parameters['particle_centers'][0],control_parameters['particle_centers'][1] = control_parameters['traps_relative_pos'][0],control_parameters['traps_relative_pos'][1] # Do not want it wandering on its own

# Create a empty list to put the threads in
thread_list = []

T_D = TkinterDisplay(tkinter.Tk(), "Control display")

# Close the threads and the camera
control_parameters['continue_capture'] = False # All threds exits their main loop once this parameter is changed
control_parameters['motor_running'] = False

# Shut down camera and motors safely
terminate_threads()
cam.close()
print(thread_list)
sys.exit()
