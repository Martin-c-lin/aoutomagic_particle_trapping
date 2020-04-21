# Script for controlling the whole setup automagically
import ThorlabsCam as TC
#import ThorlabsMotor as TM
import find_particle_threshold as fpt
from instrumental import u
import matplotlib.pyplot as plt
import numpy as np
# from keras.models import load_model
import threading,time,cv2,queue,copy,sys,tkinter,os
from tkinter import messagebox,RIGHT,LEFT
from functools import partial
import datetime
from cv2 import VideoWriter, VideoWriter_fourcc
import PIL.Image, PIL.ImageTk
def get_default_control_parameters(recording_path=None):

    if recording_path == None:
        now = datetime.datetime.now()
        recording_path = 'F:/Martin/D'+str(now.year)+'-'+str(now.month)+'-'+str(now.day)+'_T'+str(now.hour)+'_'+str(now.minute)
        try:
            os.mkdir(recording_path)
        except:
            print('Directory already exist')
    control_parameters = {
    'trapped_counter': 0, # Maybe does not need to be here
    'serial_num_X': '27502438',
    'serial_num_Y': '27502419',
    'network_path': 'C:/Martin/Networks/',
    'recording_path':recording_path,
    'polling_rate': 100,
    'continue_capture': True, # True if camera, dispaly, etc should keep updating
    'motor_running': True, # Should the motor thread keep running?
    'zoomed_in': False, # Keeps track of whether the image is cropped or not
    'record': False, # Default
    'half_image_width':500, # TODO remove this parameter, should not be needed
    'AOI':[0,1000,0,1000],
    'new_AOI_camera': False,
    'new_AOI_display': False,
    'movement_threshold': 40,
    'record':False,
    'target_temperature':25,

    'traps_absolute_pos':[[500],[500]], #[500,500],# Change to aritrary number of traps
    'traps_relative_pos': [[500],[500]], #[500,500],
    'particle_centers': [[500],[500]], #[400,400],
    'target_particle_center':[500,500], # Position of the particle we currently are trying to trap
    # Used to minimize changes in code when updating to multiparticle tracking
    'target_trap_pos':[500,500],# Position of the trap we currently are trying to trap in
    'particle_threshold':120,
    'particle_size_threshold':100, # Parcticle detection threshold
    'bright_particle':True, # Is particle brighter than the background?
    'traps_occupied':[False,False,False], # Which of the traps have a particle inside them
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
    global motor_X
    global motor_Y
    # global temperature_controller
    camera_thread = CameraThread(1, 'Thread-camera')
    motor_X_thread = MotorThread(2,'Thread-motorX',motor_X,0) # Last argument is to indicate that it is the x-motor and not the y
    motor_Y_thread = MotorThread(3,'Thread-motorY',motor_Y,1)
    slm_thread =
    #display_thread = DisplayThread(4,'Thread-display')
    tracking_thread = TrackingThread(5,'Tracker_thread')
    #temperature_thread = TemperatureThread(6,'Temperature_thread')

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
    toggle_bright_particle_button = tkinter.Button(top, text ='Toggle particle brightness', command = toggle_bright_particle)
    # Idea - Use radiobutton for the toggle
    # TODO add button for zoom in
    x_position = 1020
    exit_button.place(x=x_position, y=10)
    start_button.place(x=x_position, y=50)
    up_button.place(x=x_position, y=90)
    down_button.place(x=x_position, y=130)
    right_button.place(x=x_position, y=170)
    left_button.place(x=x_position, y=210)
    start_record_button.place(x=x_position, y=250)
    stop_record_button.place(x=x_position, y=290)
    toggle_bright_particle_button.place(x=x_position, y=330)

    '''
    # Cannot mix pack,place and grid
    exit_button.pack(side=RIGHT)
    start_button.pack(side=RIGHT)
    up_button.pack(side=RIGHT)
    down_button.pack(side=RIGHT)
    right_button.pack(side=RIGHT)
    left_button.pack(side=RIGHT)
    start_record_button.pack(side=RIGHT)
    stop_record_button.pack(side=RIGHT)
    toggle_bright_particle_button.pack(side=RIGHT)


    start_button.grid(column=0,row=4)
    exit_button.grid(column=0,row=1)

    start_record_button.grid(column=0,row=2)
    stop_record_button.grid(column=0,row=3)

    up_button.grid(column=1,row=0)
    down_button.grid(column=1,row=1)
    right_button.grid(column=1,row=2)
    left_button.grid(column=1,row=3)

    toggle_bright_particle_button.grid(column=2,row=0)
    '''
class SLMThread(threading.Thread,threadID,name):
    import SLM
    def __init__(self):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.setDaemon(True, threadID, name)
    def run():
        Delta,N,M = SLM.get_delta()
        SLM_image = SLM.GSW(N,M,Delta)
        SLM.setup_fullscreen_plt_image()
        plt.imshow(image,cmap='gist_gray')
        plt.show()
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

         # open video source (by default this will try to open the computer webcam)
         #self.vid = MyVideoCapture()

         # Create a canvas that can fit the above video source size
         self.canvas = tkinter.Canvas(window, width = 1000, height = 1000)
         #self.canvas.grid(column=0,row=2)
         self.canvas.place(x=0, y=0)
         #self.canvas.pack(side=LEFT)

         # Button that lets the user take a snapshot
         #self.btn_snapshot=tkinter.Button(window, text="Snapshot", width=50, command=self.snapshot)
         #self.btn_snapshot.pack(anchor=tkinter.CENTER, expand=True)
         create_buttons(self.window)
         # TODO Add all the buttons
         self.window.geometry('1500x1000')
         # After it is called once, the update method will be automatically called every delay milliseconds
         self.delay = 50
         self.update()

         self.window.mainloop()

    def snapshot(self):
         global image
         cv2.imwrite("frame-" + time.strftime("%d-%m-%Y-%H-%M-%S") + ".jpg", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

    def update(self):
         # Get a frame from the video source
         global image
         #TODO? Might wanna do some rescaling of the image so we can zoom in
         self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(image))
         self.canvas.create_image(0, 0, image = self.photo, anchor = tkinter.NW) # need to use a compatible image type

         self.window.after(self.delay, self.update)
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

           if np.abs(control_parameters['target_particle_center'][self.axis]-control_parameters['traps_relative_pos'][self.axis])>=control_parameters['movement_threshold'] and not np.isnan(control_parameters['target_particle_center'][self.axis]):
                maxPixel = np.abs(control_parameters['AOI'][1]-control_parameters['AOI'][0])
                TM.MoveMotorToPixel(self.motor ,targetPixel=control_parameters['target_particle_center'][self.axis],currentPixel=control_parameters['traps_relative_pos'][self.axis],maxPixel=maxPixel)
           control_parameters['motor_locks'][self.axis].release()
           time.sleep(0.1) # To give other threads some time to work
       return
class CameraThread(threading.Thread):
   def __init__(self, threadID, name,batch_size=100):
      # TODO add camera here
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
       video_index = 0 # Number of current video
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
               fourcc = VideoWriter_fourcc(*'MP42')
               image_width = control_parameters['AOI'][1]-control_parameters['AOI'][0]
               video = VideoWriter(control_parameters['recording_path']+'/moive'+str(video_index)+'.avi', fourcc, float(20), (image_width, image_width),isColor=False)
               video_created = True
               video_index += 1
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
                   # saved_images[number_images_saved%self.batch_size ] = copy.copy(image) # Do not want a reference but a copy of its own
                   # number_images_saved+=1
                   # if number_images_saved%self.batch_size ==0 and number_images_saved>=1:
                   #     np.save(control_parameters['recording_path']+'/'+str(number_images_saved-self.batch_size )+'_'+str(number_images_saved),np.uint8(saved_images))

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
       i = 0
       zoom_counter = 0
       while control_parameters['continue_capture']: # Change to continue tracking?
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
                   min_index_trap,min_index_particle = find_closest_unoccupied()
                   control_parameters['target_trap_pos'] = control_parameters['traps_relative_pos'][min_index_trap]
                   control_parameters['target_particle_center'] = control_parameters['particle_centers'][min_index_particle]
               else:
                   print('No particles in frame!')

           #     #control_parameters['particle_centers'][0],control_parameters['particle_centers'][1],picture = fpt.find_single_particle_center(copy.copy(image),threshold = 200) # Do we need a copy of this or move it to another thread?s
           #     # TODO, change the zoom in function
           #     if is_trapped_single():
           #         '''
           #         We are not zoomed in but have found a particle.
           #         Time to zoom in on it.
           #         '''
           #         control_parameters['zoomed_in'] =  True
           #         half_image_width = 50
           #         control_parameters['movement_threshold'] = 10
           #         set_AOI(half_image_width=half_image_width)
           # elif control_parameters['zoomed_in']:
           #     '''
           #     We are in zoomed in mode ready to predict the position using a network
           #     '''
           #     # predict_particle_position(network2,print_position=False) # Network cannot yet be trusted. Was fit to differen objectve magnification
           #     #control_parameters['particle_centers'][0],control_parameters['particle_centers'][1],picture = fpt.find_single_particle_center(copy.copy(image),threshold = 200)
           #
           #     x,y = fpt.find_particle_centers(copy.copy(image),
           #                                     threshold=control_parameters['particle_threshold'],
           #                                     particle_size_threshold=control_parameters['particle_size_threshold'],
           #                                     bright_particle=control_parameters['bright_particle'])
           #     control_parameters['particle_centers'] = [x,y]
           #     print('Centers are',control_parameters['particle_centers'])
           #
           #     if is_trapped_single():
           #         zoom_counter = 0 # particle is trapped do not want to increase the counter
           #     else:
           #        '''
           #        We are in zoomed in mode but have lost the particle
           #        '''
           #        zoom_counter += 1 # Particle might not be trapped, lets wait and see
           #        if zoom_counter>10:
           #            #
           #            control_parameters['zoomed_in'] =  False
           #            half_image_width = 500
           #            control_parameters['movement_threshold'] = 20
           #            set_AOI(half_image_width=half_image_width)
           #            zoom_counter = 0
           time.sleep(0.15) # Needed to prevent this thread from running too fast
           i+=1
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
def is_trapped_single(threshold_distance=50):
    '''
    Function for detemining if a particle has been trapped or not
    '''
    global control_parameters
    distance = np.sqrt((control_parameters['target_particle_center'][0]-control_parameters['traps_relative_pos'][0])**2 + (control_parameters['target_particle_center'][1]-control_parameters['traps_relative_pos'][1])**2)

    if distance<threshold_distance:
        control_parameters['trapped_counter'] += 1 # TODO put uppper limit on trapped zoom_counter
        control_parameters['trapped_counter'] = np.minimum(10,control_parameters['trapped_counter'])
    else:
        control_parameters['trapped_counter'] -= 1
    if control_parameters['trapped_counter']<0:
        control_parameters['trapped_counter']=0
    return control_parameters['trapped_counter']>=8
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
            distances[i][j] = np.sqrt((control_parameters['traps_relative_pos'][i][0]-control_parameters['particle_centers'][j][0])**2+...
                                (control_parameters['traps_relative_pos'][i][1]-control_parameters['particle_centers'][j][1])**2)
    return distances
def trap_occupied(distances,trap_index,threshold_distance=50):
    '''
    Checks if a specific trap is occupied by a particle. If so set that trap to occupied
    '''
    global control_parameters

    # Check that trap index is ok
    if trap_index>len(distances) or trap_index>len(control_parameters['traps_occupied']):
        print('Trap index out of range')
        return
    for dist_to_trap in distances[trap_index]:
        if dist_to_trap<=threshold_distance:
            control_parameters['traps_occupied'][trap_index] = True
            return
    control_parameters['traps_occupied'][trap_index] = False
    return
def check_all_traps(distances=None):
    '''
    Updates all traps to see if they are occupied
    '''
    if distances is None:
        distances = get_particle_trap_distances()

    for trap_index in range(len(distances)):
        trap_occupied(distances,trap_index)
def find_closest_unoccupied():
    '''
    Function for finding the paricle and (unoccupied) trap which are the closest

    Returns : min_index_trap,min_index_particle,min_distance

    '''

    distances = get_particle_trap_distances()
    check_all_traps(distances)


    min_distance = 20
    min_index_particle = 0 # Index of particle which is closes to an unoccupied trap
    min_index_trap = 0 # Index of unoccupied trap which is closest to a particle

    for trap_idx in range(len(control_parameters['traps_occupied'])):
        trapped = control_parameters['traps_occupied'][idx]
        if not trapped:
            particle_idx = np.argmin(distances[trap_idx])
            if distances[trap_idx][particle_idx]<min_distance:
                min_distance = distances[trap_idx][particle_idx]
                min_index_trap = trap_idx
                min_index_particle = particle_idx

    '''# Todo rewrite this function in a sexier way
    indices = [i for i in range(len(control_parameters['traps_occupied'])) if not control_parameters['traps_occupied']]
    tmp = 1e6*np.ones((np.shape(distances)))
    tmp[indices] = distances[indices]
    return ind = np.unravel_index(np.argmin(tmp, axis=None), np.shape(tmp))
    '''
    return min_index_trap,min_index_particle,min_distance
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
def toggle_bright_particle():
    '''
    Function for switching between bright and other particle
    '''
    control_parameters['bright_particle'] = not control_parameters['bright_particle']
def set_particle_threshold():
    return
############### Main script starts here ####################################

control_parameters = get_default_control_parameters(recording_path='test_recording')
cam = TC.get_camera()
cam.set_defaults(left=control_parameters['AOI'][0],right=control_parameters['AOI'][1],top=control_parameters['AOI'][2],bot=control_parameters['AOI'][3],n_frames=1)
exposure_time = TC.find_exposure_time(cam) # automagically finds a decent exposure time
print('Exposure time = ',exposure_time)
image = cam.grab_image()
camera_thread = CameraThread(1, 'Thread-camera')
thread_list = []
thread_list.append(camera_thread)
camera_thread.start()
print(np.shape(image))
T_D = TkinterDisplay(tkinter.Tk(), "Control display")

#camera_thread.join

'''
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

#control_parameters['particle_centers'][0],control_parameters['particle_centers'][1] = fpt.find_particle_centers(copy.copy(image),threshold = 150) # Do we need a copy of this or move it to another thread?s
control_parameters['particle_centers'][0],control_parameters['particle_centers'][1] = control_parameters['traps_relative_pos'][0],control_parameters['traps_relative_pos'][1] # Do not want it wandering on its own

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
'''
