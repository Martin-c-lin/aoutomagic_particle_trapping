# Script for controlling the whole setup automagically
import ThorlabsCam as TC
import SLM
import ThorlabsMotor as TM
import TemperatureControllerTED4015
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
from tkinter import *
import PIL.Image, PIL.ImageTk

# TODO Combine this with Automagic trapping

def get_default_control_parameters(recording_path=None):
    '''
    Dictionary containing primarily parameters used for specifying the experiment and synchronizing
    the program threads, such as current trap and motor serial numbers.
    '''
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
    'recording': False, # Default
    'half_image_width':500, # TODO remove this parameter, should not be needed
    'AOI':[0,1200,0,1000],
    'new_AOI_camera': False,
    'new_AOI_display': False,
    'new_phasemask':False, # True if the phasemask is to be recalculated
    'phasemask_updated':False, # True if the phasemask image needs to be updated
    'movement_threshold': 30,
    'framerate':10,
    'recording':False,
    'tracking_on':False,
    'setpoint_temperature':30,
    'current_temperature':30,
    'starting_temperature':25,
    'temperature_stable':False,
    'search_direction':'right',
    'particle_centers': [[500],[500]], #[400,400],
    'target_particle_center':[500,500], # Position of the particle we currently are trying to trap
    # Used to minimize changes in code when updating to multiparticle tracking
    'target_trap_pos':[500,500],# Position of the trap we currently are trying to trap in
    'motor_movements':[0,0], # How much x and y motor should be moved
    'motor_starting_pos':[0,0], # Startng position of x-y motors, needed for z-compensation
    'motor_current_pos':[0,0,0], # Current position of x-y motors, needed for z-compensation, z is the last
    'z_starting_position':0, # Where the experiments starts in z position
    'z_movement':0, # Target z-movement in "ticks" positive for up, negative for down
    'z_x_diff':0,#-200, # Used for compensating drift in z when moving the sample. Caused by sample being slightly tilted Needs to be calibrated
    'z_y_diff':0,#-400,
    'temperature_z_diff':-190,#-200, # How much the objective need to be moved to compensate for the changes in temperature.Measured in [ticks/deg C]
    'return_z_home':False,
    'particle_threshold':120,
    'particle_size_threshold':200, # Parcticle detection threshold
    'bright_particle':True, # Is particle brighter than the background?
    'xy_movement_limit':1200,
    'motor_locks': [threading.Lock(),threading.Lock()],

    # Parameters specific to this experiment



    'exposure_time':2,
    'SLM_iterations':30,
    'trap_separation':20e-6,
    'new_video':False,
    'recording_duration':3000,
    'experiment_schedule':[20e-6,25],
    'experiment_progress':0, # number of experiments run
    }

    # Set traps positions
    control_parameters['traps_absolute_pos'] = np.zeros((2,1)) # This will need updating
    control_parameters['traps_relative_pos'] = np.zeros((2,1))

    # Position of first trap
    control_parameters['traps_absolute_pos'][0][0] = 678
    control_parameters['traps_absolute_pos'][1][0] = 465
    control_parameters['traps_relative_pos'][0][0] = 678
    control_parameters['traps_relative_pos'][1][0] = 465
    '''
    for i in range(16):
        control_parameters['traps_absolute_pos'][0][i] = 459+154*(i%4)#x
        control_parameters['traps_absolute_pos'][1][i] = 311+154*(i//4)#y
        control_parameters['traps_relative_pos'][0][i] = 459+154*(i%4)#x
        control_parameters['traps_relative_pos'][1][i] = 311+154*(i//4)#y
    '''
    control_parameters['traps_occupied'] = [False for i in range(len(control_parameters['traps_absolute_pos'][0]))]
    control_parameters['phasemask'] = np.zeros((1080,1080)) # phasemask of
    return control_parameters
def terminate_threads():
    '''
    Function for killing all the threads
    '''
    control_parameters['continue_capture'] = False # All threds exits their main loop once this parameter is changed
    control_parameters['motor_running'] = False
    #print('Terminating threads \n')
    time.sleep(1)
    global thread_list
    for thread in thread_list:
        thread.join()
    for thread in thread_list:
        del thread
def start_threads(cam=True,motor_x=True,motor_y=True,motor_z=True,slm=True,tracking=False,isaac=True,temp=True):
    """
    Function for starting all the threads, can only be called once
    """
    global thread_list
    global control_parameters

    if cam:
        camera_thread = CameraThread(1, 'Thread-camera')
        camera_thread.start()
        thread_list.append(camera_thread)
        print('Camera thread started')

    if motor_x:
        try:
            motor_X_thread = MotorThread(2,'Thread-motorX',0)
            motor_X_thread.start()
            thread_list.append(motor_X_thread)
            print('Motor x thread started')
        except:
            print('Could not start motor x thread')

    if motor_y:
        try:
            motor_Y_thread = MotorThread(3,'Thread-motorY',1)
            motor_Y_thread.start()
            thread_list.append(motor_Y_thread)
            print('Motor y thread started')
        except:
            print('Could not start motor y thread')

    if motor_z:
        try:
            z_thread = z_movement_thread(4, 'z-thread',serial_no=control_parameters['serial_no_piezo'],channel=control_parameters['channel'])
            z_thread.start()
            thread_list.append(z_thread)
            print('Motor z thread started')
        except:
            print('Could not start motor z thread')

    if slm:
        slm_thread =CreateSLMThread(5,'Thread-SLM')
        slm_thread.start()
        thread_list.append(slm_thread)
        print('SLM thread started')

    if tracking:
        tracking_thread = ExperimentControlThread(6,'Tracker_thread')
        tracking_thread.start()
        thread_list.append(tracking_thread)
        print('Tracking thread started')

    if temp:
        temperature_controller = TemperatureControllerTED4015.TED4015()
        temperature_thread = TemperatureThread(7,'Temperature_thread',temperature_controller=temperature_controller)
        temperature_thread.start()
        thread_list.append(temperature_thread)
        print('Temperature thread started')
def create_buttons(top):
    def get_y_separation(start=50,distance=40):
        index = 0
        while True:
            yield start + (distance * index)
            index += 1
    global control_parameters
    exit_button = tkinter.Button(top, text ='Exit program', command = terminate_threads)
    start_record_button = tkinter.Button(top, text ='Start recording', command = start_record)
    stop_record_button = tkinter.Button(top, text ='Stop recording', command = stop_record)
    toggle_bright_particle_button = tkinter.Button(top, text ='Toggle particle brightness', command = toggle_bright_particle)
    threshold_entry = tkinter.Entry(top, bd =5)
    temperature_entry = tkinter.Entry(top, bd =5)
    iterations_entry = tkinter.Entry(top,bd=5)
    separation_entry = tkinter.Entry(top,bd=5)
    toggle_tracking_button = tkinter.Button(top, text ='Toggle auto experiment', command = toggle_tracking)
    # TODO: Add a camera exposure buyttonfunction
    def set_threshold():
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
    def set_temperature():
        entry = temperature_entry.get()
        try:
            temperature = float(entry)
            if 20<temperature<40:
                control_parameters['setpoint_temperature'] = temperature
                print("Temperature set to ",temperature)
            else:
                print('Temperature out of bounds, it is no good to cook or freeze your samples')
        except:
            print('Cannot convert entry to float')
        temperature_entry.delete(0,last=5000)
    def set_iterations():
        entry = iterations_entry.get()
        try:
            iterations = int(entry)
            if 1<iterations<1000:
                control_parameters['SLM_iterations'] = iterations
                print("Number SLM iterations set to ",iterations)
                control_parameters['new_phasemask'] = True
            else:
                print('SLM iterations out of bounds')
        except:
            print('Cannot convert entry to integer')
        iterations_entry.delete(0,last=5000)
    def set_particle_separtion():
        entry = separation_entry.get()
        try:
            separation = float(entry)*1e-6
            if 0<=separation<40*1e-6:
                control_parameters['trap_separation'] = separation
                print("Trap separation set to ",separation)
                control_parameters['new_phasemask'] = True
            else:
                print('SLM trap positions out of bounds')
        except:
            print('Cannot convert entry to integer')
        separation_entry.delete(0,last=5000)

    threshold_button = tkinter.Button(top, text ='Set threshold', command = set_threshold)
    temperature_button = tkinter.Button(top, text ='Set setpoint temperature', command = set_temperature)
    SLM_Iterations_button = tkinter.Button(top, text ='Set SLM iterations', command = set_iterations)
    set_separation_button = tkinter.Button(top, text ='Set particle separation', command = set_particle_separtion)
    zoom_in_button = tkinter.Button(top, text ='Zoom in', command = zoom_in)
    zoom_out_button = tkinter.Button(top, text ='Zoom out', command = zoom_out)
    x_position = 1220
    y_position = get_y_separation()
    exit_button.place(x=x_position, y=y_position.__next__())
    start_record_button.place(x=x_position, y=y_position.__next__())
    stop_record_button.place(x=x_position,y=y_position.__next__())
    toggle_bright_particle_button.place(x=x_position, y=y_position.__next__())
    threshold_entry.place(x=x_position,y=y_position.__next__())
    threshold_button.place(x=x_position,y=y_position.__next__())
    toggle_tracking_button.place(x=x_position,y=y_position.__next__())
    temperature_entry.place(x=x_position,y=y_position.__next__())
    temperature_button.place(x=x_position,y=y_position.__next__())
    zoom_in_button.place(x=x_position,y=y_position.__next__())
    zoom_out_button.place(x=x_position,y=y_position.__next__())
    # New ones for this experiment
    iterations_entry.place(x=x_position,y=y_position.__next__())
    SLM_Iterations_button.place(x=x_position,y=y_position.__next__())

    separation_entry.place(x=x_position,y=y_position.__next__())
    set_separation_button.place(x=x_position,y=y_position.__next__())
class CreateSLMThread(threading.Thread):
    def __init__(self,threadID,name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.setDaemon(True)
    def run(self):
        global control_parameters
        nbr_active_traps = 2#1
        max_nbr_traps = 2
        traps_positions = np.zeros((2,max_nbr_traps))
        phasemask_to_pixel = 10 # Ratio between length in pixels and length in phasemask generator

        xm,ym = SLM.get_Isaac_xm_ym(d = control_parameters['trap_separation'])
        screen_x = [578,727]
        screen_y = [465,465]
        Delta,N,M = SLM.get_delta(xm=xm,ym=ym)
        control_parameters['phasemask'] = SLM.GS(N,M,Delta,nbr_iterations=control_parameters['SLM_iterations']) # Regular GS surperior to GSW when having only 2 traps
        control_parameters['phasemask_updated'] = True

        control_parameters['traps_absolute_pos'] = np.zeros((2,nbr_active_traps))
        control_parameters['traps_relative_pos'] = np.zeros((2,nbr_active_traps))

        control_parameters['traps_absolute_pos'][0] = screen_x[:nbr_active_traps]
        control_parameters['traps_absolute_pos'][1] = screen_y[:nbr_active_traps]
        control_parameters['traps_relative_pos'][0] = [x - control_parameters['AOI'][0] for x in screen_x[:nbr_active_traps]]
        control_parameters['traps_relative_pos'][1] = [y - control_parameters['AOI'][2] for y in screen_y[:nbr_active_traps]]

        control_parameters['traps_occupied'] = [False for i in range(len(control_parameters['traps_absolute_pos'][0]))]

        while control_parameters['continue_capture']:
            if control_parameters['new_phasemask']:
                # Update number of traps in use
                # Calcualte new delta and phasemask
                xm,ym = SLM.get_Isaac_xm_ym(d = control_parameters['trap_separation'])
                Delta,N,M = SLM.get_delta(xm=xm,ym=ym)
                control_parameters['phasemask'] = SLM.GS(N,M,Delta,nbr_iterations=control_parameters['SLM_iterations']) # Note changed to GS
                control_parameters['phasemask_updated'] = True
                # Update the number of traps and their position
                control_parameters['traps_absolute_pos'] = np.zeros((2,nbr_active_traps))
                control_parameters['traps_relative_pos'] = np.zeros((2,nbr_active_traps))

                control_parameters['traps_absolute_pos'][0] = screen_x[:nbr_active_traps]
                control_parameters['traps_absolute_pos'][1] = screen_y[:nbr_active_traps]
                control_parameters['traps_relative_pos'][0] = [x - control_parameters['AOI'][0] for x in screen_x[:nbr_active_traps]]
                control_parameters['traps_relative_pos'][1] = [y - control_parameters['AOI'][2] for y in screen_y[:nbr_active_traps]]

                control_parameters['traps_occupied'] = [False for i in range(len(control_parameters['traps_absolute_pos'][0]))]

                # Acknowledge that a new phasemask was recived
                control_parameters['new_phasemask'] = False
            time.sleep(1)
class TemperatureThread(threading.Thread):
        '''
        Class for running the temperature controller in the background
        '''
        def __init__(self, threadID, name,temperature_controller=None,max_diff=0.01):
            threading.Thread.__init__(self)
            self.threadID = threadID
            self.name = name
            self.temperature_history = []
            self.temp_hist_length = 30#60
            self.max_diff = max_diff # Maximum value by which temperature is allowed to deviate from target temperature
            # for temperature to be considered as stable.
            if temperature_controller is not None:
                self.temperature_controller = temperature_controller
                control_parameters['starting_temperature'] = self.temperature_controller.measure_temperature()
                control_parameters['current_temperature'] = control_parameters['starting_temperature']
            else:
                try:
                    self.temperature_controller = TemperatureControllerTED4015.TED4015()
                    control_parameters['starting_temperature'] = self.temperature_controller.measure_temperature()
                    control_parameters['current_temperature'] = control_parameters['starting_temperature']
                except:
                    # Handling the case of not being having a temperature controller
                    print('\nWARNING, COULD NOT ESTABLISH CONTACT WITH TEMEPERATURE CONTROLLER!\n')
                    self.temperature_controller = None
            self.setDaemon(True)
        def run(self):
            global control_parameters
            if self.temperature_controller is not None:
                # Turn on output and continuosly set and query the temperature.
                self.temperature_controller.turn_on_output()
                while control_parameters['continue_capture']:
                    self.temperature_controller.set_setpoint_temperature(control_parameters['setpoint_temperature'])
                    control_parameters['current_temperature'] = self.temperature_controller.measure_temperature()
                    self.temperature_history.append(control_parameters['current_temperature'])

                    if len(self.temperature_history)>=self.temp_hist_length:
                        self.temperature_history.pop(0)
                    history = [T-control_parameters['setpoint_temperature'] for T in self.temperature_history]
                    if max(np.abs(history))<self.max_diff:
                        control_parameters['temperature_stable'] = True
                    else:
                        control_parameters['temperature_stable'] = False
                        #print(self.temperature_history)

                    time.sleep(1) # We do not need to update the temperature very often
                self.temperature_controller.turn_off_output()
class TkinterDisplay:

    def __init__(self, window, window_title,):
         self.window = window
         self.window.title(window_title)

         # Create a canvas that can fit the above video source size
         self.canvas_width = 1200
         self.canvas_height = 1000
         self.canvas = tkinter.Canvas(window, width = self.canvas_width, height = self.canvas_height)
         self.canvas.place(x=0, y=0)

         # Button that lets the user take a snapshot
         self.btn_snapshot=tkinter.Button(window, text="Snapshot", command=self.snapshot)
         self.btn_snapshot.place(x=1300, y=0)
         create_buttons(self.window)
         self.window.geometry('1500x1000')
         # After it is called once, the update method will be automatically called every delay milliseconds
         self.delay = 50

         self.create_SLM_window(SLM_window)
         self.create_indicators()
         self.update()
         start_threads(cam=True,motor_x=False,motor_y=False,motor_z=False,slm=True,tracking=True,isaac=True,temp=True)
         self.window.mainloop()
    def create_SLM_window(self, _class):
        try:
            if self.new.state() == "normal":
                self.new.focus()
        except:
            self.new = tkinter.Toplevel(self.window)
            self.SLM_Window = _class(self.new)
    def snapshot(self):
         global image
         global control_parameters
         cv2.imwrite(control_parameters['recording_path']+"/frame-" + time.strftime("%d-%m-%Y-%H-%M-%S") + ".jpg", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    def create_indicators(self):
            global control_parameters
            # Update if recording is turned on or not
            # TODO replace this with an information box
            if control_parameters['recording']:
             self.recording_label = Label(self.window,text='recording is on',bg='green')
            else:
             self.recording_label = Label(self.window,text='recording is off',bg='red')
            self.recording_label.place(x=1220,y=900)

            if control_parameters['tracking_on']:
             self.tracking_label = Label(self.window,text='Auto experimentis on',bg='green')
            else:
             self.tracking_label = Label(self.window,text='Auto experimentis is off',bg='red')
            self.tracking_label.place(x=1220,y=930)

            position_text = 'Current trap separation is: ' + str(control_parameters['trap_separation'])
            self.position_label = Label(self.window,text=position_text)
            self.position_label.place(x=1220,y=800)

            temperature_text = 'Current objective temperature is: '\
                +str(control_parameters['current_temperature'])+' C'+\
                '\n setpoint temperature is: '+str(control_parameters['setpoint_temperature'])+' C'
            if control_parameters['temperature_stable']:
                temperature_text += '\n Temperature is stable'
            else:
                temperature_text += '\n Temperature is not stable'
            self.temperature_label = Label(self.window,text=temperature_text)
            self.temperature_label.place(x=1220,y=840)
    def update_indicators(self):
        '''
        Helper function for updating on-screen indicators
        '''
        global control_parameters
        # Update if recording is turned on or not
        # TODO replace this with an information box
        if control_parameters['recording']:
            self.recording_label.config(text='recording is on',bg='green')
        else:
            self.recording_label.config(text='recording is off',bg='red')

        if control_parameters['tracking_on']:
            self.tracking_label.config(text='Auto experiment is on',bg='green')
        else:
            self.tracking_label.config(text='Auto experiment is off',bg='red')
        temperature_text = 'Current objective temperature is: '\
            +str(control_parameters['current_temperature'])+' C'+\
            '\n setpoint temperature is: '+str(control_parameters['setpoint_temperature'])+' C'
        if control_parameters['temperature_stable']:
            temperature_text += '\n Temperature is stable'
        else:
            temperature_text += '\n Temperature is not stable'
        self.temperature_label.config(text=temperature_text)
        position_text = 'Current trap separation is: ' + str(control_parameters['trap_separation'])+ \
        '\n Experiments run: '+str(control_parameters['experiment_progress'])+' out of: ' + str(len(control_parameters['experiment_schedule']))
        self.position_label.config(text=position_text)
    def resize_display_image(self,img):
        img_size = np.shape(img)
        #print(img_size)
        if img_size[1]==self.canvas_width or img_size[0] == self.canvas_height:
            return img

        if img_size[1]/self.canvas_width > img_size[0]/self.canvas_height:
            dim = (int(self.canvas_width/img_size[1]*img_size[0]),int(self.canvas_width))
        else:
            dim = ( int(self.canvas_height),int(self.canvas_height/img_size[0]*img_size[1]))
        return cv2.resize(img, (dim[1],dim[0]), interpolation = cv2.INTER_AREA)
    def update(self):
         # Get a frame from the video source
         global image
         self.update_indicators()
         if control_parameters['phasemask_updated']:
             self.SLM_Window.update()
             control_parameters['phasemask_updated'] = False
         self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(self.resize_display_image(image)))
         self.canvas.create_image(0, 0, image = self.photo, anchor = tkinter.NW) # need to use a compatible image type
         self.window.after(self.delay, self.update)

    def __del__(self):
        terminate_threads()
class SLM_window(Frame):
    global control_parameters
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.master = master
        self.master.geometry("1920x1080+1920+0")#("1920x1080+2340+0")
        self.pack(fill=BOTH, expand=1)

        load = PIL.Image.open("SLM_16p_1080x1080.jpg")

        render = PIL.ImageTk.PhotoImage(load)
        self.img = Label(self, image=render)
        self.img.place(x=420, y=0)
        self.img.image = image
        ####
        self.delay = 500
        self.update()
    def update(self):
        # This implementation does work but is perhaps a tiny bit janky
        self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(control_parameters['phasemask']))
        del self.img.image
        self.img = Label(self,image=self.photo)
        self.img.image = self.photo # This ate lots of memory
        self.img.place(x=420, y=0) # Do not think this is needed
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
      control_parameters['motor_starting_pos'][self.axis] = float(str(self.motor.Position))
      print('Motor is at ',control_parameters['motor_starting_pos'][self.axis])
      self.setDaemon(True)
    def run(self):
       print('Running motor thread')
       global control_parameters
       while control_parameters['motor_running']:
            # Acquire lock to ensure that it is safe to move the motor
            control_parameters['motor_locks'][self.axis].acquire()

            if np.abs(control_parameters['motor_movements'][self.axis])>0:
                    # The movement limit must be positive
                    control_parameters['xy_movement_limit'] = np.abs(control_parameters['xy_movement_limit'])
                    # Check how much the motor is allowed to move

                    if np.abs(control_parameters['motor_movements'][self.axis])<=control_parameters['xy_movement_limit']:
                        TM.MoveMotorPixels(self.motor,control_parameters['motor_movements'][self.axis])
                    else:
                        if control_parameters['motor_movements'][self.axis]>0:
                            TM.MoveMotorPixels(self.motor,control_parameters['xy_movement_limit'])
                        else:
                            TM.MoveMotorPixels(self.motor,-control_parameters['xy_movement_limit'])
                    control_parameters['motor_movements'][self.axis] = 0
            control_parameters['motor_current_pos'][self.axis] = float(str(self.motor.Position))
            control_parameters['motor_locks'][self.axis].release()
            time.sleep(0.1) # To give other threads some time to work

       TM.DisconnectMotor(self.motor)
class z_movement_thread(threading.Thread):
    '''
    Thread for controling movement of the objective in z-directio.
    Will also help with automagically adjusting the focus to the sample.
    '''
    def __init__(self, threadID, name,serial_no,channel,polling_rate=250):
        threading.Thread.__init__(self)
        global control_parameters
        self.threadID = threadID
        self.name = name
        self.piezo = TM.PiezoMotor(serial_no,channel=channel,pollingRate=polling_rate)
        control_parameters['z_starting_position'] = self.piezo.get_position()
        self.setDaemon(True)
    def compensate_focus(self):
        '''
        Function for compensating the change in focus caused by x-y movement.
        '''
        global control_parameters
        new_z_pos = (control_parameters['z_starting_position']
            +control_parameters['z_x_diff']*(control_parameters['motor_starting_pos'][0] - control_parameters['motor_current_pos'][0])
            +control_parameters['z_y_diff']*(control_parameters['motor_starting_pos'][1] - control_parameters['motor_current_pos'][1]) )
        new_z_pos += control_parameters['temperature_z_diff']*(control_parameters['current_temperature']-control_parameters['starting_temperature'])
        return int(new_z_pos)
    def run(self):
        global control_parameters
        lifting_distance = 0

        while control_parameters['continue_capture']:

            # Check if the objective should be moved
            self.piezo.move_to_position(self.compensate_focus()+lifting_distance)

            if control_parameters['z_movement'] is not 0:
                #try:
                    control_parameters['z_movement'] = int(control_parameters['z_movement'])
                    # Move up if we are not already up
                    print("Trying to lift particles")
                    #if self.piezo.get_position()<control_parameters['z_starting_position']+300:
                    if self.piezo.move_relative(control_parameters['z_movement']):
                        lifting_distance += control_parameters['z_movement']

                    control_parameters['z_movement'] = 0
                # except:
                #     print('Cannot move objective to',control_parameters['z_movement'] )
                #     print('Resetting target z movement.')
            elif control_parameters['return_z_home']:
                #self.piezo.move_to_position(control_parameters['z_starting_position'])
                lifting_distance = 0
                control_parameters['return_z_home'] = False
                print('homing z')
            time.sleep(0.1)
            control_parameters['motor_current_pos'][2] = self.piezo.get_position()
class CameraThread(threading.Thread):
   def __init__(self, threadID, name):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.setDaemon(True)
   def create_video_writer(self):
        '''
        Funciton for creating a VideoWriter
        '''
        now = datetime.datetime.now()
        fourcc = VideoWriter_fourcc(*'MJPG')
        image_width = control_parameters['AOI'][1]-control_parameters['AOI'][0]
        image_height = control_parameters['AOI'][3]-control_parameters['AOI'][2]
        video_name = control_parameters['recording_path']+'/video-'+str(now.hour)+\
            '-'+str(now.minute)+'-'+str(now.second)+'T'+str(round(control_parameters['setpoint_temperature'],2))+\
            'C_'+str(control_parameters['trap_separation'])+'um.avi'
        video = VideoWriter(video_name, fourcc, float(control_parameters['framerate']), (image_width, image_height),isColor=False)
        return video,video_name
   def new_video(self):

       pass
   def run(self):

       print('Initiating threaded capture sequence')

       number_images_saved = 0 # counts
       video_created = False
       # TODO - set framreate so we have a proper framerate in the videos!
       # Also need to record the framerate etc
       global control_parameters
       global cam
       while control_parameters['continue_capture']:
           # Set defaults for camera, aknowledge that this has been done
           cam.set_defaults(left=control_parameters['AOI'][0],right=control_parameters['AOI'][1],
                    top=control_parameters['AOI'][2],bot=control_parameters['AOI'][3],
                    exposure_time=control_parameters['exposure_time'])
           control_parameters['new_AOI_camera'] = False

           # Grab one example image
           global image
           image = cam.grab_image(n_frames=1)# This gave lots of errors for unkown reason
           image_count = 0
           # Start livefeed from the camera

           cam.start_live_video(framerate=str(control_parameters['framerate'])+'hertz' ) # Maximum framerate, shoul probably cap it
           start = time.time()

           # Create an array to store the images which have been captured in
           if not video_created:
               video,video_name = self.create_video_writer()
               video_created = True
           # Start continously capturin images now that the camera parameters have been set
           while control_parameters['continue_capture'] and not control_parameters['new_AOI_camera']\
                 and not control_parameters['new_video']:
               cam.wait_for_frame(timeout=None)
               if control_parameters['recording']:
                   video.write(image)
               # Capture an image and update the image count
               image_count = image_count+1
               image[:][:][:] = cam.latest_frame()

           video.release()

           del video
           video_created = False
           # Close the livefeed and calculate the fps of the captures
           end = time.time()
           cam.stop_live_video()
           print('Capture sequence finished',image_count, 'Images captured in ',end-start,'seconds. \n FPS is ',image_count/(end-start))
class ExperimentControlThread(threading.Thread):
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
       run_no = 0
       while control_parameters['continue_capture']:

            if control_parameters['tracking_on']:
                separation,temperature = experiment_schedule[run_no]
                control_parameters['setpoint_temperature'] = temperature
                if np.abs(separation-control_parameters['trap_separation'])>3e-6:
                    move_particles_slowly(separation)
                control_parameters['trap_separation'] = separation
                control_parameters['new_phasemask'] = True
                run_finished = False

                while not run_finished:
                    if not control_parameters['new_phasemask'] and control_parameters['temperature_stable'] and control_parameters['tracking_on']:
                        zoom_in() # automagically creates a new video I think
                        start_record()
                        print('Making an experiment')
                        counter = 0
                        while counter <control_parameters['recording_duration'] and control_parameters['tracking_on']:
                            time.sleep(1) # Sleep to give the
                            counter +=1
                        # TODO record temperature and ensure that one can cancel the process
                        stop_record()
                        zoom_out()
                        start_record()
                        time.sleep(5) # Want to see the fluctuations
                        stop_record()
                        run_finished = True
                        control_parameters['experiment_progress'] += 1
                    else:
                        time.sleep(3)
                run_no += 1
                if run_no>=len(experiment_schedule):
                    break
                time.sleep(0.3)
            time.sleep(0.3) # Needed to prevent this thread from running too fast
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
    # Do we need the camera lock here?
    # Update the area of interest
    #if control_parameters['zoomed_in']:
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
    # else:
    #     # Use defult center
    #     control_parameters['AOI'] = [0,2*half_image_width,0,2*half_image_width]
    print('Setting AOI to ',control_parameters['AOI'])

    # Inform the camera and display thread about the updated AOI
    control_parameters['new_AOI_camera'] = True
    control_parameters['new_AOI_display'] = True

    # Update trap relative position
    control_parameters['traps_relative_pos'][0] = [x - control_parameters['AOI'][0] for x in control_parameters['traps_absolute_pos'][0]] #control_parameters['traps_absolute_pos'][0]- control_parameters['AOI'][0]
    control_parameters['traps_relative_pos'][1] = [y - control_parameters['AOI'][2] for y in control_parameters['traps_absolute_pos'][1]]#control_parameters['traps_absolute_pos'][1]- control_parameters['AOI'][2]

    #control_parameters['target_particle_center'] = [control_parameters['traps_relative_pos'][0],control_parameters['traps_relative_pos'][1]] # Don't want the motors to move just yet

    time.sleep(0.5) # Give motor threads time to catch up
    control_parameters['xy_movement_limit'] = 40
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

    control_parameters['target_particle_center'][0] = half_image_width+pred[0][1]*half_image_width
    control_parameters['target_particle_center'][1] = half_image_width+pred[0][0]*half_image_width

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
    Checks if a specific trap is occupied by a particle. If so set that trap to occupied.
    Updates if the trap is occupied or not and returns the index of the particle in the trap
    '''
    global control_parameters

    # Check that trap index is ok
    if trap_index>len(control_parameters['traps_occupied']) or trap_index<0:
        print('Trap index out of range')
        return None
    for i in range(len(distances[trap_index,:])):
        dist_to_trap = distances[trap_index,i]
        if dist_to_trap<=control_parameters['movement_threshold']:
            control_parameters['traps_occupied'][trap_index] = True
            return i
    try:
        control_parameters['traps_occupied'][trap_index] = False
        return None
    except:
        print(" Indexing error for trap index",str(trap_index)," length is ",len(control_parameters['traps_occupied']))
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
    move_distance = 200
    if move_direction==0:
        # Move up (Particles in image move up on the screen)
        #control_parameters['jog_motor_in_direction'][3]=True
        control_parameters['motor_movements'][1] = move_distance
    elif move_direction==1:
        # Move down
        control_parameters['motor_movements'][1] = -move_distance
        #control_parameters['jog_motor_in_direction'][2]=True
    elif move_direction==2:
        # control_parameters['jog_motor_in_direction'][0]=True
        # Move right
        control_parameters['motor_movements'][0] = move_distance
    elif move_direction==3:
        # control_parameters['jog_motor_in_direction'][1]=True
        # Move left
        control_parameters['motor_movements'][0] = -move_distance
    else:
        print('Invalid move direction')
def start_record():
    '''
    Button function for starting of recording
    '''
    control_parameters['recording']= True
    print('Recording is on')
def stop_record():
    '''
    Button function for starting of recording
    '''
    control_parameters['recording']= False
    print('Recording is off')
def toggle_bright_particle():
    '''
    Function for switching between bright and other particle
    '''
    control_parameters['bright_particle'] = not control_parameters['bright_particle']
    print("control_parameters['bright_particle'] set to",control_parameters['bright_particle'])
def toggle_tracking():
    control_parameters['tracking_on'] = not control_parameters['tracking_on']
    print("Tracking is ",control_parameters['tracking_on'])
def focus_up():
    '''
    Used for focus button to shift focus slightly up
    '''
    control_parameters['z_starting_position'] += 5
def focus_down():
    '''
    Used for focus button to shift focus slightly up
    '''
    control_parameters['z_starting_position'] -= 5
def zoom_in(margin=50):
    # Helper function for zoom button.
    # automagically zoom in on our traps
    # TODO make this so that it does not change shape when running an experiment
    left = max(min(control_parameters['traps_absolute_pos'][0])-margin,0)
    left = int(left // 10 * 10)
    right = min(max(control_parameters['traps_absolute_pos'][0])+margin,1200)
    right = int(right // 10 * 10)
    up = max(min(control_parameters['traps_absolute_pos'][1])-margin,0)
    up = int(up // 10 * 10)
    down = min(max(control_parameters['traps_absolute_pos'][1])+margin,1000)
    down = int(down // 10 * 10)

    control_parameters['framerate'] = 150 # Todo fix this so that it is better
    set_AOI(left=700,right=800,up=580,down=820)
def zoom_out():
    # Zooms out the camera and sets default framerate
    control_parameters['framerate'] = 10
    set_AOI(left=0,right=1200,up=0,down=1000)
def search_for_particles():
    '''
    Function for searching after particles. Threats the sample as a grid and systmatically searches it
    '''
    x_max = 3
    delta_y = 0.05
    print('searching for particles in '+control_parameters['search_direction']+' direction.')
    # Make movement
    # Todo, double check the signs of these
    if control_parameters['search_direction']== 'right':
        control_parameters['motor_movements'][0] = 300
    elif control_parameters['search_direction']== 'left':
        control_parameters['motor_movements'][0] = -300
    elif control_parameters['search_direction']== 'up':
        control_parameters['motor_movements'][1] = 300
    elif control_parameters['search_direction']== 'down': # currently not used
        control_parameters['motor_movements'][1] = -300

    # Update control_parameters['search_direction']for the 4 possible corners in the gridsearch
    if control_parameters['search_direction']== 'right' and (control_parameters['motor_current_pos'][0] - control_parameters['motor_starting_pos'][0])>x_max:
        control_parameters['search_direction']=='up'
        # y_start = control_parameters['motor_current_pos'][1]
    if control_parameters['search_direction']== 'up' and (control_parameters['motor_current_pos'][1]-y_start)>delta_y:
        control_parameters['search_direction']= 'left'
    if control_parameters['search_direction']== 'left' and control_parameters['motor_current_pos'][0]<=control_parameters['motor_starting_pos'][0]:
        control_parameters['search_direction']= 'right'
def move_particles_slowly(last_d = 30e-6):
    # Function for moving the particles out from the center to the edges
    global control_parameters
    if last_d>40e-6 or last_d<10e-6:
        print('Too large distance.')
        return
    if last_d>control_parameters['trap_separation']:
        while control_parameters['trap_separation']<last_d:
            if control_parameters['new_phasemask']==False:
                control_parameters['trap_separation'] += 1e-6
                control_parameters['new_phasemask'] = True
                print(control_parameters['trap_separation'])
                time.sleep(0.5)
            time.sleep(1)
    else:
        while control_parameters['trap_separation']>last_d:
            if control_parameters['new_phasemask']==False:
                control_parameters['trap_separation'] -= 1e-6
                control_parameters['new_phasemask'] = True
                print(control_parameters['trap_separation'])
                time.sleep(0.5)
            time.sleep(1)
    return

temperatures = [25]
#for i in range(8):
#    temperatures.append(28+(i+1)/10)
distances = np.linspace(10,35,26)*1e-6#[ 11e-6, 12e-6,15e-6, 30e-6 ]#[33e-6,30e-6,25e-6,24e-6,23e-6,22e-6,21e-6,20e-6,19e-6,18e-6,17e-6,16e-6,15e-6,14e-6,13e-6,12.5e-6,12e-6,11.5e-6,11e-6]
experiment_schedule = []
for temp in temperatures:
    for distance in distances:
        experiment_schedule.append([distance,temp])
print(experiment_schedule)
############### Main script starts here ####################################
control_parameters = get_default_control_parameters()
control_parameters['experiment_schedule'] = experiment_schedule# Arranged a distance,temp

# Create camera and set defaults
cam = TC.get_camera()
cam.set_defaults(left=control_parameters['AOI'][0],right=control_parameters['AOI'][1],top=control_parameters['AOI'][2],bot=control_parameters['AOI'][3],n_frames=1)
control_parameters['exposure_time'] = TC.find_exposure_time(cam) # automagically finds a decent exposure time
#print('Exposure time = ',control_parameters['exposure_time'] )
#control_parameters['exposure_time'] = 80
#TC.set_exposure(cam,80)
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
