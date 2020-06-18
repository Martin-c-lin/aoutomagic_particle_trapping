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

def get_default_c_p(recording_path=None):
    '''
    Dictionary containing primarily parameters used for specifying the
    experiment and synchronizing the program threads, such as current trap and
    motor serial numbers.
    '''
    # TODO : change this into a class
    if recording_path == None:
        now = datetime.datetime.now()
        recording_path = 'F:/Martin/D'+str(now.year)+'-'+str(now.month)+'-'+str(now.day)
        try:
            os.mkdir(recording_path)
        except:
            print('Directory already exist')
    c_p = {
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
    'setpoint_temperature':25,
    'current_temperature':25,
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


    'use_LGO': [True, True],  # Set to true for the traps which should
    # LGO beam instead of regular gaussian.
    'LGO_order': -8,
    'exposure_time':2,
    'SLM_iterations':30,
    'trap_separation':0,#20e-6,
    'd0x':-30e-6,
    'd0y':-30e-6,
    'new_video':False,
    'recording_duration':4000,
    'experiment_schedule':[20e-6, 25],
    'experiment_progress':0, # number of experiments run
    }

    # Set traps positions
    c_p['traps_absolute_pos'] = np.zeros((2, 1)) # This will need updating
    c_p['traps_relative_pos'] = np.zeros((2, 1))

    # Position of first trap
    c_p['traps_absolute_pos'][0][0] = 678
    c_p['traps_absolute_pos'][1][0] = 465
    c_p['traps_relative_pos'][0][0] = 678
    c_p['traps_relative_pos'][1][0] = 465
    c_p['traps_occupied'] = [False for i in range(len(c_p['traps_absolute_pos'][0]))]
    c_p['phasemask'] = np.zeros((1080,1080)) # phasemask of
    return c_p
def terminate_threads():
    '''
    Function for killing all the threads
    '''
    c_p['continue_capture'] = False # All threds exits their main loop once this parameter is changed
    c_p['motor_running'] = False
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
    global c_p

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
            z_thread = z_movement_thread(4, 'z-thread',serial_no=c_p['serial_no_piezo'],channel=c_p['channel'])
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
    global c_p
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
                c_p['particle_threshold'] = threshold
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
                c_p['setpoint_temperature'] = temperature
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
                c_p['SLM_iterations'] = iterations
                print("Number SLM iterations set to ",iterations)
                c_p['new_phasemask'] = True
            else:
                print('SLM iterations out of bounds')
        except:
            print('Cannot convert entry to integer')
        iterations_entry.delete(0,last=5000)
    def set_particle_separtion():
        entry = separation_entry.get()
        try:
            separation = float(entry)*1e-6
            if 0<=separation<100*1e-6:
                c_p['trap_separation'] = separation
                print("Trap separation set to ",separation)
                c_p['new_phasemask'] = True
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
        global c_p
        nbr_active_traps = 2#1
        max_nbr_traps = 2
        traps_positions = np.zeros((2, max_nbr_traps))
        phasemask_to_pixel = 10 # Ratio between length in pixels and length in phasemask generator

        xm, ym = SLM.get_xm_ym_rect(nbr_rows=1, nbr_columns=1,d0x=c_p['d0x'],d0y=c_p['d0y'])
        screen_x = [578,727]
        screen_y = [465,465]
        print(xm,ym)
        Delta,N,M = SLM.get_delta(xm=xm, ym=ym, use_LGO=c_p['use_LGO'],
            order=c_p['LGO_order'])
        c_p['phasemask'] = SLM.GSW(N,M,Delta,nbr_iterations=c_p['SLM_iterations']) # Regular GS surperior to GSW when having only 2 traps
        c_p['phasemask_updated'] = True

        c_p['traps_absolute_pos'] = np.zeros((2,nbr_active_traps))
        c_p['traps_relative_pos'] = np.zeros((2,nbr_active_traps))

        c_p['traps_absolute_pos'][0] = screen_x[:nbr_active_traps]
        c_p['traps_absolute_pos'][1] = screen_y[:nbr_active_traps]
        c_p['traps_relative_pos'][0] = [x - c_p['AOI'][0] for x in screen_x[:nbr_active_traps]]
        c_p['traps_relative_pos'][1] = [y - c_p['AOI'][2] for y in screen_y[:nbr_active_traps]]

        c_p['traps_occupied'] = [False for i in range(len(c_p['traps_absolute_pos'][0]))]

        while c_p['continue_capture']:
            if c_p['new_phasemask']:
                # Update number of traps in use
                # Calcualte new delta and phasemask
                xm, ym = SLM.get_xm_ym_rect(nbr_rows=1, nbr_columns=1,
                    d0x=c_p['d0x'], d0y=c_p['d0y'])
                Delta,N,M = SLM.get_delta(xm=xm, ym=ym, use_LGO=c_p['use_LGO'],
                    order=c_p['LGO_order'])
                c_p['phasemask'] = SLM.GSW(N,M,Delta,nbr_iterations=c_p['SLM_iterations']) # Note changed to GS
                c_p['phasemask_updated'] = True
                # Update the number of traps and their position
                c_p['traps_absolute_pos'] = np.zeros((2,nbr_active_traps))
                c_p['traps_relative_pos'] = np.zeros((2,nbr_active_traps))

                c_p['traps_absolute_pos'][0] = screen_x[:nbr_active_traps]
                c_p['traps_absolute_pos'][1] = screen_y[:nbr_active_traps]
                c_p['traps_relative_pos'][0] = [x - c_p['AOI'][0] for x in screen_x[:nbr_active_traps]]
                c_p['traps_relative_pos'][1] = [y - c_p['AOI'][2] for y in screen_y[:nbr_active_traps]]

                c_p['traps_occupied'] = [False for i in range(len(c_p['traps_absolute_pos'][0]))]

                # Acknowledge that a new phasemask was recived
                c_p['new_phasemask'] = False
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
            self.temp_hist_length = 30 # how long the temperature needs to be
            # within max_diff to be considered as stable
            self.max_diff = max_diff # Maximum value by which temperature is
            # allowed to deviate from target temperature for temperature to be
            # considered as stable.
            if temperature_controller is not None:
                self.temperature_controller = temperature_controller
                c_p['starting_temperature'] = self.temperature_controller.measure_temperature()
                c_p['current_temperature'] = c_p['starting_temperature']
            else:
                try:
                    self.temperature_controller = TemperatureControllerTED4015.TED4015()
                    c_p['starting_temperature'] = self.temperature_controller.measure_temperature()
                    c_p['current_temperature'] = c_p['starting_temperature']
                except:
                    # Handling the case of not being having a temperature controller
                    print('\nWARNING, COULD NOT ESTABLISH CONTACT WITH TEMEPERATURE CONTROLLER!\n')
                    self.temperature_controller = None
            self.setDaemon(True)
        def run(self):
            global c_p
            if self.temperature_controller is not None:
                # Turn on output and continuosly set and query the temperature.
                self.temperature_controller.turn_on_output()
                while c_p['continue_capture']:
                    self.temperature_controller.set_setpoint_temperature(c_p['setpoint_temperature'])
                    c_p['current_temperature'] = self.temperature_controller.measure_temperature()
                    self.temperature_history.append(c_p['current_temperature'])

                    if len(self.temperature_history)>=self.temp_hist_length:
                        self.temperature_history.pop(0)
                    history = [T-c_p['setpoint_temperature'] for T in self.temperature_history]
                    if max(np.abs(history))<self.max_diff:
                        c_p['temperature_stable'] = True
                    else:
                        c_p['temperature_stable'] = False
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
         global c_p
         cv2.imwrite(c_p['recording_path']+"/frame-" + time.strftime("%d-%m-%Y-%H-%M-%S") + ".jpg", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    def create_indicators(self):
            global c_p
            # Update if recording is turned on or not
            # TODO replace this with an information box
            if c_p['recording']:
             self.recording_label = Label(self.window,text='recording is on',bg='green')
            else:
             self.recording_label = Label(self.window,text='recording is off',bg='red')
            self.recording_label.place(x=1220,y=900)

            if c_p['tracking_on']:
             self.tracking_label = Label(self.window,text='Auto experimentis on',bg='green')
            else:
             self.tracking_label = Label(self.window,text='Auto experimentis is off',bg='red')
            self.tracking_label.place(x=1220,y=930)

            position_text = 'Current trap separation is: ' + str(c_p['trap_separation'])
            self.position_label = Label(self.window,text=position_text)
            self.position_label.place(x=1220,y=720)

            temperature_text = 'Current objective temperature is: '\
                +str(c_p['current_temperature'])+' C'+\
                '\n setpoint temperature is: '+str(c_p['setpoint_temperature'])+' C'
            if c_p['temperature_stable']:
                temperature_text += '\n Temperature is stable'
            else:
                temperature_text += '\n Temperature is not stable'
            self.temperature_label = Label(self.window,text=temperature_text)
            self.temperature_label.place(x=1220,y=840)
    def update_indicators(self):
        '''
        Helper function for updating on-screen indicators
        '''
        global c_p
        # Update if recording is turned on or not
        # TODO replace this with an information box
        if c_p['recording']:
            self.recording_label.config(text='recording is on',bg='green')
        else:
            self.recording_label.config(text='recording is off',bg='red')

        if c_p['tracking_on']:
            self.tracking_label.config(text='Auto experiment is on',bg='green')
        else:
            self.tracking_label.config(text='Auto experiment is off',bg='red')
        temperature_text = 'Current objective temperature is: '\
            +str(c_p['current_temperature'])+' C'+\
            '\n setpoint temperature is: '+str(c_p['setpoint_temperature'])+' C'
        if c_p['temperature_stable']:
            temperature_text += '\n Temperature is stable'
        else:
            temperature_text += '\n Temperature is not stable'

        self.temperature_label.config(text=temperature_text)

        position_text = 'Current trap separation is: ' + str(c_p['trap_separation'])+ \
        '\n Experiments run: '+str(c_p['experiment_progress'])+' out of: ' + str(len(c_p['experiment_schedule']))
        position_text += '\n LGO is ' + str(c_p['use_LGO']) + '\n order is ' + str(c_p['LGO_order'])
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
         if c_p['phasemask_updated']:
             self.SLM_Window.update()
             c_p['phasemask_updated'] = False
         self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(self.resize_display_image(image)))
         self.canvas.create_image(0, 0, image = self.photo, anchor = tkinter.NW) # need to use a compatible image type
         self.window.after(self.delay, self.update)

    def __del__(self):
        terminate_threads()
class SLM_window(Frame):
    global c_p
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
        self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(c_p['phasemask']))
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
      global c_p
      self.threadID = threadID
      self.name = name
      self.axis = axis # 0 = x-axis, 1 = y axis
      if self.axis==0:
          self.motor = TM.InitiateMotor(c_p['serial_num_X'],pollingRate=c_p['polling_rate'])
      elif self.axis==1:
          self.motor = TM.InitiateMotor(c_p['serial_num_Y'],pollingRate=c_p['polling_rate'])
      else:
          print('Invalid choice of axis, no motor available')
      c_p['motor_starting_pos'][self.axis] = float(str(self.motor.Position))
      print('Motor is at ',c_p['motor_starting_pos'][self.axis])
      self.setDaemon(True)
    def run(self):
       print('Running motor thread')
       global c_p
       while c_p['motor_running']:
            # Acquire lock to ensure that it is safe to move the motor
            c_p['motor_locks'][self.axis].acquire()

            if np.abs(c_p['motor_movements'][self.axis])>0:
                    # The movement limit must be positive
                    c_p['xy_movement_limit'] = np.abs(c_p['xy_movement_limit'])
                    # Check how much the motor is allowed to move

                    if np.abs(c_p['motor_movements'][self.axis])<=c_p['xy_movement_limit']:
                        TM.MoveMotorPixels(self.motor,c_p['motor_movements'][self.axis])
                    else:
                        if c_p['motor_movements'][self.axis]>0:
                            TM.MoveMotorPixels(self.motor,c_p['xy_movement_limit'])
                        else:
                            TM.MoveMotorPixels(self.motor,-c_p['xy_movement_limit'])
                    c_p['motor_movements'][self.axis] = 0
            c_p['motor_current_pos'][self.axis] = float(str(self.motor.Position))
            c_p['motor_locks'][self.axis].release()
            time.sleep(0.1) # To give other threads some time to work

       TM.DisconnectMotor(self.motor)
class z_movement_thread(threading.Thread):
    '''
    Thread for controling movement of the objective in z-directio.
    Will also help with automagically adjusting the focus to the sample.
    '''
    def __init__(self, threadID, name,serial_no,channel,polling_rate=250):
        threading.Thread.__init__(self)
        global c_p
        self.threadID = threadID
        self.name = name
        self.piezo = TM.PiezoMotor(serial_no,channel=channel,pollingRate=polling_rate)
        c_p['z_starting_position'] = self.piezo.get_position()
        self.setDaemon(True)
    def compensate_focus(self):
        '''
        Function for compensating the change in focus caused by x-y movement.
        '''
        global c_p
        new_z_pos = (c_p['z_starting_position']
            +c_p['z_x_diff']*(c_p['motor_starting_pos'][0] - c_p['motor_current_pos'][0])
            +c_p['z_y_diff']*(c_p['motor_starting_pos'][1] - c_p['motor_current_pos'][1]) )
        new_z_pos += c_p['temperature_z_diff']*(c_p['current_temperature']-c_p['starting_temperature'])
        return int(new_z_pos)
    def run(self):
        global c_p
        lifting_distance = 0

        while c_p['continue_capture']:

            # Check if the objective should be moved
            self.piezo.move_to_position(self.compensate_focus()+lifting_distance)

            if c_p['z_movement'] is not 0:
                #try:
                    c_p['z_movement'] = int(c_p['z_movement'])
                    # Move up if we are not already up
                    print("Trying to lift particles")
                    #if self.piezo.get_position()<c_p['z_starting_position']+300:
                    if self.piezo.move_relative(c_p['z_movement']):
                        lifting_distance += c_p['z_movement']

                    c_p['z_movement'] = 0
                # except:
                #     print('Cannot move objective to',c_p['z_movement'] )
                #     print('Resetting target z movement.')
            elif c_p['return_z_home']:
                #self.piezo.move_to_position(c_p['z_starting_position'])
                lifting_distance = 0
                c_p['return_z_home'] = False
                print('homing z')
            time.sleep(0.1)
            c_p['motor_current_pos'][2] = self.piezo.get_position()
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
        image_width = c_p['AOI'][1]-c_p['AOI'][0]
        image_height = c_p['AOI'][3]-c_p['AOI'][2]
        video_name = c_p['recording_path']+'/video-'+str(now.hour)+\
            '-'+str(now.minute)+'-'+str(now.second)+'T'+str(round(c_p['setpoint_temperature'],2))+\
            'C_'+str(c_p['trap_separation'])+'um_LGO_order_' +str(c_p['LGO_order'])+ '.avi'
            # Added LGO order to name for upcoming experiments
        video = VideoWriter(video_name, fourcc, float(c_p['framerate']), (image_width, image_height),isColor=False)
        return video,video_name
   def new_video(self):

       pass
   def run(self):

       print('Initiating threaded capture sequence')

       number_images_saved = 0 # counts
       video_created = False
       # TODO - set framreate so we have a proper framerate in the videos!
       # Also need to record the framerate etc
       global c_p
       global cam
       while c_p['continue_capture']:
           # Set defaults for camera, aknowledge that this has been done
           cam.set_defaults(left=c_p['AOI'][0],right=c_p['AOI'][1],
                    top=c_p['AOI'][2],bot=c_p['AOI'][3],
                    exposure_time=c_p['exposure_time'])
           c_p['new_AOI_camera'] = False

           # Grab one example image
           global image
           image = cam.grab_image(n_frames=1)# This gave lots of errors for unkown reason
           image_count = 0
           # Start livefeed from the camera

           cam.start_live_video(framerate=str(c_p['framerate'])+'hertz' ) # Maximum framerate, shoul probably cap it
           start = time.time()

           # Create an array to store the images which have been captured in
           if not video_created:
               video,video_name = self.create_video_writer()
               video_created = True
           # Start continously capturin images now that the camera parameters have been set
           while c_p['continue_capture'] and not c_p['new_AOI_camera']\
                 and not c_p['new_video']:
               cam.wait_for_frame(timeout=None)
               if c_p['recording']:
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
       global c_p
       run_no = 0
       while c_p['continue_capture']:

            if c_p['tracking_on']:
                separation, temperature, order = experiment_schedule[run_no]
                c_p['setpoint_temperature'] = temperature
                c_p['LGO_order'] = order
                if np.abs(separation-c_p['trap_separation'])>3e-6:
                    move_particles_slowly(separation)
                c_p['trap_separation'] = separation
                c_p['new_phasemask'] = True
                run_finished = False

                while not run_finished:
                    if not c_p['new_phasemask'] and c_p['temperature_stable'] and c_p['tracking_on']:
                        zoom_in() # automagically creates a new video I think
                        start_record()
                        print('Making an experiment')
                        counter = 0
                        while counter <c_p['recording_duration'] and c_p['tracking_on']:
                            time.sleep(1) # Sleep to give the
                            counter +=1
                        # TODO record temperature and ensure that one can cancel the process
                        stop_record()
                        zoom_out()
                        start_record()
                        time.sleep(5) # Want to see the fluctuations
                        stop_record()
                        run_finished = True
                        c_p['experiment_progress'] += 1
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
    global c_p

    # Do not want motors to be moving when changing AOI!
    c_p['motor_locks'][0].acquire()
    c_p['motor_locks'][1].acquire()
    # Do we need the camera lock here?
    # Update the area of interest
    #if c_p['zoomed_in']:
    # Zoom in on particle

    # If exact values have been provided for all the
    # TODO change so that this syntax is default
    if left is not None and right is not None and up is not None and down is not None:
        if 0<=left<=1279 and left<=right<=1280 and 0<=up<=1079 and up<=down<=1080:
            c_p['AOI'][0] = left
            c_p['AOI'][1] = right
            c_p['AOI'][2] = up
            c_p['AOI'][3] = down
        else:
            print("Trying to set invalid area")
    else:
        c_p['AOI'] = [c_p['traps_relative_pos'][0]-half_image_width, c_p['traps_relative_pos'][0]+half_image_width, c_p['traps_relative_pos'][1]-half_image_width, c_p['traps_relative_pos'][1]+half_image_width]# +1 due to deeptrack oddity
    # else:
    #     # Use defult center
    #     c_p['AOI'] = [0,2*half_image_width,0,2*half_image_width]
    print('Setting AOI to ',c_p['AOI'])

    # Inform the camera and display thread about the updated AOI
    c_p['new_AOI_camera'] = True
    c_p['new_AOI_display'] = True

    # Update trap relative position
    c_p['traps_relative_pos'][0] = [x - c_p['AOI'][0] for x in c_p['traps_absolute_pos'][0]] #c_p['traps_absolute_pos'][0]- c_p['AOI'][0]
    c_p['traps_relative_pos'][1] = [y - c_p['AOI'][2] for y in c_p['traps_absolute_pos'][1]]#c_p['traps_absolute_pos'][1]- c_p['AOI'][2]

    #c_p['target_particle_center'] = [c_p['traps_relative_pos'][0],c_p['traps_relative_pos'][1]] # Don't want the motors to move just yet

    time.sleep(0.5) # Give motor threads time to catch up
    c_p['xy_movement_limit'] = 40
    c_p['motor_locks'][0].release()
    c_p['motor_locks'][1].release()
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
    global c_p
    #  TODO - Read half_image_width from the image
    resized = cv2.resize(copy.copy(image), (network_image_width,network_image_width), interpolation = cv2.INTER_AREA)
    pred = network.predict(np.reshape(resized/255,[1,network_image_width,network_image_width,1]))

    c_p['target_particle_center'][0] = half_image_width+pred[0][1]*half_image_width
    c_p['target_particle_center'][1] = half_image_width+pred[0][0]*half_image_width

    if print_position:
        print('Predicted posiiton is ',c_p['particle_centers'])
def get_particle_trap_distances():
    '''
    Calcualtes the distance between all particles and traps and returns a distance matrix,
    ordered as distances(traps,particles),
        To clarify the distance between trap n and particle m is distances[n][m
        ]

    '''
    global c_p
    nbr_traps = len(c_p['traps_absolute_pos'][0])
    nbr_particles = len(c_p['particle_centers'][0])
    distances = np.ones((nbr_traps,nbr_particles))
    for i in range(nbr_traps):
        for j in range(nbr_particles):
            dx = (c_p['traps_relative_pos'][0][i]-c_p['particle_centers'][0][j])
            dy = (c_p['traps_relative_pos'][1][i]-c_p['particle_centers'][1][j])
            distances[i,j] = np.sqrt(dx*dx+dy*dy)
            #distances[i][j] = np.sqrt((c_p['traps_relative_pos'][0][i]-c_p['particle_centers'][0][j])**2+(c_p['traps_relative_pos'][1][i]-c_p['particle_centers'][1][j])**2)
    return distances
def trap_occupied(distances,trap_index):
    '''
    Checks if a specific trap is occupied by a particle. If so set that trap to occupied.
    Updates if the trap is occupied or not and returns the index of the particle in the trap
    '''
    global c_p

    # Check that trap index is ok
    if trap_index>len(c_p['traps_occupied']) or trap_index<0:
        print('Trap index out of range')
        return None
    for i in range(len(distances[trap_index,:])):
        dist_to_trap = distances[trap_index,i]
        if dist_to_trap<=c_p['movement_threshold']:
            c_p['traps_occupied'][trap_index] = True
            return i
    try:
        c_p['traps_occupied'][trap_index] = False
        return None
    except:
        print(" Indexing error for trap index",str(trap_index)," length is ",len(c_p['traps_occupied']))
        return None
def check_all_traps(distances=None):
    '''
    Updates all traps to see if they are occupied.
    Returns the indices of the particles which are trapped. Indices refers to their
    position in the c_p['particle_centers'] array.
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
    for trap_idx in range(len(c_p['traps_occupied'])):
        trapped = c_p['traps_occupied'][trap_idx]

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
    global c_p
    move_distance = 200
    if move_direction==0:
        # Move up (Particles in image move up on the screen)
        #c_p['jog_motor_in_direction'][3]=True
        c_p['motor_movements'][1] = move_distance
    elif move_direction==1:
        # Move down
        c_p['motor_movements'][1] = -move_distance
        #c_p['jog_motor_in_direction'][2]=True
    elif move_direction==2:
        # c_p['jog_motor_in_direction'][0]=True
        # Move right
        c_p['motor_movements'][0] = move_distance
    elif move_direction==3:
        # c_p['jog_motor_in_direction'][1]=True
        # Move left
        c_p['motor_movements'][0] = -move_distance
    else:
        print('Invalid move direction')
def start_record():
    '''
    Button function for starting of recording
    '''
    c_p['recording']= True
    print('Recording is on')
def stop_record():
    '''
    Button function for starting of recording
    '''
    c_p['recording']= False
    print('Recording is off')
def toggle_bright_particle():
    '''
    Function for switching between bright and other particle
    '''
    c_p['bright_particle'] = not c_p['bright_particle']
    print("c_p['bright_particle'] set to",c_p['bright_particle'])
def toggle_tracking():
    c_p['tracking_on'] = not c_p['tracking_on']
    print("Tracking is ",c_p['tracking_on'])
def focus_up():
    '''
    Used for focus button to shift focus slightly up
    '''
    c_p['z_starting_position'] += 5
def focus_down():
    '''
    Used for focus button to shift focus slightly up
    '''
    c_p['z_starting_position'] -= 5
def zoom_in(margin=50):
    # Helper function for zoom button.
    # automagically zoom in on our traps
    # TODO make this so that it does not change shape when running an experiment
    left = max(min(c_p['traps_absolute_pos'][0])-margin,0)
    left = int(left // 10 * 10)
    right = min(max(c_p['traps_absolute_pos'][0])+margin,1200)
    right = int(right // 10 * 10)
    up = max(min(c_p['traps_absolute_pos'][1])-margin,0)
    up = int(up // 10 * 10)
    down = min(max(c_p['traps_absolute_pos'][1])+margin,1000)
    down = int(down // 10 * 10)

    c_p['framerate'] = 250 # Todo fix this so that it is better
    set_AOI(left=560, right=740, up=620, down=800)
def zoom_out():
    # Zooms out the camera and sets default framerate
    c_p['framerate'] = 10
    set_AOI(left=0,right=1200,up=0,down=1000)
def search_for_particles():
    '''
    Function for searching after particles. Threats the sample as a grid and systmatically searches it
    '''
    x_max = 3
    delta_y = 0.05
    print('searching for particles in '+c_p['search_direction']+' direction.')
    # Make movement
    # Todo, double check the signs of these
    if c_p['search_direction']== 'right':
        c_p['motor_movements'][0] = 300
    elif c_p['search_direction']== 'left':
        c_p['motor_movements'][0] = -300
    elif c_p['search_direction']== 'up':
        c_p['motor_movements'][1] = 300
    elif c_p['search_direction']== 'down': # currently not used
        c_p['motor_movements'][1] = -300

    # Update c_p['search_direction']for the 4 possible corners in the gridsearch
    if c_p['search_direction']== 'right' and (c_p['motor_current_pos'][0] - c_p['motor_starting_pos'][0])>x_max:
        c_p['search_direction']=='up'
        # y_start = c_p['motor_current_pos'][1]
    if c_p['search_direction']== 'up' and (c_p['motor_current_pos'][1]-y_start)>delta_y:
        c_p['search_direction']= 'left'
    if c_p['search_direction']== 'left' and c_p['motor_current_pos'][0]<=c_p['motor_starting_pos'][0]:
        c_p['search_direction']= 'right'
def move_particles_slowly(last_d=30e-6):
    # Function for moving the particles between the center and the edges
    # without dropping then
    global c_p
    if last_d>40e-6 or last_d<0:
        print('Too large distance.')
        return
    if last_d>c_p['trap_separation']:
        while c_p['trap_separation']<last_d:
            if c_p['new_phasemask']==False:
                if last_d - c_p['trap_separation'] < 1e-6:
                    c_p['trap_separation'] = last_d
                else:
                    c_p['trap_separation'] += 1e-6
                c_p['new_phasemask'] = True
                print(c_p['trap_separation'])
                time.sleep(0.5)
            time.sleep(1)
    else:
        while c_p['trap_separation']>last_d:
            if c_p['new_phasemask']==False:
                if c_p['trap_separation'] - last_d < 1e-6:
                    c_p['trap_separation'] = last_d
                else:
                    c_p['trap_separation'] -= 1e-6
                c_p['new_phasemask'] = True
                print(c_p['trap_separation'])
                time.sleep(0.5)
            time.sleep(1)
    return

temperatures = [25]
#for i in range(8):
#    temperatures.append(28+(i+1)/10)
distances = [0]
LGO_orders = [-4, 4, -8, 8, -12, 12, 16, -16]
experiment_schedule = []
for temp in temperatures:
    for distance in distances:
        for LGO_order in LGO_orders:
            experiment_schedule.append([distance, temp, LGO_order])
print(experiment_schedule)
############### Main script starts here ####################################
c_p = get_default_c_p() # C_P short for control_parameters
c_p['experiment_schedule'] = experiment_schedule# Arranged a distance,temp

# Create camera and set defaults
cam = TC.get_camera()
cam.set_defaults(left=c_p['AOI'][0],right=c_p['AOI'][1],top=c_p['AOI'][2],bot=c_p['AOI'][3],n_frames=1)
c_p['exposure_time'] = TC.find_exposure_time(cam,targetIntensity=70) # automagically finds a decent exposure time

print('Exposure time = ',c_p['exposure_time'] )
# c_p['exposure_time'] = 2
# TC.set_exposure(cam,2)
# Capture an example image to work with
image = cam.grab_image()

#c_p['particle_centers'][0],c_p['particle_centers'][1] = fpt.find_particle_centers(copy.copy(image),threshold = 150) # Do we need a copy of this or move it to another thread?s
c_p['particle_centers'][0],c_p['particle_centers'][1] = c_p['traps_relative_pos'][0],c_p['traps_relative_pos'][1] # Do not want it wandering on its own

# Create a empty list to put the threads in
thread_list = []

T_D = TkinterDisplay(tkinter.Tk(), "Control display")

# Close the threads and the camera
c_p['continue_capture'] = False # All threds exits their main loop once this parameter is changed
c_p['motor_running'] = False

# Shut down camera and motors safely
terminate_threads()
cam.close()
print(thread_list)
sys.exit()
