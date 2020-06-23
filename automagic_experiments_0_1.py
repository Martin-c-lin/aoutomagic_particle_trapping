# Script for controlling the whole setup automagically
import ThorlabsCam as TC
import SLM
import ThorlabsMotor as TM
import TemperatureControllerTED4015
import find_particle_threshold as fpt
from instrumental import u
import matplotlib.pyplot as plt
import numpy as np
import threading, time, cv2, queue, copy, sys, tkinter, os
from tkinter import messagebox
from functools import partial
import datetime
from cv2 import VideoWriter, VideoWriter_fourcc
from tkinter import *
import PIL.Image, PIL.ImageTk

def get_default_c_p(recording_path=None):
    '''
    Dictionary containing primarily parameters used for specifying the
    experiment and synchronizing
    the program threads, such as current trap and motor serial numbers.
    # TODO : Consider to change this into a class
    '''
    if recording_path is None:
        now = datetime.datetime.now()
        recording_path = 'F:/Martin/D' + str(now.year) \
            + '-' + str(now.month) + '-' + str(now.day)
        try:
            os.mkdir(recording_path)
        except:
            print('Directory already exist')
    c_p = {
        'serial_num_X': '27502438',
        'serial_num_Y': '27502419',
        'serial_no_piezo': '97100532',
        'channel': 1,
        'network_path': 'C:/Martin/Networks/',
        'recording_path': recording_path,
        'polling_rate': 100,
        'continue_capture': True,  # True if camera etc should keep updating
        'motor_running': True,  # Should the motor thread keep running
        'zoomed_in': False,  # Keeps track of whether the image is cropped or
        # not
        'recording': False,  # True if recording is on
        'half_image_width': 500,  # TODO remove this parameter,
        'AOI': [0, 1200, 0, 1000],
        'new_AOI_camera': False,
        'new_AOI_display': False,
        'new_phasemask': False,
        'phasemask_updated': False,  # True if the phasemask is to be udpated
        'SLM_iterations': 30,
        'movement_threshold': 30,
        'nbr_experiments':1,
        'framerate': 10,
        'recording': False,
        'tracking_on': False,
        'setpoint_temperature': 25,
        'current_temperature': 25,
        'starting_temperature': 23.4,
        'temperature_controller_connected': False,
        'temperature_stable': False,
        'search_direction': 'right',
        'particle_centers': [[500], [500]],
        'target_particle_center': [500, 500],  # Position of the particle we
        # currently are trying to trap. Used to minimize changes in code when
        # updating to multiparticle tracking.
        'target_trap_pos': [500, 500],
        'motor_movements': [0, 0],  # How much x and y motor should be moved
        'motor_starting_pos': [0, 0],  # Startng position of x-y motors,
        # needed for z-compensation
        'motor_current_pos': [0, 0, 0],  # Current position of x-y motors,
        # needed for z-compensation, z is the last
        'z_starting_position': 0,  # Where the experiments starts in z position
        'z_movement': 0,  # Target z-movement in "ticks" positive for up,
        # negative for down
        'target_experiment_z': 150,  # height in ticks at which experiment should
        # be performed
        'z_x_diff': -200,  # Used for compensating drift in z when moving the
        # sample. Caused by sample being slightly tilted Needs to be calibrated
        # calculated as the change needed in z (measured in steps) when the
        # motor is moved 1 mm in positive direction z_x_diff = (z1-z0)/(x1-x0) steps/mm
        'z_y_diff': 0,
        'temperature_z_diff': 0, #-190,  # How much the objective need to be moved
        # in ticks when the objective is heated 1C. Needs to be calibrated manually.

        'slm_x_center': 700,#795, # needs to be recalibrated if camera is moved.
        # This is the position of the 0th order of the SLM (ie where the trap)
        # with xm=ym=0 is located in camera coordinates
        'slm_y_center': 890,#840,
        'slm_to_pixel': 4550000.0,
        # to compensate for the changes in temperature.Measured in
        # [ticks/deg C]
        'return_z_home': False,
        'particle_threshold': 120,
        'particle_size_threshold': 200,  # Parcticle detection threshold
        'bright_particle': True,  # Is particle brighter than the background?
        'xy_movement_limit': 1200,
        'motor_locks': [threading.Lock(), threading.Lock()],

        'use_LGO':[False],
        'LGO_order': -8,
        'exposure_time':2,
        'SLM_iterations':30,
        'trap_separation_x':20e-6,
        'trap_separation_y':20e-6,
        'new_video':False,
        'recording_duration':3000,
        'experiment_schedule':[20e-6, 25],
        'experiment_progress':0, # number of experiments run
        'experiment_runtime':0, # How many seconds have the experiment been running
        # TODO add progressbar for single experiments
    }

    # Set traps positions
    c_p['traps_absolute_pos'] = np.zeros((2,1))
    c_p['traps_relative_pos'] = np.zeros((2,1))

    # Position of first trap
    c_p['traps_absolute_pos'][0][0] = 678
    c_p['traps_absolute_pos'][1][0] = 465
    c_p['traps_relative_pos'][0][0] = 678
    c_p['traps_relative_pos'][1][0] = 465
    c_p['xm'], c_p['ym'] =  SLM.get_xm_ym_rect(
            nbr_rows=2, nbr_columns=2,
            d0x=-40e-6, d0y=-40e-6)
    #SLM_loc_to_trap_loc(c_p['xm'],c_p['ym'],c_p=c_p)

    # Cannot call SLM_loc_to_trap_loc until c_p has been created so we manually
    # converto from xm,ym to trap locs here
    tmp_x = [x * c_p['slm_to_pixel'] + c_p['slm_x_center'] for x in c_p['xm']]
    tmp_y = [y * c_p['slm_to_pixel'] + c_p['slm_y_center'] for y in c_p['ym']]
    tmp = np.asarray([tmp_x, tmp_y])
    c_p['traps_absolute_pos'] = tmp

    c_p['traps_occupied'] = [False for i in range(len(c_p['traps_absolute_pos'][0]))]
    c_p['phasemask'] = np.zeros((1080, 1080))  # phasemask  size
    return c_p


def terminate_threads():
    '''
    Function for terminating all threads.

    Returns
    -------
    None.

    '''
    c_p['continue_capture'] = False
    c_p['motor_running'] = False
    c_p['tracking_on'] = False
    time.sleep(1)
    global thread_list
    for thread in thread_list:
        thread.join()
    for thread in thread_list:
        del thread


def start_threads(cam=True, motor_x=True, motor_y=True, motor_z=True, slm=True,
        tracking=True, isaac=False, temp=True, experiment_schedule=None):
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
        if experiment_schedule is None:
            experiment_schedule = [
            {'setpoint_temperature':25,'xm':[-40e-6, -10e-6],'ym':[-40e-6, -10e-6]},
            {'setpoint_temperature':25,'xm':[-45e-6],'ym':[-45e-6]}
            ]


        tracking_thread = ExperimentControlThread(6,'Tracker_thread',experiment_schedule)
        tracking_thread.start()
        thread_list.append(tracking_thread)
        print('Tracking thread started')

    if temp:
        temperature_controller = TemperatureControllerTED4015.TED4015()
        temperature_thread = TemperatureThread(7,'Temperature_thread',temperature_controller=temperature_controller)
        temperature_thread.start()
        thread_list.append(temperature_thread)
        print('Temperature thread started')


class CreateSLMThread(threading.Thread):
    def __init__(self, threadID, name):
        '''
        Thread for controlling the SLM creation. When new_phasemask set to true
        the phasemask is updated.
        Parameters
        ----------
        threadID : int
            Thread number.
        name : string
            Name of thread.

        Returns
        -------
        None.

        '''
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.setDaemon(True)

    def run(self):
        '''
        Thread calculates the new phasemask when the parameter 'new phasemask'
        is set to true. It does this using the control parameters (xm, ym) for
        particle positions. use_LGO to determine if to use LGO or not.

        '''
        global c_p

        c_p['xm'], c_p['ym'] = SLM.get_default_xm_ym()
        Delta, N, M = SLM.get_delta(xm=c_p['xm'], ym=c_p['xm'],
            use_LGO=c_p['use_LGO'],
            order=c_p['LGO_order'])
        c_p['phasemask'] = SLM.GSW(
            N, M, Delta, nbr_iterations=c_p['SLM_iterations'])

        c_p['phasemask_updated'] = True
        SLM_loc_to_trap_loc(xm=c_p['xm'], ym=c_p['ym'])

        #print(c_p['traps_absolute_pos'])
        #print(c_p['traps_relative_pos'])

        c_p['traps_occupied'] =\
            [False for i in range(len(c_p['traps_absolute_pos'][0]))]

        while c_p['continue_capture']:
            if c_p['new_phasemask']:
                # Calcualte new delta and phasemask
                Delta, N, M = SLM.get_delta(xm=c_p['xm'], ym=c_p['ym'],
                    use_LGO=c_p['use_LGO'],
                    order=c_p['LGO_order'])
                c_p['phasemask'] = SLM.GSW(
                    N, M, Delta,
                    nbr_iterations=c_p['SLM_iterations'])
                c_p['phasemask_updated'] = True
                c_p['new_phasemask'] = False

                # Update the number of traps and their position
                SLM_loc_to_trap_loc(xm=c_p['xm'], ym=c_p['ym'])
                print(c_p['traps_absolute_pos'])
                c_p['traps_occupied'] =\
                    [False for i in range(len(c_p['traps_absolute_pos'][0]))]
            time.sleep(0.5)


class TemperatureThread(threading.Thread):
        '''
        Class for running the temperature controller in the background
        '''
        def __init__(self, threadID, name, temperature_controller=None, max_diff=0.01):
            '''


            Parameters
            ----------
            threadID : int
                Thread id number.
            name : String
                Name of thread.
            temperature_controller : temperaturecontroller, optional
                Controller of objective temperature. The default is None.
            max_diff : Float, optional
                 Maximum value by which temperature is allowed to deviate from
                 target temperature for temperature to be considered as stable.
                 The default is 0.01.

            Returns
            -------
            None.

            '''
            threading.Thread.__init__(self)
            self.threadID = threadID
            self.name = name
            self.temperature_history = []
            self.temp_hist_length = 100
            self.max_diff = max_diff
            if temperature_controller is not None:
                self.temperature_controller = temperature_controller
            else:
                try:
                    self.temperature_controller =\
                        TemperatureControllerTED4015.TED4015_controller()
                    c_p['starting_temperature'] =\
                        self.temperature_controller.measure_temperature()
                    c_p['current_temperature'] =\
                        c_p['starting_temperature']
                    c_p['temperature_controller_connected'] = True
                except:
                    # Handling the case of not having a temperature controller
                    print('\nWARNING, COULD NOT ESTABLISH CONTACT WITH \
                          TEMEPERATURE CONTROLLER!\n')
                    self.temperature_controller = None
            self.setDaemon(True)

        def run(self):
            global c_p
            if self.temperature_controller is not None:
                # Turn on output and continuosly set and query the temperature.
                self.temperature_controller.turn_on_output()
                while c_p['continue_capture']:
                    self.temperature_controller.set_setpoint_temperature(c_p['setpoint_temperature'])
                    c_p['current_temperature'] =\
                        self.temperature_controller.measure_temperature()
                    self.temperature_history.append(
                        c_p['current_temperature'])

                    if len(self.temperature_history)>self.temp_hist_length:
                        self.temperature_history.pop()
                    history = [T-c_p['setpoint_temperature'] for T in self.temperature_history]
                    if max(np.abs(history))<self.max_diff:

                        c_p['temperature_stable'] = True

                    else:
                        c_p['temperature_stable'] = False

                    time.sleep(1) # We do not need to update the temperature very often
                self.temperature_controller.turn_off_output()


class TkinterDisplay:

    def __init__(self, window, window_title,experiment_schedule=None):
        self.window = window
        self.window.title(window_title)

        # Create a canvas that can fit the above video source size
        self.canvas_width = 1200
        self.canvas_height = 1000
        self.canvas = tkinter.Canvas(
            window, width=self.canvas_width, height=self.canvas_height)
        self.canvas.place(x=0, y=0)

        # Button that lets the user take a snapshot
        self.btn_snapshot = tkinter.Button(
            window, text="Snapshot", command=self.snapshot)
        self.btn_snapshot.place(x=1300, y=0)
        self.create_buttons(self.window)
        self.window.geometry('1500x1000')
        # After it is called once, the update method will be automatically
        # called every delay milliseconds
        self.delay = 50

        self.create_SLM_window(SLM_window)
        self.create_indicators()
        self.update()
        start_threads(experiment_schedule=experiment_schedule)

        self.window.mainloop()

    def create_buttons(self,top=None):
        if top is None:
            top = self.window

        def get_y_separation(start=50, distance=40):
            # Simple generator to avoid printing all the y-positions of the
            # buttons
            index = 0
            while True:
                yield start + (distance * index)
                index += 1

        global c_p

        # TODO add home z button
        exit_button = tkinter.Button(top, text='Exit program',
                                     command=terminate_threads)
        up_button = tkinter.Button(top, text='Move up',
                                   command=partial(move_button, 0))
        down_button = tkinter.Button(top, text='Move down',
                                     command=partial(move_button, 1))
        right_button = tkinter.Button(top, text='Move right',
                                      command=partial(move_button, 2))
        left_button = tkinter.Button(top, text='Move left',
                                     command=partial(move_button, 3))
        start_record_button = tkinter.Button(top, text='Start recording',
                                             command=start_record)
        stop_record_button = tkinter.Button(top, text='Stop recording',
                                            command=stop_record)
        toggle_bright_particle_button = tkinter.Button(
            top, text='Toggle particle brightness',
            command=toggle_bright_particle)

        threshold_entry = tkinter.Entry(top, bd=5)
        temperature_entry = tkinter.Entry(top, bd=5)
        toggle_tracking_button = tkinter.Button(
            top, text='Toggle particle tracking', command=toggle_tracking)

        def set_threshold():
            entry = threshold_entry.get()
            try:
                threshold = int(entry)
                if 0 < threshold < 255:
                    c_p['particle_threshold'] = threshold
                    print("Threshold set to ", threshold)
                else:
                    print('Threshold out of bounds')
            except:
                print('Cannot convert entry to integer')
            threshold_entry.delete(0, last=5000)

        def set_temperature():
            entry = temperature_entry.get()
            try:
                temperature = float(entry)
                if 20 < temperature < 40:
                    c_p['setpoint_temperature'] = temperature
                    print("Temperature set to ", temperature)
                else:
                    print('Temperature out of bounds, it is no good to cook or \
                          freeze your samples')
            except:
                print('Cannot convert entry to integer')
            temperature_entry.delete(0, last=5000)

        threshold_button = tkinter.Button(
            top, text='Set threshold', command=set_threshold)
        focus_up_button = tkinter.Button(
            top, text='Move focus up', command=focus_up)
        focus_down_button = tkinter.Button(
            top, text='Move focus down', command=focus_down)
        temperature_button = tkinter.Button(
            top, text='Set setpoint temperature', command=set_temperature)
        zoom_in_button = tkinter.Button(top, text='Zoom in', command=zoom_in)
        zoom_out_button = tkinter.Button(top, text='Zoom out', command=zoom_out)

        x_position = 1220
        y_position = get_y_separation()
        exit_button.place(x=x_position, y=y_position.__next__())
        up_button.place(x=x_position, y=y_position.__next__())
        down_button.place(x=x_position, y=y_position.__next__())
        right_button.place(x=x_position, y=y_position.__next__())
        left_button.place(x=x_position, y=y_position.__next__())
        start_record_button.place(x=x_position, y=y_position.__next__())
        stop_record_button.place(x=x_position, y=y_position.__next__())
        toggle_bright_particle_button.place(x=x_position, y=y_position.__next__())
        threshold_entry.place(x=x_position, y=y_position.__next__())
        threshold_button.place(x=x_position, y=y_position.__next__())
        toggle_tracking_button.place(x=x_position, y=y_position.__next__())
        focus_up_button.place(x=x_position, y=y_position.__next__())
        focus_down_button.place(x=x_position, y=y_position.__next__())
        temperature_entry.place(x=x_position, y=y_position.__next__())
        temperature_button.place(x=x_position, y=y_position.__next__())
        zoom_in_button.place(x=x_position, y=y_position.__next__())
        zoom_out_button.place(x=x_position, y=y_position.__next__())

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
        cv2.imwrite(c_p['recording_path'] + "/frame-" +\
                    time.strftime("%d-%m-%Y-%H-%M-%S") +\
                    ".jpg", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

    def create_indicators(self):
        global c_p
        # Update if recording is turned on or not
        # TODO replace this with an information box
        if c_p['recording']:
            self.recording_label = Label(
                self.window, text='recording is on', bg='green')
        else:
            self.recording_label = Label(
                self.window, text='recording is off', bg='red')
        self.recording_label.place(x=1220, y=900)

        if c_p['tracking_on']:
             self.tracking_label = Label(
                 self.window, text='particle tracking is on', bg='green')
        else:
            self.tracking_label = Label(
                self.window, text='particle tracking is off', bg='red')
        self.tracking_label.place(x=1220, y=930)

        position_text = 'x: ' +\
            str(c_p['motor_current_pos'][0]) + ' y: ' \
                + str(c_p['motor_current_pos'][1]) + 'z: '\
                + str(c_p['motor_current_pos'][2])

        self.position_label = Label(self.window, text=position_text)
        self.position_label.place(x=1220, y=800)
        # TODO add trap positions
        temperature_text = 'Current objective temperature is: '+\
            str(c_p['current_temperature']) + ' C' +\
                '\n setpoint temperature is: ' +\
                str(c_p['setpoint_temperature']) + ' C'
        self.temperature_label = Label(self.window, text=temperature_text)
        self.temperature_label.place(x=1220, y=840)

    def update_indicators(self):
        '''
        Helper function for updating on-screen indicators
        '''
        global c_p
        # Update if recording is turned on or not
        # TODO replace this with an information box
        if c_p['recording']:
            self.recording_label.config(text='recording is on', bg='green')
        else:
            self.recording_label.config(text='recording is off', bg='red')

        if c_p['tracking_on']:
            self.tracking_label.config(text='particle tracking is on',bg='green')
        else:
            self.tracking_label.config(text='particle tracking is off', bg='red')
        temperature_text = 'Current objective temperature is: '+str(c_p['current_temperature'])+' C'+'\n setpoint temperature is: '+str(c_p['setpoint_temperature'])+' C'
        if c_p['temperature_stable']:
            temperature_text += '\nTemperature is stable. '
        else:
            temperature_text += '\nTemperature is not stable. '
        self.temperature_label.config(text=temperature_text)

        position_text = 'x: '+str(c_p['motor_current_pos'][0])+\
            ' y: '+str(c_p['motor_current_pos'][1])+\
            ' z: '+str(c_p['motor_current_pos'][2])
        position_text += '\n Experiments run ' + str(c_p['experiment_progress'])
        position_text += ' out of ' + str(c_p['nbr_experiments'])
        position_text += '  ' + str(c_p['experiment_runtime']) + 's run out of ' + str(c_p['recording_duration'])
        self.position_label.config(text=position_text)

    def resize_display_image(self, img):
        img_size = np.shape(img)
        if img_size[1]==self.canvas_width or img_size[0] == self.canvas_height:
            return img

        if img_size[1]/self.canvas_width > img_size[0]/self.canvas_height:
            dim = (int(self.canvas_width/img_size[1]*img_size[0]), int(self.canvas_width))
        else:
            dim = ( int(self.canvas_height), int(self.canvas_height/img_size[0]*img_size[1]))
        return cv2.resize(img, (dim[1],dim[0]), interpolation = cv2.INTER_AREA)

    def update(self):
         # Get a frame from the video source
         global image
         if c_p['phasemask_updated']:
              print('New phasemask')
              self.SLM_Window.update()
              c_p['phasemask_updated'] = False
         self.update_indicators()
         self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(self.resize_display_image(image)))
         self.canvas.create_image(0, 0, image = self.photo, anchor = tkinter.NW) # need to use a compatible image type
         self.window.after(self.delay, self.update)


class SLM_window(Frame):
    global c_p

    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.master = master
        self.master.geometry("1920x1080+1920+0")
        self.pack(fill=BOTH, expand=1)

        render = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(c_p['phasemask']))
        self.img = Label(self, image=render)
        self.img.place(x=420, y=0)
        self.img.image = image
        ####
        self.delay = 500
        self.update()

    def update(self):
        # This implementation does work but is perhaps a tiny bit janky
        self.photo = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(c_p['phasemask']))
        del self.img.image
        self.img = Label(self, image=self.photo)
        self.img.image = self.photo # This ate lots of memory
        self.img.place(x=420, y=0) # Do not think this is needed


class MotorThread(threading.Thread):
    '''
    Thread in which a motor is controlled. The motor object is available globally.
    '''
    def __init__(self, threadID, name, axis):

      threading.Thread.__init__(self)
      global c_p
      self.threadID = threadID
      self.name = name
      self.axis = axis # 0 = x-axis, 1 = y axis
      if self.axis==0:
          self.motor = TM.InitiateMotor(c_p['serial_num_X'],
            pollingRate=c_p['polling_rate'])
      elif self.axis==1:
          self.motor = TM.InitiateMotor(c_p['serial_num_Y'],
            pollingRate=c_p['polling_rate'])
      else:
          print('Invalid choice of axis, no motor available')
      c_p['motor_starting_pos'][self.axis] = float(str(self.motor.Position))
      print('Motor is at ', c_p['motor_starting_pos'][self.axis])
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
                        TM.MoveMotorPixels(self.motor, c_p['motor_movements'][self.axis])
                    else:
                        if c_p['motor_movements'][self.axis]>0:
                            TM.MoveMotorPixels(self.motor, c_p['xy_movement_limit'])
                        else:
                            TM.MoveMotorPixels(self.motor, -c_p['xy_movement_limit'])
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
    def __init__(self, threadID, name, serial_no, channel, polling_rate=250):
        threading.Thread.__init__(self)
        global c_p
        self.threadID = threadID
        self.name = name
        self.piezo = TM.PiezoMotor(serial_no, channel=channel, pollingRate=polling_rate)
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
        c_p['motor_current_pos'][2] = self.piezo.get_position()
        while c_p['continue_capture']:

            # Check if the objective should be moved
            self.piezo.move_to_position(self.compensate_focus()+lifting_distance)

            if c_p['z_movement'] is not 0:
                    c_p['z_movement'] = int(c_p['z_movement'])
                    # Move up if we are not already up
                    print("Trying to lift particles")
                    if self.piezo.move_relative(c_p['z_movement']):
                        lifting_distance += c_p['z_movement']

                    c_p['z_movement'] = 0

            elif c_p['return_z_home']:
                self.piezo.move_to_position(self.compensate_focus()) # Or should it be the starging point? which is best?
                lifting_distance = 0
                c_p['return_z_home'] = False
                print('homing z')
            time.sleep(0.3)
            c_p['motor_current_pos'][2] = self.piezo.get_position()
        del(self.piezo)

class CameraThread(threading.Thread):
   def __init__(self, threadID, name):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.setDaemon(True)
   def get_important_parameters(self):
       global c_p
       parameter_dict = {
       'xm':c_p['xm'],
       'ym':c_p['ym'],
       'use_LGO':c_p['use_LGO'],
       'LGO_order':c_p['LGO_order'],
       'setpoint_temperature':c_p['setpoint_temperature'],
       'target_experiment_z':c_p['target_experiment_z'],
       }
       return parameter_dict

   def create_video_writer(self):
        '''
        Funciton for creating a VideoWriter.
        TODO: Consider writing to a file to save the settings instead.
        # Write framerate to this in the end
        '''
        now = datetime.datetime.now()
        fourcc = VideoWriter_fourcc(*'MJPG')
        image_width = c_p['AOI'][1]-c_p['AOI'][0]
        image_height = c_p['AOI'][3]-c_p['AOI'][2]
        video_name = c_p['recording_path'] + '/moive-' + \
            str(now.hour) + '-' + str(now.minute) + '-' + str(now.second)+'.avi'
        experiment_info_name =c_p['recording_path'] + '/moive-' + \
            str(now.hour) + '-' + str(now.minute) + '-' + str(now.second) + 'info'

        video = VideoWriter(video_name, fourcc,
            float(c_p['framerate']),
            (image_width, image_height), isColor=False)
        exp_params = self.get_important_parameters()
        np.save(experiment_info_name, exp_params,  allow_pickle=True)
        return video, video_name

   def run(self):

       print('Initiating threaded capture sequence')

       number_images_saved = 0 # counts
       video_created = False
       # Also need to record the framerate etc
       global c_p
       global cam
       while c_p['continue_capture']:
           # Set defaults for camera, aknowledge that this has been done
           cam.set_defaults(left=c_p['AOI'][0],
               right=c_p['AOI'][1],
               top=c_p['AOI'][2],
               bot=c_p['AOI'][3])
           c_p['new_AOI_camera'] = False

           # Grab one example image
           global image
           image = cam.grab_image(n_frames=1)# This gave lots of errors for unkown reason
           image_count = 0
           # Start livefeed from the camera

           # Setting  maximum framerate. Will cap it to make it stable
           cam.start_live_video(
                framerate=str(c_p['framerate']) + 'hertz' )

           start = time.time()

           # Create an array to store the images which have been captured in
           if not video_created:
               video, video_name = self.create_video_writer()
               video_created = True
           # Start continously capturin images now that the camera parameters have been set
           while c_p['continue_capture']\
                and not c_p['new_AOI_camera']:
               cam.wait_for_frame(timeout=None)
               if c_p['recording']:
                   video.write(image) # TODO, ensure this is done in the background
               # Capture an image and update the image count
               image_count = image_count+1
               image[:][:][:] = cam.latest_frame()


           video.release()

           del video
           video_created = False
           # Close the livefeed and calculate the fps of the captures
           end = time.time()
           cam.stop_live_video()
           fps = image_count/(end-start)
           print('Capture sequence finished', image_count,
                'Images captured in ', end-start, 'seconds. \n FPS is ',
                fps)
           fps_file = video_name[:-4] + 'fps'
           np.save(fps_file, fps, allow_pickle=True)


class ExperimentControlThread(threading.Thread):
   '''
   Thread which does the tracking.
   '''
   def __init__(self, threadID, name, experiment_schedule):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.setDaemon(True)
        self.experiment_schedule = experiment_schedule

   def __del__(self):
       c_p['tracking_on'] = False

   def catch_particle(self, min_index_trap=None, min_index_particle=None):
        '''
        Function for determimning where and how to move when min_index_particle
        has been found
        '''
        global c_p
        if min_index_particle is not None:
          c_p['target_trap_pos'] = [c_p['traps_relative_pos'][0][min_index_trap],
                                    c_p['traps_relative_pos'][1][min_index_trap]]
          c_p['target_particle_center'] = [c_p['particle_centers'][0][min_index_particle],
                                            c_p['particle_centers'][1][min_index_particle]]
          c_p['motor_movements'][0] = -(c_p['target_trap_pos'][0] - c_p['target_particle_center'][0]) # Note: Sign of this depends on setup
          c_p['motor_movements'][1] = c_p['target_trap_pos'][1] - c_p['target_particle_center'][1]
          print('Moving to ', c_p['motor_movements'])
          print('Particle at ', c_p['target_particle_center'] )
          # If there is a trapped particle then we do not want to move very far so we accidentally lose it
          if True in c_p['traps_occupied']:
              c_p['xy_movement_limit'] = 40
          else:
              c_p['xy_movement_limit'] = 1200
        else:
            c_p['target_particle_center'] = []

   def lift_for_experiment(self,patiance=3):
       '''
       Assumes that all particles have been caught.
       patiance, how long(s) we allow a trap to be unoccipied for

       Returns true if lift succeded
       '''
       # TODO, add parameter to check if lifting was ok
       z_starting_pos = c_p['motor_current_pos'][2]
       patiance_counter = 0
       print('Lifting time. Starting from ', z_starting_pos)
       while c_p['target_experiment_z'] > c_p['motor_current_pos'][2] - z_starting_pos:
            time.sleep(0.5)
            all_filled, nbr_particles, min_index_trap, min_index_particle  =\
                self.check_exp_conditions()
            if all_filled:
                c_p['z_movement'] = 40
                c_p['return_z_home'] = False
                patiance_counter = 0
                # print('Particles at ', c_p['particle_centers'], ' traps at ' ,
                #    c_p['traps_relative_pos'])
            else:
                patiance_counter += 1
            if patiance_counter >= patiance or not c_p['tracking_on']:
                c_p['return_z_home'] = True
                c_p['z_movement'] = 0
                return False
       print('Lifting done. Now at',  c_p['motor_current_pos'][2])
       return True


   def check_exp_conditions(self, tracking_func=None):
        '''
        Checks if all traps are occupied. Returns true if this is the case.
        Tries to catch the closes unoccupied particle.
        '''
        if tracking_func is None:
            x, y = fpt.find_particle_centers(copy.copy(image),
                      threshold=c_p['particle_threshold'],
                      particle_size_threshold=c_p['particle_size_threshold'],
                      bright_particle=c_p['bright_particle'])
        else:
            x, y = tracking_func(copy.copy(image))

        c_p['particle_centers'] = [x, y]
        c_p['traps_occupied'] = [False for i in range(len(c_p['traps_absolute_pos'][0]))]
        min_index_trap, min_index_particle = find_closest_unoccupied()
        #self.catch_particle(min_index_trap, min_index_particle)

        if False not in c_p['traps_occupied']:
            # All traps have been occupied
            return True, -1, min_index_trap, min_index_particle
        # Not all traps have been occupied, might need to go searching
        return False, len(x), min_index_trap, min_index_particle

   def run_experiment(self, duration):
        '''
        Run an experiment for 'duration'.
        Returns 0 if it ran to the end without interruption otherwise it
        returns the amount of time remaining of the experiment.
        '''
        start = time.time()

        c_p['recording'] = True
        zoom_in()
        while time.time() <= start + duration and c_p['tracking_on']:
            all_filled, nbr_particles, min_index_trap, min_index_particle  =\
                self.check_exp_conditions()
            if all_filled:
                time.sleep(1)
                #print('Experiment is running', self.check_exp_conditions())
                c_p['experiment_runtime'] = np.round(time.time() - start)
            else:
                break
        zoom_out()
        c_p['recording'] = False
        if time.time() >= start + duration:
            return 0
        return start + duration - time.time()

   def run(self):
        '''
        Plan - have an experiment procedure.
        Before each experiment is allowed to start make sure all traps
        which are supposed to be filled are filled.
        Then lift the particles and start recording. If any particle
        is dropped go down and refill the traps and continue* the
        experiment.
        In between each experiment(when experiment parameters are to be changed)
        try to move the particles which are already trapped rather than
        cathing new ones (unless this is necessary). Then change all desired parameters.


        Control the experiments with a experiment Dictionary which
        keeps track of 'c_p' which are to be changed during the experiment.
        For instance might desire to change LGO orders as well as
        particle distance and temperature,then this should be possible

        * Do not record the full length but only the missing part of the video
        '''
        global image
        global c_p
        c_p['nbr_experiments'] = len(self.experiment_schedule)
        c_p['experiment_progress'] = 0

        while c_p['continue_capture']: # Change to continue tracking?
            time.sleep(0.3)
            #print('Schedule is ',self.experiment_schedule)
            # Wi
            # Look through the whole shedule, a list of dictionaries.

            if c_p['tracking_on']:
                    setup_dict = self.experiment_schedule[c_p['experiment_progress']]
                    print('Next experiment is', setup_dict)
                    run_finished = False
                    update_c_p(setup_dict)
                    time_remaining = c_p['recording_duration']
                    all_filled, nbr_particles, min_index_trap, min_index_particle = self.check_exp_conditions()

                    # Check if we need to go down and look for more particles in
                    # between the experiments
                    if not all_filled:
                        c_p['return_z_home'] = True
                        time.sleep(1)
                    while not run_finished and c_p['tracking_on']: # Are both these conditions needed, (while and if)?
                       time.sleep(0.3)
                        #if c_p['tracking_on'] and not run_finished:
                       '''
                       We are (probably) in full frame mode looking for a particle
                       '''

                       all_filled, nbr_particles, min_index_trap, min_index_particle = self.check_exp_conditions()
                       # print(all_filled, nbr_particles, min_index_trap, min_index_particle )
                       if not all_filled and nbr_particles <= c_p['traps_occupied'].count(True): # should check number of trues in this
                           # Fewer particles than traps
                           search_for_particles()
                       elif not all_filled and nbr_particles > c_p['traps_occupied'].count(True):
                           self.catch_particle(min_index_trap=min_index_trap,
                                min_index_particle=min_index_particle)
                       elif all_filled:
                           if self.lift_for_experiment():
                               # TODO change so that run_experiment is called
                               # with the recording duration of this specific
                               # experiment
                               print('lifted!')
                               time_remaining = self.run_experiment(time_remaining)
                           else:
                               c_p['return_z_home'] = True
                       if time_remaining < 1:
                           run_finished = True
                           c_p['experiment_progress'] += 1


def update_c_p(update_dict):
    '''
    Simple function for updating c_p['keys'] with new values 'values'.
    Ensures that all updates where successfull
    '''
    # TODO Add possibility to require temperature to be stable

    ok_parameters = ['use_LGO', 'LGO_order', 'xm', 'ym', 'setpoint_temperature',
    'recording_duration', 'target_experiment_z', 'SLM_iterations']

    requires_new_phasemask = ['use_LGO', 'LGO_order', 'xm', 'ym', 'SLM_iterations']

    for key in update_dict:
        if key in ok_parameters:
            try:
                c_p[key] = update_dict[key]
            except:
                print('Could not update control parameter ', key, 'with value',
                value)
                return
        else:
            print('Invalid key: ', key)
    # Check that both xm and ym are updated
    if len(c_p['xm']) > len(c_p['ym']):
        c_p['xm'] = c_p['xm'][:len(c_p['ym'])]
        print(' WARNING! xm and ym not the same length, cutting off xm!')

    if len(c_p['ym']) > len(c_p['xm']):
        c_p['ym'] = c_p['ym'][:len(c_p['xm'])]
        print(' WARNING! xm and ym not the same length, cutting off ym!')

    for key in update_dict:
        if key in requires_new_phasemask:
            c_p['new_phasemask'] = True


def set_AOI(half_image_width=50,left=None,right=None,up=None,down=None):
    '''
    Function for changing the Area Of Interest for the camera to the box specified by
    left,right,top,bottom
    Assumes global access to c_p
    '''
    global c_p

    # Do not want motors to be moving when changing AOI!
    c_p['motor_locks'][0].acquire()
    c_p['motor_locks'][1].acquire()
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
        c_p['AOI'] = [c_p['traps_relative_pos'][0]-half_image_width,
            c_p['traps_relative_pos'][0]+half_image_width,
            c_p['traps_relative_pos'][1]-half_image_width,
            c_p['traps_relative_pos'][1]+half_image_width]

    print('Setting AOI to ',c_p['AOI'])

    # Inform the camera and display thread about the updated AOI
    c_p['new_AOI_camera'] = True
    c_p['new_AOI_display'] = True

    # Update trap relative position
    update_traps_relative_pos()

    # Give motor threads time to catch up
    time.sleep(0.5)
    c_p['motor_locks'][0].release()
    c_p['motor_locks'][1].release()


def predict_particle_position(network,half_image_width=50,
    network_image_width=101,
    print_position=False):
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

    c_p['target_particle_center'][0] = half_image_width + pred[0][1] * half_image_width
    c_p['target_particle_center'][1] = half_image_width + pred[0][0] * half_image_width

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
    update_traps_relative_pos() # just in case
    nbr_traps = len(c_p['traps_relative_pos'][0])
    nbr_particles = len(c_p['particle_centers'][0])
    distances = np.ones((nbr_traps, nbr_particles))

    #print(c_p['particle_centers'][0])
    for i in range(nbr_traps):
        for j in range(nbr_particles):
            dx = (c_p['traps_relative_pos'][0][i] - c_p['particle_centers'][0][j])
            dy = (c_p['traps_relative_pos'][1][i] - c_p['particle_centers'][1][j])
            distances[i,j] = np.sqrt(dx * dx + dy * dy)
    return distances


def trap_occupied(distances, trap_index):
    '''
    Checks if a specific trap is occupied by a particle. If so set that trap to occupied.
    Updates if the trap is occupied or not and returns the index of the particle in the trap
    '''
    global c_p

    # Check that trap index is ok
    if trap_index > len(c_p['traps_occupied']) or trap_index < 0:
        print('Trap index out of range')
        return None
    for i in range(len(distances[trap_index, :])):
        dist_to_trap = distances[trap_index, i]
        if dist_to_trap <= c_p['movement_threshold']:
            c_p['traps_occupied'][trap_index] = True
            return i
    try:
        c_p['traps_occupied'][trap_index] = False
        return None
    except:
        print(" Indexing error for trap index", str(trap_index),\
        " length is ",len(c_p['traps_occupied']))
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
        if not trapped and len(distances[0]) > 0:
            # Had problems with distances being [] if there were no particles
            particle_idx = np.argmin(distances[trap_idx])

            # If particle is within the threshold then update min_index and trap index as well as min distance
            if distances[trap_idx,particle_idx]<min_distance:
                min_distance = distances[trap_idx,particle_idx]
                min_index_trap = trap_idx
                min_index_particle = particle_idx

    return min_index_trap, min_index_particle


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
        c_p['motor_movements'][1] = move_distance
    elif move_direction==1:
        # Move down
        c_p['motor_movements'][1] = -move_distance
    elif move_direction==2:
        # Move right
        c_p['motor_movements'][0] = move_distance
    elif move_direction==3:
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


def zoom_in(margin=60):
    # Helper function for zoom button.
    # automagically zoom in on our traps

    left = max(min(c_p['traps_absolute_pos'][0]) - margin, 0)
    left = int(left // 20 * 20)
    right = min(max(c_p['traps_absolute_pos'][0]) + margin, 1200)
    right = int(right // 20 * 20)
    up = max(min(c_p['traps_absolute_pos'][1]) - margin, 0)
    up = int(up // 20 * 20)
    down = min(max(c_p['traps_absolute_pos'][1]) + margin, 1000)
    down = int(down // 20 * 20)

    c_p['framerate'] = 300 # Todo fix this so that it is better
    set_AOI(left=left, right=right, up=up, down=down)


def zoom_out():
    set_AOI(left=0, right=1200, up=0, down=1000)
    c_p['framerate'] = 10


def search_for_particles():
    '''
    Function for searching after particles. Threats the sample as a grid and
    systmatically searches it
    '''
    x_max = 3 # [mm]
    delta_y = 0.05 # [mm]
    print('searching for particles in ' + c_p['search_direction'] + ' direction.')
    # Make movement
    # Todo, double check the signs of these
    if c_p['search_direction']== 'right':
        c_p['motor_movements'][0] = 300 # [px]

    elif c_p['search_direction']== 'left':
        c_p['motor_movements'][0] = -300

    elif c_p['search_direction']== 'up':
        c_p['motor_movements'][1] = 300

    elif c_p['search_direction']== 'down': # currently not used
        c_p['motor_movements'][1] = -300

    # Update c_p['search_direction']for the 4 possible corners in the gridsearch
    if c_p['search_direction']== 'right' \
        and (c_p['motor_current_pos'][0] - c_p['motor_starting_pos'][0])>x_max:
        c_p['search_direction']=='up'
        # y_start = c_p['motor_current_pos'][1]

    if c_p['search_direction']== 'up' and \
        (c_p['motor_current_pos'][1]-y_start)>delta_y:
        c_p['search_direction']= 'left'

    if c_p['search_direction']== 'left' and \
        c_p['motor_current_pos'][0]<=c_p['motor_starting_pos'][0]:

        c_p['search_direction']= 'right'


def update_traps_relative_pos():
    global c_p
    tmp_x = [x - c_p['AOI'][0] for x in c_p['traps_absolute_pos'][0] ]
    tmp_y = [y - c_p['AOI'][2] for y in c_p['traps_absolute_pos'][1] ]
    tmp = np.asarray([tmp_x, tmp_y])
    c_p['traps_relative_pos'] = tmp


def SLM_loc_to_trap_loc(xm, ym):
    '''
    Fucntion for updating the traps position based on their locaitons
    on the SLM.
    '''
    global c_p
    tmp_x = [x * c_p['slm_to_pixel'] + c_p['slm_x_center'] for x in xm]
    tmp_y = [y * c_p['slm_to_pixel'] + c_p['slm_y_center'] for y in ym]
    tmp = np.asarray([tmp_x, tmp_y])
    c_p['traps_absolute_pos'] = tmp
    print('Traps are at: ', c_p['traps_absolute_pos'] )
    update_traps_relative_pos()


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


############### Main script starts here ####################################
c_p = get_default_c_p()
# Create camera and set defaults
cam = TC.get_camera()
cam.set_defaults(left=c_p['AOI'][0], right=c_p['AOI'][1], top=c_p['AOI'][2], bot=c_p['AOI'][3], n_frames=1)
exposure_time = TC.find_exposure_time(cam) # automagically finds a decent exposure time
print('Exposure time = ', exposure_time)
image = cam.grab_image()

# Create a empty list to put the threads in
thread_list = []

# Define experiment to be run
xm1, ym1 = SLM.get_xm_ym_rect(nbr_rows=2,nbr_columns=3, dx=30e-6,dy=30e-6, d0x=-115e-6, d0y=-115e-6)
#get_xm_ym_triangle_with_center(d=30e-6, d0x=-40e-6, d0y=-40e-6)

experiment_schedule = [
{'setpoint_temperature':25,'xm':xm1, 'ym':ym1, 'use_LGO':[False],
'LGO_order':-8, 'target_experiment_z':1500, 'recording_duration':4_000},
{'setpoint_temperature':25,'xm':xm1, 'ym':ym1, 'use_LGO':[False,True,True,True],
'LGO_order':8, 'target_experiment_z':1500, 'recording_duration':4_000},
]

# {'xm':[-30e-6, -45e-6, -60e-6, -30e-6, -45e-6, -60e-6, -30e-6, -45e-6, -60e-6],
# 'ym':[-30e-6, -30e-6, -30e-6, -45e-6, -45e-6, -45e-6, -60e-6, -60e-6, -60e-6],
# 'use_LGO':[False]},
# {'xm':[-30e-6, -50e-6, -70e-6, -30e-6, -50e-6, -70e-6, -30e-6, -50e-6, -70e-6],
# 'ym':[-30e-6, -30e-6, -30e-6, -50e-6, -50e-6, -50e-6, -70e-6, -70e-6, -70e-6],
# 'use_LGO':[False]},
# {'xm':[-30e-6, -55e-6, -80e-6, -30e-6, -45e-6, -60e-6, -30e-6, -45e-6, -60e-6],
# 'ym':[-30e-6, -30e-6, -30e-6, -45e-6, -45e-6, -45e-6, -60e-6, -60e-6, -60e-6],
# 'use_LGO':[False]},

T_D = TkinterDisplay(tkinter.Tk(), "Control display",
    experiment_schedule=experiment_schedule)

# Close the threads and the camera
c_p['continue_capture'] = False # All threds exits their main loop once this parameter is changed
c_p['motor_running'] = False

# Shut down camera and motors safely
terminate_threads()
cam.close()
print(thread_list)
sys.exit()
