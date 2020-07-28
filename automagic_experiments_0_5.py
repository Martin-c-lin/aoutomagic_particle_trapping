# Script for controlling the whole setup automagically
import ThorlabsCam as TC
import SLM
import ThorlabsMotor as TM
import TemperatureControllerTED4015
import find_particle_threshold as fpt
import read_dict_from_file as rdff
from instrumental import u
import matplotlib.pyplot as plt
import numpy as np
import threading, time, cv2, queue, copy, sys, tkinter, os, pickle
from tkinter import messagebox
from tkinter import filedialog as fd
from functools import partial
from datetime import datetime
from cv2 import VideoWriter, VideoWriter_fourcc
from tkinter import *  # TODO Should avoid this type of import statements.
import PIL.Image, PIL.ImageTk
from pypylon import pylon

def get_recording_path(base_path='F:/Martin/D', extension_path=""):
    now = datetime.now()
    recording_path = base_path + str(now.year) \
        + '-' + str(now.month) + '-' + str(now.day)
    recording_path = recording_path + extension_path if len(extension_path) > 0 else recording_path
    print(recording_path)
    try:
        os.mkdir(recording_path)
    except:
        print('Directory already exist')
    return recording_path


def get_default_c_p(recording_path=None):
    '''
    Dictionary containing primarily parameters used for specifying the
    experiment and synchronizing
    the program threads, such as current trap and motor serial numbers.
    '''
    # TODO : Consider to change this into a class.
    # Make this object possible to pickle and unpickle to make it easier to
    # reuse settings.
    if recording_path is None:
        recording_path = get_recording_path()
    c_p = {
        'serial_nums_motors': ['27502438','27502419'],
        'serial_no_piezo': '97100532',
        'channel': 1,
        'network_path': 'G:/',
        'recording_path': recording_path,
        'polling_rate': 100,
        'program_running': True,  # True if camera etc should keep updating
        'motor_running': True,  # Should the motor thread keep running
        'zoomed_in': False,  # Keeps track of whether the image is cropped or
        # not
        'recording': False,  # True if recording is on
        'AOI': [0, 672, 0, 512], # Default for basler camera [0,1200,0,1000] TC
        'new_AOI_camera': False,
        'new_AOI_display': False,
        'new_phasemask': False,
        'phasemask_updated': False,  # True if the phasemask is to be udpated
        'SLM_iterations': 30,
        'movement_threshold': 30,
        'nbr_experiments':1,
        'framerate': 500,
        'recording': False,
        'tracking_on': False,
        'setpoint_temperature': 25,
        'current_temperature': 25,
        'starting_temperature': 23.4,
        'temperature_controller_connected': False,
        'temperature_stable': False,
        'temperature_output_on':True,
        'need_T_stable':False,
        'search_direction': 'up',
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
        'motors_connected':[False, False, False],
        'connect_motor':[True, True, True],
        'z_starting_position': 0,  # Where the experiments starts in z position
        'z_movement': 0,  # Target z-movement in "ticks" positive for up,
        # negative for down
        'target_experiment_z': 150,  # height in ticks at which experiment should
        # be performed
        'z_x_diff': 200,  # Used for compensating drift in z when moving the
        # sample. Caused by sample being slightly tilted Needs to be calibrated
        # calculated as the change needed in z (measured in steps) when the
        # motor is moved 1 mm in positive direction z_x_diff = (z1-z0)/(x1-x0) steps/mm
        # Sign ,+ or -,of this?
        'z_y_diff': -400, # approximate, has not measured this
        'x_start': 0,
        'temperature_z_diff': 0,#-180, #-80,  # How much the objective need to be moved
        # in ticks when the objective is heated 1C. Needs to be calibrated manually.
        # to compensate for the changes in temperature.Measured in
        # [ticks/deg C]

        'slm_x_center': 720,#700,#711, # needs to be recalibrated if camera is moved.
        # This is the position of the 0th order of the SLM (ie where the trap)
        # with xm=ym=0 is located in camera coordinates
        'slm_y_center': 605,#594 seem to be a tiny bit off, +5?
        'slm_to_pixel': 5000000.0, # Basler
        #4550000.0,# Thorlabs

        'return_z_home': False,
        'focus_threshold':1_000, #
        'particle_threshold': 100,
        'particle_size_threshold': 200,  # Parcticle detection threshold
        'bright_particle': True,  # Is particle brighter than the background?
        'xy_movement_limit': 1200,
        'motor_locks': [threading.Lock(), threading.Lock()],

        'use_LGO':[False],
        'LGO_order': -8,
        'exposure_time':80, # ExposureTime in micro s
        'SLM_iterations':5,
        'trap_separation_x':20e-6,
        'trap_separation_y':20e-6,
        'new_video':False,
        'recording_duration':3000,
        'experiment_schedule':[],
        'experiment_progress':0, # number of experiments run
        'experiment_runtime':0, # How many seconds have the experiment been running
        'activate_traps_one_by_one':True, # If true then the program will
        # activate and fill traps one by one.
        'camera_model':'basler',
        'cell_width':32,  # Width of cells when dividing the frame into a grid
        # for the path-search
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
            d0x=-50e-6, d0y=-50e-6)

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
    c_p['program_running'] = False
    c_p['motor_running'] = False
    c_p['tracking_on'] = False
    time.sleep(1)
    global thread_list
    for thread in thread_list:
        thread.join()
    for thread in thread_list:
        del thread


def start_threads(cam=True, motor_x=True, motor_y=True, motor_z=True, slm=True, tracking=True, isaac=False, temp=True):

    """
    Function for starting all the threads, should only be called once!
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

        try:
            temperature_controller = TemperatureControllerTED4015.TED4015()
        except:
            temperature_controller = None
            print('problem connecting to temperature controller')
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
        Delta, N, M = SLM.get_delta(xm=c_p['xm'], ym=c_p['ym'],
            use_LGO=c_p['use_LGO'],
            order=c_p['LGO_order'])

        c_p['phasemask'] = SLM.GSW(
            N, M, Delta, nbr_iterations=c_p['SLM_iterations'])

        c_p['phasemask_updated'] = True
        SLM_loc_to_trap_loc(xm=c_p['xm'], ym=c_p['ym'])


        c_p['traps_occupied'] =\
            [False for i in range(len(c_p['traps_absolute_pos'][0]))]

        while c_p['program_running']:
            if c_p['new_phasemask']:
                # Calcualte new delta and phasemask
                Delta, N, M = SLM.get_delta(xm=c_p['xm'], ym=c_p['ym'],
                    use_LGO=c_p['use_LGO'],
                    order=c_p['LGO_order'])
                if M==2:
                    print('Using normal Grechbgerg-Saxton since there are 2 traps')
                    c_p['phasemask'] = SLM.GS(
                        N, M, Delta,
                        nbr_iterations=c_p['SLM_iterations'])
                else:
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
        def __init__(self, threadID, name, temperature_controller=None, max_diff=0.05):
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
                c_p['starting_temperature'] =\
                    self.temperature_controller.measure_temperature()
                c_p['current_temperature'] =\
                    c_p['starting_temperature']
                c_p['setpoint_temperature'] = c_p['starting_temperature']
                c_p['temperature_controller_connected'] = True
            else:
                try:
                    self.temperature_controller =\
                        TemperatureControllerTED4015.TED4015_controller()
                    c_p['starting_temperature'] =\
                        self.temperature_controller.measure_temperature()
                    c_p['current_temperature'] =\
                        c_p['starting_temperature']
                    c_p['setpoint_temperature'] = c_p['starting_temperature']
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
                if c_p['temperature_output_on']:
                    self.temperature_controller.turn_on_output()
                while c_p['program_running']:
                    if 0 < c_p['setpoint_temperature'] < 40:
                        self.temperature_controller.set_setpoint_temperature(c_p['setpoint_temperature'])
                    else:
                        print('Setpoint temperature NOK')
                    c_p['current_temperature'] =\
                        self.temperature_controller.measure_temperature()
                    self.temperature_history.append(
                        c_p['current_temperature'])

                    # Update and check history
                    if len(self.temperature_history)>self.temp_hist_length:
                        self.temperature_history.pop()
                    history = [T-c_p['setpoint_temperature'] for T in self.temperature_history]
                    if max(np.abs(history))<self.max_diff:
                        c_p['temperature_stable'] = True

                    else:
                        c_p['temperature_stable'] = False

                    # Check output and if it shoould be on
                    if self.temperature_controller.query_output()==0 and \
                        c_p['temperature_output_on']:
                        self.temperature_controller.turn_on_output()
                    elif self.temperature_controller.query_output()==1 and not\
                        c_p['temperature_output_on']:
                        self.temperature_controller.turn_off_output()
                    time.sleep(1) # We do not need to update the temperature very often
                self.temperature_controller.turn_off_output()


class UserInterface:

    def __init__(self, window, window_title):
        self.window = window
        self.window.title(window_title)

        # Create a canvas that can fit the above video source size
        self.canvas_width = 1200
        self.canvas_height = 1000

        self.mini_canvas_width = 240
        self.mini_canvas_height = 200

        self.canvas = tkinter.Canvas(
            window, width=self.canvas_width, height=self.canvas_height)
        self.canvas.place(x=0, y=0)

        self.mini_canvas = tkinter.Canvas(
            window, width=self.mini_canvas_width, height=self.mini_canvas_height)
        self.mini_canvas.place(x=1200, y=800)
        self.mini_image = np.zeros((200,240,3))
        # Button that lets the user take a snapshot
        self.btn_snapshot = tkinter.Button(
            window, text="Snapshot", command=self.snapshot)
        self.btn_snapshot.place(x=1300, y=0)
        self.create_buttons(self.window)
        self.window.geometry('1700x1000')
        # After it is called once, the update method will be automatically
        # called every delay milliseconds
        self.delay = 100#50

        self.create_SLM_window(SLM_window)
        self.create_indicators()
        self.update()
        start_threads()

        self.window.mainloop()

    def __del__(self):
        # Close the program
        terminate_threads()
        c_p['program_running'] = False
        c_p['motor_running'] = False
        c_p['tracking_on'] = False

    def read_experiment_dictionary(self):
        global c_p
        filepath = fd.askopenfilename()
        # TODO make it so that we can handle exceptions from the file better here.
        # Also make it so it creates a new directory to save the data in named
        # after the schedule file.
        experiment_list = rdff.ReadFileToExperimentList(filepath)
        if len(experiment_list) > 0:
            c_p['experiment_schedule'] = experiment_list
            print('Starting the following experiment. \n', experiment_list)
            # Reset experiment progress
            if c_p['tracking_on']:
                c_p['tracking_on'] = False
                time.sleep(0.5)
                c_p['experiment_progress'] = 0
                time.sleep(0.2)
                c_p['tracking_on'] = True
            else:
                c_p['experiment_progress'] = 0
            c_p['nbr_experiments'] = len(c_p['experiment_schedule'])
            # Update recording path
            name = filepath[filepath.rfind('/')+1:filepath.rfind('.')]
            c_p['recording_path'] = get_recording_path(extension_path='_'+name)
        else:
            print('Invalid or empty file.')

    def create_trap_image(self):
        global c_p
        trap_x = c_p['traps_absolute_pos'][0]
        trap_y = c_p['traps_absolute_pos'][1]
        particle_x = c_p['particle_centers'][0]
        particle_y = c_p['particle_centers'][1]
        AOI = c_p['AOI']
        # Define new mini-image
        mini_image = np.zeros((200,240,3))
        scale_factor = 5

        l = int(round(AOI[2]/scale_factor))  # left
        r = int(round(AOI[3]/scale_factor))  # right
        u = int(round(AOI[0]/scale_factor))  # up
        d = int(round(AOI[1]/scale_factor))  # down

        # Draw the traps
        if len(trap_x) > 0 and len(trap_x) == len(trap_y):
            for x, y in zip(trap_x, trap_y):
                # Round down and recalculate
                x = int(round(x/scale_factor))
                y = int(round(y/scale_factor))

                if 1 <= x <= 239 and 1 <= y <= 199:
                    mini_image[(y-1):(y+2),(x-1):(x+2),0] = 255

        # Draw the particles
        if  len(particle_x) > 0 and len(particle_x) == len(particle_y):
            for x, y in zip(particle_x, particle_y):
                # Round down and recalculate
                x = int(round(x/scale_factor + u))
                y = int(round(y/scale_factor + l))
                if 1 <= x <= 239 and 1 <= y <= 199:
                    mini_image[y-1:y+2,x-1:x+2,2] = 255

        # Draw the AOI
        try:
            mini_image[l,u:d,1:2] = 255  # Left edge
            mini_image[l:r,u,1:2] = 255  # Upper edge
            mini_image[r,u:d,1:2] = 255  # Right edge
            mini_image[l:r,d,1:2] = 255  # Bottom edge
        except:
            mini_image[0,0:-1,1:2] = 255  # Left edge
            mini_image[0:-1,0,1:2] = 255  # Upper edge
            mini_image[-1,0:-1,1:2] = 255  # Right edge
            mini_image[0:-1,-1,1:2] = 255  # Bottom edge

        self.mini_image = mini_image.astype('uint8')

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
        # TODO: Check if we can change colors of buttons by making buttons part of
        # object.

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
        exposure_entry = tkinter.Entry(top, bd=5)

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

        def set_exposure():
            if c_p['camera_model'] == 'basler':
                entry = exposure_entry.get()
                try:
                    exposure_time = int(entry)
                    if 59 < exposure_time < 4e5: # If you need more than that you are
                        c_p['exposure_time'] = exposure_time
                        print("Exposure time set to ", exposure_time)
                        c_p['new_AOI_camera'] = True
                    else:
                        print('Exposure time out of bounds!')
                except:
                    print('Cannot convert entry to integer')
                exposure_entry.delete(0, last=5000)

        def connect_disconnect_motorX():
            # TODO: Behaviour of this might be odd if the motor did not get connected.
            c_p['connect_motor'][0] = not c_p['connect_motor'][0]

        def connect_disconnect_motorY():
            c_p['connect_motor'][1] = not c_p['connect_motor'][1]

        def connect_disconnect_piezo():
            c_p['connect_motor'][2] = not c_p['connect_motor'][2]

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
        temperature_output_button = tkinter.Button(top,
            text='toggle temperature output', command=toggle_temperature_output)
        set_exposure_button = tkinter.Button(top, text='Set exposure(basler)', command=set_exposure)
        experiment_schedule_button = tkinter.Button(top,
            text='Select experiment schedule',
            command=self.read_experiment_dictionary
            )
        # Motor buttons. Attributes of UserInterface class os we can easily change
        # the description text of them.
        self.toggle_motorX_button = tkinter.Button(
            top, text='Connect motor x', command=connect_disconnect_motorX)
        self.toggle_motorY_button = tkinter.Button(
            top, text='Connect motor y', command=connect_disconnect_motorY)
        self.toggle_piezo_button = tkinter.Button(
            top, text='Connect piezo motor', command=connect_disconnect_piezo)

        x_position = 1220
        x_position_2 = 1420
        y_position = get_y_separation()
        y_position_2 = get_y_separation()

        # Place all the buttons, starting with first column
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

        # Second column
        temperature_output_button.place(x=x_position_2, y=y_position_2.__next__())
        exposure_entry.place(x=x_position_2, y=y_position_2.__next__())
        set_exposure_button.place(x=x_position_2, y=y_position_2.__next__())
        experiment_schedule_button.place(x=x_position_2, y=y_position_2.__next__())
        self.toggle_motorX_button.place(x=x_position_2, y=y_position_2.__next__())
        self.toggle_motorY_button.place(x=x_position_2, y=y_position_2.__next__())
        self.toggle_piezo_button.place(x=x_position_2, y=y_position_2.__next__())

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

    def get_temperature_info(self):
        global c_p
        if c_p['temperature_controller_connected']:
            temperature_info = 'Current objective temperature is: '+str(c_p['current_temperature'])+' C'+'\n setpoint temperature is: '+str(c_p['setpoint_temperature'])+' C'
            if c_p['temperature_stable']:
                temperature_info += '\nTemperature is stable. '
            else:
                temperature_info += '\nTemperature is not stable. '
            if c_p['temperature_output_on']:
                temperature_info += '\n Temperature controller output is on.'
            else:
                temperature_info += '\n Temperature controller output is off.'
        else:
            temperature_info = 'Temperature controller is not connected.'


        return temperature_info

    def get_position_info(self):

        global c_p
        # Add position info
        position_text = 'x: '+str(c_p['motor_current_pos'][0])+\
            'mm   y: '+str(c_p['motor_current_pos'][1])+\
            'mm   z: '+str(c_p['motor_current_pos'][2])
        position_text += '\n Experiments run ' + str(c_p['experiment_progress'])
        position_text += ' out of ' + str(c_p['nbr_experiments'])
        position_text += '  ' + str(c_p['experiment_runtime']) + 's run out of ' + str(c_p['recording_duration'])
        position_text += '\n Current search direction is: ' + str(c_p['search_direction'] + '\n')

        # Add motor connection info
        x_connected = 'connected. ' if c_p['motors_connected'][0] else 'disconnected.'
        y_connected = 'connected. ' if c_p['motors_connected'][1] else 'disconnected.'
        piezo_connected = 'connected. ' if c_p['motors_connected'][2] else 'disconnected.'

        position_text += 'Motor-X is ' + x_connected
        position_text += ' Motor-Y is ' + y_connected + '\n'
        position_text += ' Focus (piezo) motor is ' + piezo_connected + '\n'

        return position_text

    def update_motor_buttons(self):
        # Motor connection buttons
        x_connect = 'Disconnect' if c_p['connect_motor'][0] else 'Connect'
        self.toggle_motorX_button.config(text=x_connect + ' motor x')
        y_connect = 'Disconnect' if c_p['connect_motor'][1] else 'Connect'
        self.toggle_motorY_button.config(text=y_connect + ' motor y')
        piezo_connected = 'Disconnect' if c_p['connect_motor'][2] else 'Connect'
        self.toggle_piezo_button.config(text=piezo_connected + ' piezo motor')

    def create_indicators(self):
        global c_p
        # Update if recording is turned on or not
        if c_p['recording']:
            self.recording_label = Label(
                self.window, text='recording is on', bg='green')
        else:
            self.recording_label = Label(
                self.window, text='recording is off', bg='red')
        self.recording_label.place(x=1220, y=750)

        if c_p['tracking_on']:
             self.tracking_label = Label(
                 self.window, text='particle tracking is on', bg='green')
        else:
            self.tracking_label = Label(
                self.window, text='particle tracking is off', bg='red')
        self.tracking_label.place(x=1220, y=780)

        self.position_label = Label(self.window, text=self.get_position_info())
        self.position_label.place(x=1420, y=400)
        self.temperature_label = Label(self.window, text=self.get_temperature_info())
        self.temperature_label.place(x=1420, y=540)

    def update_indicators(self):
        '''
        Helper function for updating on-screen indicators
        '''
        # TODO: Try an incorporate some of the labels into the buttons.
        global c_p
        # Update if recording is turned on or not
        if c_p['recording']:
            self.recording_label.config(text='recording is on', bg='green')
        else:
            self.recording_label.config(text='recording is off', bg='red')

        if c_p['tracking_on']:
            self.tracking_label.config(text='particle tracking is on',bg='green')
        else:
            self.tracking_label.config(text='particle tracking is off', bg='red')

        self.temperature_label.config(text=self.get_temperature_info())

        self.position_label.config(text=self.get_position_info())

        self.update_motor_buttons()

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

         # Update mini-window
         self.create_trap_image()
         self.mini_photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(self.mini_image, mode='RGB'))
         self.mini_canvas.create_image(0, 0, image = self.mini_photo, anchor = tkinter.NW) # need to use a compatible image type

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
    # TODO: Try removing the treadlocks on the motors.
    # Try replacing some of the c_p with events.
    def __init__(self, threadID, name, axis):

      threading.Thread.__init__(self)
      global c_p
      self.threadID = threadID
      self.name = name
      self.axis = axis # 0 = x-axis, 1 = y axis

      # Initiate contact with motor
      if self.axis == 0 or self.axis == 1:
          self.motor = TM.InitiateMotor(c_p['serial_nums_motors'][self.axis],
            pollingRate=c_p['polling_rate'])
      else:
          raise Exception("Invalid choice of axis, no motor available.")

      # Read motor starting position
      if self.motor is not None:
          c_p['motor_starting_pos'][self.axis] = float(str(self.motor.Position))
          print('Motor is at ', c_p['motor_starting_pos'][self.axis])
          c_p['motors_connected'][self.axis] = True
      else:
          c_p['motors_connected'][self.axis] = False
      self.setDaemon(True)

    def run(self):
        print('Running motor thread')
        global c_p
        while c_p['motor_running']:
            # If motor connected and it should be connected, check for next move
            if c_p['motors_connected'][self.axis] and \
                c_p['connect_motor'][self.axis] and c_p['motors_connected'][self.axis]:
                # Acquire lock to ensure that it is safe to move the motor
                with c_p['motor_locks'][self.axis]:
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
            # Motor is connected but should be disconnected
            elif c_p['motors_connected'][self.axis] and not c_p['connect_motor'][self.axis]:
                TM.DisconnectMotor(self.motor)
                c_p['motors_connected'][self.axis] = False
                self.motor = None
            # Motor is not connected but should be
            elif not c_p['motors_connected'][self.axis] and c_p['connect_motor'][self.axis]:
                self.motor = TM.InitiateMotor(c_p['serial_nums_motors'][self.axis],
                  pollingRate=c_p['polling_rate'])
                # Check if motor was successfully connected.
                if self.motor is not None:
                    c_p['motors_connected'][self.axis] = True
                else:
                    motor_ = 'x' if self.axis == 0 else 'y'
                    print('Failed to connect motor '+motor_)
            time.sleep(0.1) # To give other threads some time to work
        if c_p['motors_connected'][self.axis]:
            TM.DisconnectMotor(self.motor)


def compensate_focus():
    '''
    Function for compensating the change in focus caused by x-y movement.
    Returns the positon in ticks which z  should take to compensate for the focus
    '''
    global c_p
    new_z_pos = (c_p['z_starting_position']
        +c_p['z_x_diff']*(c_p['motor_starting_pos'][0] - c_p['motor_current_pos'][0])
        +c_p['z_y_diff']*(c_p['motor_starting_pos'][1] - c_p['motor_current_pos'][1]) )
    new_z_pos += c_p['temperature_z_diff']*(c_p['current_temperature']-c_p['starting_temperature'])
    return int(new_z_pos)


class z_movement_thread(threading.Thread):
    '''
    Thread for controling movement of the objective in z-direction.
    Will also help with automagically adjusting the focus to the sample.
    '''
    def __init__(self, threadID, name, serial_no, channel, polling_rate=250):
        threading.Thread.__init__(self)
        global c_p
        self.threadID = threadID
        self.name = name
        self.piezo = TM.PiezoMotor(serial_no, channel=channel, pollingRate=polling_rate)
        if self.piezo.is_connected:
            c_p['z_starting_position'] = self.piezo.get_position()
            c_p['motor_current_pos'][2] = self.piezo.get_position()
            c_p['motors_connected'][2] = self.piezo.is_connected
        self.setDaemon(True)

    def run(self):
        global c_p
        lifting_distance = 0
        while c_p['program_running']:
            c_p['motors_connected'][2] = self.piezo.is_connected

            # Check if piezo connected and should be connected
            if self.piezo.is_connected and c_p['connect_motor'][2]:

                # Check if the objective should be moved
                self.piezo.move_to_position(compensate_focus()+lifting_distance)

                if c_p['z_movement'] is not 0:
                        c_p['z_movement'] = int(c_p['z_movement'])
                        # Move up if we are not already up
                        if self.piezo.move_relative(c_p['z_movement']):
                            lifting_distance += c_p['z_movement']
                        c_p['z_movement'] = 0

                elif c_p['return_z_home'] and c_p['motor_current_pos'][2]>compensate_focus():
                    lifting_distance -= min(40,c_p['motor_current_pos'][2]-compensate_focus())
                    # Compensating for hysteresis effect in movement
                    print('homing z')
                if c_p['motor_current_pos'][2]<=compensate_focus() or c_p['z_movement'] != 0:
                    c_p['return_z_home'] = False
                if self.piezo.is_connected:
                    c_p['motor_current_pos'][2] = self.piezo.get_position()

            # Piezomotor not connected but should be
            elif not self.piezo.is_connected and c_p['connect_motor'][2]:
                self.piezo.connect_piezo_motor()

            # Piezo motor connected but should not be
            elif self.piezo.is_connected and not c_p['connect_motor'][2]:
                self.piezo.disconnect_piezo()

            time.sleep(0.3)
        del(self.piezo)


class CameraThread(threading.Thread):

   def __init__(self, threadID, name):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      # Initalize camera and global image
      if c_p['camera_model'] == 'ThorlabsCam':
          # Get a thorlabs camera
          self.cam = TC.get_camera()
          self.cam.set_defaults(left=c_p['AOI'][0], right=c_p['AOI'][1], top=c_p['AOI'][2], bot=c_p['AOI'][3], n_frames=1)
          exposure_time = TC.find_exposure_time(cam, targetIntensity=70) # automagically finds a decent exposure time
          print('Exposure time = ', exposure_time)
      else:
          # Get a basler camera
          tlf = pylon.TlFactory.GetInstance()
          self.cam = pylon.InstantCamera(tlf.CreateFirstDevice())
          self.cam.Open()
          image = np.zeros((672,512,1))
      self.setDaemon(True)

   def __del__(self):
        if c_p['camera_model'] == 'basler':
            self.cam.Close()
        else:
            self.cam.close()

   def get_important_parameters(self):
       global c_p
       parameter_dict = {
       'xm':c_p['xm'],
       'ym':c_p['ym'],
       'use_LGO':c_p['use_LGO'],
       'LGO_order':c_p['LGO_order'],
       'setpoint_temperature':c_p['setpoint_temperature'],
       'target_experiment_z':c_p['target_experiment_z'],
       'temperature_output_on':c_p['temperature_output_on'],
       'exposure_time':c_p['exposure_time'],
       'starting_temperature':c_p['current_temperature'],
       }
       return parameter_dict

   def create_video_writer(self):
        '''
        Funciton for creating a VideoWriter.
        Will also save the relevant parameters of the experiments.
        '''
        global c_p
        now = datetime.now()
        fourcc = VideoWriter_fourcc(*'MJPG')
        image_width = c_p['AOI'][1]-c_p['AOI'][0]
        image_height = c_p['AOI'][3]-c_p['AOI'][2]
        video_name = c_p['recording_path'] + '/video-' + \
            str(now.hour) + '-' + str(now.minute) + '-' + str(now.second)+'.avi'
        experiment_info_name =c_p['recording_path'] + '/video-' + \
            str(now.hour) + '-' + str(now.minute) + '-' + str(now.second) + '_info'

        video = VideoWriter(video_name, fourcc,
            float(c_p['framerate']),
            (image_width, image_height), isColor=False)
        exp_info_params = self.get_important_parameters()
        return video, experiment_info_name, exp_info_params

   def thorlabs_capture(self):
      number_images_saved = 0 # counts
      video_created = False
      global c_p

      while c_p['program_running']:
          # Set defaults for camera, aknowledge that this has been done
          self.cam.set_defaults(left=c_p['AOI'][0],
              right=c_p['AOI'][1],
              top=c_p['AOI'][2],
              bot=c_p['AOI'][3])
          c_p['new_AOI_camera'] = False
          # Grab one example image
          global image
          image = self.cam.grab_image(n_frames=1)
          image_count = 0
          # Start livefeed from the camera

          # Setting  maximum framerate. Will cap it to make it stable
          self.cam.start_live_video(
               framerate=str(c_p['framerate']) + 'hertz' )

          start = time.time()

          # Create an array to store the images which have been captured in
          if not video_created:
              video, experiment_info_name, exp_info_params = self.create_video_writer()
              video_created = True
          # Start continously capturin images now that the camera parameters have been set
          while c_p['program_running']\
               and not c_p['new_AOI_camera']:
              self.cam.wait_for_frame(timeout=None)
              if c_p['recording']:
                  video.write(image)
              # Capture an image and update the image count
              image_count = image_count+1
              image[:][:][:] = self.cam.latest_frame()


          video.release()

          del video
          video_created = False
          # Close the livefeed and calculate the fps of the captures
          end = time.time()
          self.cam.stop_live_video()
          fps = image_count/(end-start)
          print('Capture sequence finished', image_count,
               'Images captured in ', end-start, 'seconds. \n FPS is ',
               fps)
          # Save the experiment data in a pickled dict.
          outfile = open(experiment_info_name, 'wb')
          exp_info_params['fps'] = fps
          pickle.dump(exp_info_params, outfile)
          outfile.close()

   def set_basler_AOI(self):
       '''
       Function for setting AOI of basler camera to c_p['AOI']
       '''
       global c_p

       try:
            # The order in which you set the size and offset parameters matter.
            # If you ever get the offset + width greater than max width the
            # camera won't accept your valuse. Thereof the if-else-statements
            # below. Conditions might need to be changed if the usecase of this
            #  funciton change
            c_p['AOI'][1] -= np.mod(c_p['AOI'][1]-c_p['AOI'][0],16)
            c_p['AOI'][3] -= np.mod(c_p['AOI'][3]-c_p['AOI'][2],16)

            width = int(c_p['AOI'][1] - c_p['AOI'][0])
            offset_x = 672 - width - c_p['AOI'][0]
            height = int(c_p['AOI'][3] - c_p['AOI'][2])
            offset_y = 512 - height - c_p['AOI'][2]

            self.cam.OffsetX = 0
            self.cam.Width = width
            self.cam.OffsetX = offset_x
            self.cam.OffsetY = 0
            self.cam.Height = height
            self.cam.OffsetY = offset_y
       except Exception as e:
           print('AOI not accepted',c_p['AOI'])
           print(e)

   def basler_capture(self):
      number_images_saved = 0 # counts
      video_created = False
      global c_p
      img = pylon.PylonImage()

      while c_p['program_running']:
          # Set defaults for camera, aknowledge that this has been done

          self.set_basler_AOI()
          c_p['new_AOI_camera'] = False
          try:
              self.cam.ExposureTime = c_p['exposure_time']
          except:
              print('Exposure time not accepted by camera')
          # Grab one example image
          image_count = 0

          global image
          self.cam.StartGrabbing()

          start = time.time()

          # Create an array to store the images which have been captured in
          if not video_created:
              video, experiment_info_name, exp_info_params = self.create_video_writer()
              video_created = True
          # Start continously capturin images now that the camera parameters have been set
          while c_p['program_running']\
               and not c_p['new_AOI_camera']:

               with self.cam.RetrieveResult(2000) as result:
                  img.AttachGrabResultBuffer(result)
                  image = np.flip(img.GetArray(),axis=(0,1)) # Testing to flip this guy
                  img.Release()
                  if c_p['recording']:
                      video.write(image)
                  # Capture an image and update the image count
                  image_count = image_count+1

          video.release()
          self.cam.StopGrabbing()
          del video
          video_created = False
          # Close the livefeed and calculate the fps of the captures
          end = time.time()

          # Calculate FPS
          fps = image_count/(end-start)
          print('Capture sequence finished', image_count,
               'Images captured in ', end-start, 'seconds. \n FPS is ',
               fps)

          # Save the experiment data in a pickled dict.
          outfile = open(experiment_info_name, 'wb')
          exp_info_params['fps'] = fps
          pickle.dump(exp_info_params, outfile)
          outfile.close()

   def run(self):
       if c_p['camera_model'] == 'ThorlabsCam':
           self.thorlabs_capture()
       elif c_p['camera_model'] == 'basler':
           self.basler_capture()


class ExperimentControlThread(threading.Thread):
   '''
   Thread which does the tracking.
   '''
   def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.setDaemon(True)

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

          if True in c_p['traps_occupied']:
              c_p['xy_movement_limit'] = 40
              # Some traps are occupied. Want to avoid catching more than one
              # particle per trap.
              filled_traps_locs = []
              for idx, occupied in enumerate(c_p['traps_occupied']):
                  if occupied:
                      filled_traps_locs.append([c_p['traps_relative_pos'][0][idx],
                      c_p['traps_relative_pos'][1][idx] ])
              x, y, success = path_search(filled_traps_locs,
                              target_particle_location=c_p['target_particle_center'],
                              target_trap_location=c_p['target_trap_pos'])
          else:
              success = False
              c_p['xy_movement_limit'] = 1200
          if success:
              c_p['motor_movements'][0] = -x
              c_p['motor_movements'][1] = y
          else:
              c_p['motor_movements'][0] = -(c_p['target_trap_pos'][0] - c_p['target_particle_center'][0]) # Note: Sign of this depends on setup
              c_p['motor_movements'][1] = c_p['target_trap_pos'][1] - c_p['target_particle_center'][1]

        else:
            c_p['target_particle_center'] = []

   def lift_for_experiment(self,patiance=3):
       '''
       Assumes that all particles have been caught.
       patiance, how long(s) we allow a trap to be unoccipied for

       Returns true if lift succeded
       '''
       z_starting_pos = compensate_focus()
       patiance_counter = 0
       print('Lifting time. Starting from ', z_starting_pos)
       while c_p['target_experiment_z'] > c_p['motor_current_pos'][2] - z_starting_pos:
            time.sleep(0.2)
            all_filled, nbr_particles, min_index_trap, min_index_particle  =\
                self.check_exp_conditions()
            if all_filled:
                c_p['z_movement'] = 40
                c_p['return_z_home'] = False
                patiance_counter = 0
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
        patiance = 50
        patiance_counter = 0
        while time.time() <= start + duration and c_p['tracking_on']:
            all_filled, nbr_particles, min_index_trap, min_index_particle  =\
                self.check_exp_conditions()
            if all_filled:
                patiance_counter = 0
                time.sleep(1)
                #print('Experiment is running', self.check_exp_conditions())
                c_p['experiment_runtime'] = np.round(time.time() - start)
            else:
                patiance_counter += 1
            if patiance_counter > patiance:
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

        # TODO make the program understand when two particles have been trapped
        # in the same trap. - Possible solution: Train an AI to detect this.
        global image
        global c_p
        c_p['nbr_experiments'] = len(c_p['experiment_schedule'])
        c_p['experiment_progress'] = 0

        while c_p['program_running']: # Change to continue tracking?
            time.sleep(0.3)
            # Look through the whole shedule, a list of dictionaries.

            if c_p['tracking_on']:
                setup_dict = c_p['experiment_schedule'][c_p['experiment_progress']]
                print('Next experiment is', setup_dict)
                run_finished = False
                update_c_p(setup_dict)
                full_xm = copy.copy(c_p['xm']) # for adding one trap at a time.
                full_ym = copy.copy(c_p['ym']) # Using copy since
                time_remaining = c_p['recording_duration']
                all_filled, nbr_particles, min_index_trap, min_index_particle = self.check_exp_conditions()

                # Check if we need to go down and look for more particles in
                # between the experiments
                if all_filled:
                    nbr_active_traps = len(full_xm)
                else:
                    # Not all traps were filled, need to home and activate
                    # only the first trap
                    c_p['return_z_home'] = True
                    time.sleep(1)
                    if c_p['activate_traps_one_by_one']:
                        nbr_active_traps = min(3,len(full_xm))
                        active_traps_dict = {'xm':full_xm[:nbr_active_traps],
                            'ym':full_ym[:nbr_active_traps]}
                        update_c_p(active_traps_dict)
                # Start looking for particles.
                while not run_finished and c_p['tracking_on']:
                   time.sleep(0.3)
                   # We are (probably) in full frame mode looking for a particle

                   all_filled, nbr_particles, min_index_trap, min_index_particle = self.check_exp_conditions()

                   if not all_filled and nbr_particles <= c_p['traps_occupied'].count(True):
                       # Fewer particles than traps. Look for more particles.
                       c_p['return_z_home'] = True
                       search_for_particles()

                   elif not all_filled and nbr_particles > c_p['traps_occupied'].count(True):
                       # Untrapped particles and unfilled traps. Catch particles
                       self.catch_particle(min_index_trap=min_index_trap,
                            min_index_particle=min_index_particle)

                   elif all_filled:

                       # All active traps are filled, activate a new one if
                       # there are more to activate
                       if len(c_p['xm']) < len(full_xm):
                            nbr_active_traps += 1
                            active_traps_dict = {'xm':full_xm[:nbr_active_traps],
                                'ym':full_ym[:nbr_active_traps]}
                            update_c_p(active_traps_dict)

                       # No more traps to activate, can start lifting.
                       elif self.lift_for_experiment():
                           print('lifted!')
                           # Particles lifted, can start experiment.
                           time_remaining = self.run_experiment(time_remaining)
                       else:
                           c_p['return_z_home'] = True

                   if time_remaining < 1:
                       run_finished = True
                       c_p['experiment_progress'] += 1

        c_p['return_z_home'] = True


def get_adjacency_matrix(nx, ny):
    '''
    Function for calculating the adjacency matrix used in graph theory to
    describe which nodes are neighbours.
    '''
    X, Y = np.meshgrid(
        np.arange(0, nx),
        np.arange(0, ny)
    )

    nbr_nodes = nx*ny
    XF = np.reshape(X, (nbr_nodes, 1))
    YF = np.reshape(Y, (nbr_nodes, 1))
    adjacency_matrix = np.zeros((nbr_nodes, nbr_nodes))
    for idx in range(nx*ny):
        distance_map = (X - XF[idx])**2 + (Y - YF[idx])**2
        adjacency_matrix[idx, :] = np.reshape(distance_map, (nbr_nodes)) <= 3
        adjacency_matrix[idx, idx] = 0
    return adjacency_matrix


def path_search(filled_traps_locs, target_particle_location,
                target_trap_location):
    '''
    Function for finding paths to move the stage so as to trap more particles
    without accidentally trapping extra particles.
    Divides the AOI into a grid and calculates the shortest path to the trap
    without passing any already occupied traps.

    Parameters
    ----------
    filled_traps_locs : TYPE list of list of
        traps locations [[x1, y1], [x2, y2]...]
        DESCRIPTION.
    target_particle_location : TYPE list [x,y] of target particle location [px]
        DESCRIPTION.
    target_trap_location : TYPE list of target trap location [px]
        DESCRIPTION.

    Returns
    -------
    TYPE move_x, move_y, success
        DESCRIPTION. The move to make to try and trap the particle without it
        getting caught in another trap along the way
        success - True if path was not blocked by other particles and a move
        was found. False otherwise.

    '''
    # TODO Make this more efficient. Also make it possible to try and
    # move in between particles.
    global c_p

    nx = int( (c_p['AOI'][1]-c_p['AOI'][0]) / c_p['cell_width'])
    ny = int( (c_p['AOI'][3]-c_p['AOI'][2]) / c_p['cell_width'])
    X, Y = np.meshgrid(
        np.arange(0, nx),
        np.arange(0, ny)
    )


    nbr_nodes = nx*ny
    node_weights = 1e6 * np.ones((nbr_nodes, 1))  # Initial large weights
    unvisited_set = np.zeros(nbr_nodes)  # Replace with previous nodes
    previous_nodes = -1 * np.ones(nbr_nodes)

    def matrix_to_array_index(x, y, nx):
        return x + y * nx

    def array_to_matrix_index(idx, nx):
        y = idx // nx
        x = np.mod(idx, nx)
        return x, y

    def loc_to_index(x, y, nx):
        x = int(x/c_p['cell_width'])
        y = int(y/c_p['cell_width'])
        return matrix_to_array_index(x, y, nx)

    adjacency_matrix = get_adjacency_matrix(nx, ny)

    trap_radii = 3
    for location in filled_traps_locs:
        print(location)
        x = location[0] / c_p['cell_width']
        y = location[1] / c_p['cell_width']
        distance_map = (X - x)**2 + (Y - y)**2
        indices = [i for i, e in enumerate(np.reshape(distance_map, (nbr_nodes))) if e < trap_radii]
        adjacency_matrix[:, indices] = 0
        node_weights[indices] = 50
        node_weights[matrix_to_array_index(int(x), int(y), nx)] = 40
        unvisited_set[matrix_to_array_index(int(x), int(y), nx)] = 1

    target_node = loc_to_index(target_trap_location[0],
                               target_trap_location[1], nx)
    current_node = loc_to_index(target_particle_location[0],
                                target_particle_location[1], nx)

    # Djikstra:
    node_weights[current_node] = 0
    unvisited_set[current_node] = 1
    previous_nodes[current_node] = 0

    def update_dist(current_node, adjacency_indices, node_weights,
                    previous_nodes):
        for index in adjacency_indices:
            if node_weights[current_node] + 1 < node_weights[index]:
                node_weights[index] = node_weights[current_node] + 1
                # All distances are either inf or 1
                previous_nodes[index] = current_node

    def find_next_node(unvisited_set, node_weights):
        res_list = [i for i, value in enumerate(unvisited_set) if value == 0]
        min_value = 1e6
        min_idx = -1
        for index in res_list:
            if node_weights[index] < min_value:
                min_idx = index
                min_value = node_weights[index]
        return min_idx

    iterations = 0
    while unvisited_set[target_node] == 0 and iterations <= nbr_nodes:
        adjacency_indices = [i for i, e in enumerate(adjacency_matrix[current_node,:]) if e == 1]
        update_dist(current_node, adjacency_indices,
                    node_weights, previous_nodes)
        current_node = find_next_node(unvisited_set, node_weights)
        unvisited_set[current_node] = 1
        iterations += 1

    previous = target_node
    node_weights[previous]
    prev_x = []
    prev_y = []

    while previous != 0:
        node_weights[previous] = -3
        tmp_x, tmp_y = array_to_matrix_index(previous, nx)
        prev_x.append(tmp_x)
        prev_y.append(tmp_y)
        previous = int(previous_nodes[previous])
        if previous == -1:
            break

    if previous == -1:
        return 0, 0, False
    elif len(prev_x) > 3:
        x_move = prev_x[-3] * c_p['cell_width'] - target_particle_location[0]
        # SHould be -2 but was very slow
        y_move = prev_y[-3] * c_p['cell_width'] - target_particle_location[1]
        return x_move, y_move, True
    else:
        try:
            x_move = prev_x[-2] * c_p['cell_width'] - target_particle_location[0]
            y_move = prev_y[-2] * c_p['cell_width'] - target_particle_location[1]
            return x_move, y_move, True
        except:
            return 0, 0, False


def update_c_p(update_dict, wait_for_completion=True):
    '''
    Simple function for updating c_p['keys'] with new values 'values'.
    Ensures that all updates where successfull.
    Parameter wait_for_completion should be set to True if there is a need
    to wait for phasemask to be finished updating before continuing the program.
    '''
    ok_parameters = ['use_LGO', 'LGO_order', 'xm', 'ym', 'setpoint_temperature',
    'recording_duration', 'target_experiment_z', 'SLM_iterations',
    'temperature_output_on','activate_traps_one_by_one','need_T_stable']

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

    # Wait for new phasemask if user whishes this
    while c_p['new_phasemask'] and wait_for_completion:
        time.sleep(0.3)

    # Await stable temperature
    while c_p['need_T_stable'] and not c_p['temperature_stable'] and\
        c_p['temperature_controller_connected']:
        time.sleep(0.3)


def count_interior_particles(margin=30):
    '''
    Function for counting the number of particles in the interior of the frame.
    margin
    '''
    global c_p
    interior_particles = 0
    for position in c_p['particle_centers']:
        interior_particles += 1

    return interior_particles


def set_AOI(half_image_width=50, left=None, right=None, up=None, down=None):
    '''
    Function for changing the Area Of Interest for the camera to the box specified by
    left,right,top,bottom
    Assumes global access to c_p
    '''
    global c_p

    # Do not want motors to be moving when changing AOI!
    c_p['motor_locks'][0].acquire()
    c_p['motor_locks'][1].acquire()
    # If exact values have been provided for all the corners change AOI
    if c_p['camera_model'] == 'ThorlabsCam':
        if left is not None and right is not None and up is not None and down is not None:
            if 0<=left<=1279 and left<=right<=1280 and 0<=up<=1079 and up<=down<=1080:
                c_p['AOI'][0] = left
                c_p['AOI'][1] = right
                c_p['AOI'][2] = up
                c_p['AOI'][3] = down
            else:
                print("Trying to set invalid area")
    else:
        if left is not None and right is not None and up is not None and down is not None:
            if 0<=left<672 and left<=right<=672 and 0<=up<512 and up<=down<=512:
                c_p['AOI'][0] = left
                c_p['AOI'][1] = right
                c_p['AOI'][2] = up
                c_p['AOI'][3] = down
            else:
                print("Trying to set invalid area")

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


def find_focus():
    """
    Function which uses the laser to find a focus point
    """

    # Idea - Search for a focus, first down 1000-ticks then up 2000 ticks from down pos
    # Take small steps :10-20 per step. Between each step check if intensity
    # Around the traps have increased enough
    #Direction focus down
    #
    focus_found = False
    while not focus_found:


        print('a')


def in_focus(margin=40):
    '''
    Function for determining if a image is in focus by looking at the intensity
    close to the trap positions and comparing it to the image median.
    # Highly recommended to change to only one trap before using this function
    # This function is also very unreliable. Better to use the old method +
    # deep learning I believe.
    '''

    global image
    global c_p
    median_intesity = np.median(image)


    if c_p['camera_model'] == 'basler':
        image_limits = [672,512]
    else:
        image_limits = [1200,1000]

    left = int(max(min(c_p['traps_absolute_pos'][0]) - margin, 0))
    right = int(min(max(c_p['traps_absolute_pos'][0]) + margin, image_limits[0]))
    up = int(max(min(c_p['traps_absolute_pos'][1]) - margin, 0))
    down = int(min(max(c_p['traps_absolute_pos'][1]) + margin, image_limits[1]))

    expected_median = (left - right) * (up - down) * median_intesity
    actual_median = np.sum(image[left:right,up:down])

    print(median_intesity,actual_median)
    print( expected_median + c_p['focus_threshold'])
    if actual_median > (expected_median + c_p['focus_threshold']):
        return True
    return False


def predict_particle_position_network(network,half_image_width=50,
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
    resized = cv2.resize(copy.copy(image), (network_image_width,network_image_width), interpolation = cv2.INTER_AREA)
    pred = network.predict(np.reshape(resized/255,[1,network_image_width,network_image_width,1]))

    c_p['target_particle_center'][0] = half_image_width + pred[0][1] * half_image_width
    c_p['target_particle_center'][1] = half_image_width + pred[0][0] * half_image_width

    if print_position:
        print('Predicted posiiton is ',c_p['particle_centers'])


def get_particle_trap_distances():
    '''
    Calcualtes the distance between all particles and traps and returns a
    distance matrix, ordered as distances(traps,particles),
    To clarify the distance between trap n and particle m is distances[n][m].
    '''
    global c_p
    update_traps_relative_pos() # just in case
    nbr_traps = len(c_p['traps_relative_pos'][0])
    nbr_particles = len(c_p['particle_centers'][0])
    distances = np.ones((nbr_traps, nbr_particles))

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


def toggle_temperature_output():
    '''
    Function for toggling temperature output on/off.
    '''
    c_p['temperature_output_on'] = not c_p['temperature_output_on']
    print("c_p['temperature_output_on'] set to",c_p['temperature_output_on'])


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


def zoom_in(margin=60, use_traps=False):
    '''
    Helper function for zoom button and zoom function.
    Zooms in on an area around the traps
    '''
    if c_p['camera_model'] == 'ThorlabsCam':
        left = max(min(c_p['traps_absolute_pos'][0]) - margin, 0)
        left = int(left // 20 * 20)
        right = min(max(c_p['traps_absolute_pos'][0]) + margin, 1200)
        right = int(right // 20 * 20)
        up = max(min(c_p['traps_absolute_pos'][1]) - margin, 0)
        up = int(up // 20 * 20)
        down = min(max(c_p['traps_absolute_pos'][1]) + margin, 1000)
        down = int(down // 20 * 20)
    else:
        left = max(min(c_p['traps_absolute_pos'][0]) - margin, 0)
        left = int(left // 16 * 16)
        right = min(max(c_p['traps_absolute_pos'][0]) + margin, 672)
        right = int(right // 16 * 16)
        up = max(min(c_p['traps_absolute_pos'][1]) - margin, 0)
        up = int(up // 16 * 16)
        down = min(max(c_p['traps_absolute_pos'][1]) + margin, 512)
        down = int(down // 16 * 16)

    c_p['framerate'] = 500
    # Note calculated framerate is automagically saved.
    set_AOI(left=left, right=right, up=up, down=down)


def zoom_out():
    if c_p['camera_model'] == 'ThorlabsCam':
        set_AOI(left=0, right=1200, up=0, down=1000)
        c_p['framerate'] = 500
    else:
        set_AOI(left=0, right=672, up=0, down=512)


def search_for_particles():
    '''
    Function for searching after particles. Threats the sample as a grid and
    systmatically searches it
    '''
    x_max = 0.005 # [mm]
    delta_y = 3 # [mm]
    # Make movement
    if c_p['search_direction']== 'right':
        c_p['motor_movements'][0] = 300 # [px]

    elif c_p['search_direction']== 'left':
        c_p['motor_movements'][0] = -300

    elif c_p['search_direction']== 'up':
        c_p['motor_movements'][1] = 300

    elif c_p['search_direction']== 'down': # currently not used
        c_p['motor_movements'][1] = -300

    if c_p['search_direction']== 'up' and \
        (c_p['motor_current_pos'][1]-c_p['motor_starting_pos'][1])>delta_y:
        c_p['search_direction']= 'right'
        c_p['x_start'] = c_p['motor_current_pos'][0]
        print('changing search direction to right, x_start is ',c_p['x_start'] )

    if c_p['search_direction']== 'right' and \
        (c_p['motor_current_pos'][0]-c_p['x_start'])>x_max:

        if c_p['motor_current_pos'][1] - c_p['motor_starting_pos'][1]>delta_y/2:
            c_p['search_direction'] = 'down'
            print('changing search direction to down')
        else:
            c_p['search_direction'] = 'up'
            print('changing search direction to up')
    if c_p['search_direction']== 'down' \
        and (c_p['motor_current_pos'][1] - c_p['motor_starting_pos'][1])<0:
        c_p['search_direction']=='right'
        print('changing search direction to right')


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
global image
if c_p['camera_model'] == 'ThorlabsCam':
    image = np.zeros((c_p['AOI'][1]-c_p['AOI'][0], c_p['AOI'][3]-c_p['AOI'][2], 1))
else:
    image = np.zeros((672,512,1))

# Create a empty list to put the threads in
thread_list = []
d0x = -80e-6
d0y = -80e-6

# Define experiment to be run

xm1, ym1 = SLM.get_xm_ym_rect(nbr_rows=3, nbr_columns=3, d0x=d0x, d0y=d0y, dx=20e-6, dy=20e-6,)
# xm2, ym2 = SLM.get_xm_ym_rect(nbr_rows=1, nbr_columns=2, d0x=d0x, d0y=d0y, dx=12e-6, dy=20e-6,)
# xm3, ym3 = SLM.get_xm_ym_rect(nbr_rows=1, nbr_columns=2, d0x=d0x, d0y=d0y, dx=14e-6, dy=20e-6,)
# xm4, ym4 = SLM.get_xm_ym_rect(nbr_rows=1, nbr_columns=2, d0x=d0x, d0y=d0y, dx=16e-6, dy=20e-6,)
# xm5, ym5 = SLM.get_xm_ym_rect(nbr_rows=1, nbr_columns=2, d0x=d0x, d0y=d0y, dx=18e-6, dy=20e-6,)
# xm6, ym6 = SLM.get_xm_ym_rect(nbr_rows=1, nbr_columns=2, d0x=d0x, d0y=d0y, dx=20e-6, dy=20e-6,)
# xm7, ym7 = SLM.get_xm_ym_rect(nbr_rows=1, nbr_columns=2, d0x=d0x, d0y=d0y, dx=22e-6, dy=20e-6,)
# xm8, ym8 = SLM.get_xm_ym_rect(nbr_rows=1, nbr_columns=2, d0x=d0x, d0y=d0y, dx=24e-6, dy=20e-6,)
# xm9, ym9 = SLM.get_xm_ym_rect(nbr_rows=1, nbr_columns=2, d0x=d0x, d0y=d0y, dx=26e-6, dy=20e-6,)
# xm10, ym10 = SLM.get_xm_ym_rect(nbr_rows=1, nbr_columns=2, d0x=d0x, d0y=d0y, dx=30e-6, dy=20e-6,)


experiment_schedule = [
{'xm':xm1, 'ym':ym1, 'use_LGO':[False],'target_experiment_z':1000,
'LGO_order':4,  'recording_duration':1000,'SLM_iterations':30,'activate_traps_one_by_one':False}, # Should use few iteratoins when we only have 2 traps
# {'LGO_order':-4,'xm':xm1, 'ym':ym1, 'use_LGO':[True]},
# {'LGO_order':8,'xm':xm1, 'ym':ym1, 'use_LGO':[True]},
# {'LGO_order':-8,'xm':xm1, 'ym':ym1, 'use_LGO':[True]},
# {'LGO_order':12,'xm':xm1, 'ym':ym1, 'use_LGO':[True]},
# {'LGO_order':-12,'xm':xm1, 'ym':ym1, 'use_LGO':[True]},
# {'LGO_order':16,'xm':xm1, 'ym':ym1, 'use_LGO':[True]},
# {'LGO_order':-16,'xm':xm1, 'ym':ym1, 'use_LGO':[True]},

# {'xm':xm3, 'ym':ym3, 'use_LGO':[False,]},
# {'xm':xm4, 'ym':ym4, 'use_LGO':[False,]},
# {'xm':xm5, 'ym':ym5, 'use_LGO':[False,]},
# {'xm':xm6, 'ym':ym6, 'use_LGO':[False,]},
# {'xm':xm7, 'ym':ym7, 'use_LGO':[False,]},
# {'xm':xm8, 'ym':ym8, 'use_LGO':[False,]},
# {'xm':xm9, 'ym':ym9, 'use_LGO':[False,]},
# {'xm':xm10, 'ym':ym10, 'use_LGO':[False,]},

]

c_p['experiment_schedule'] = experiment_schedule
T_D = UserInterface(tkinter.Tk(), "Control display")

sys.exit()
