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
        'framerate': 10,
        'recording': False,
        'tracking_on': True,
        'setpoint_temperature': 25,
        'current_temperature': 25,
        'starting_temperature': 25,
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
        'z_x_diff': -200,  # Used for compensating drift in z when moving the
        # sample. Caused by sample being slightly tilted Needs to be calibrated
        'z_y_diff': -400,
        'temperature_z_diff': -190,  # How much the objective need to be moved

        'slm_x_center':1274, # needs to be recalibrated if camera is moved
        'slm_y_center':1136,
        'slm_to_pixel': 4550000.0,
        # to compensate for the changes in temperature.Measured in
        # [ticks/deg C]
        'return_z_home': False,
        'particle_threshold': 120,
        'particle_size_threshold': 200,  # Parcticle detection threshold
        'bright_particle': True,  # Is particle brighter than the background?
        'xy_movement_limit': 1200,
        'motor_locks': [threading.Lock(), threading.Lock()]
    }

    # Set traps positions
    c_p['traps_absolute_pos'] = np.zeros((2,1))
    c_p['traps_relative_pos'] = np.zeros((2,1))

    # Position of first trap
    c_p['traps_absolute_pos'][0][0] = 678
    c_p['traps_absolute_pos'][1][0] = 465
    c_p['traps_relative_pos'][0][0] = 678
    c_p['traps_relative_pos'][1][0] = 465

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
    time.sleep(1)
    global thread_list
    for thread in thread_list:
        thread.join()
    for thread in thread_list:
        del thread


def start_threads():
    """
    Function for starting all the threads, can only be called once
    """
    global thread_list
    # global temperature_controller
    camera_thread = CameraThread(1, 'Thread-camera')
    motor_X_thread = MotorThread(2, 'Thread-motorX',0)
    # Last argument is to indicate that it is the x-motor and not the y
    motor_Y_thread = MotorThread(3, 'Thread-motorY',1)
    slm_thread =CreateSLMThread(4, 'Thread-SLM')
    tracking_thread = TrackingThread(5, 'Tracker_thread')
    temperature_thread = TemperatureThread(6, 'Temperature_thread')
    z_thread = z_movement_thread(6,
                                 'z-thread',
                                 serial_no=c_p['serial_no_piezo'],
                                 channel=c_p['channel'])

    camera_thread.start()
    motor_X_thread.start()
    motor_Y_thread.start()
    tracking_thread.start()
    slm_thread.start()
    z_thread.start()
    temperature_thread.start()
    print('Camera, SLM, tracking, motor_X and motor_Y threads created')
    thread_list.append(temperature_thread)
    thread_list.append(camera_thread)
    thread_list.append(motor_X_thread)
    thread_list.append(motor_Y_thread)
    thread_list.append(tracking_thread)
    thread_list.append(slm_thread)
    thread_list.append(z_thread)


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
        global c_p
        nbr_active_traps = 2
        max_nbr_traps = 9

        xm, ym = SLM.get_default_xm_ym()
        print('xm=', xm, '\n ym=', ym)
        screen_x = [578, 727, 877, 578, 727, 877, 578, 727, 877]
        screen_y = [465, 465, 465, 615, 615, 615, 765, 765, 765]
        Delta, N, M = SLM.get_delta(xm=xm[:nbr_active_traps],
                                    ym=ym[:nbr_active_traps])
        c_p['phasemask'] = SLM.GSW(
            N, M, Delta, nbr_iterations=c_p['SLM_iterations'])
        c_p['phasemask_updated'] = True

        SLM_loc_to_trap_loc(xm=xm[:nbr_active_traps], ym=ym[:nbr_active_traps])
        print(c_p['traps_absolute_pos'])
        print(c_p['traps_relative_pos'])

        c_p['traps_occupied'] =\
            [False for i in range(len(c_p['traps_absolute_pos'][0]))]

        while c_p['continue_capture']:
            if c_p['new_phasemask']:
                # Update number of traps in use
                nbr_active_traps += 1

                if nbr_active_traps<max_nbr_traps+1:
                    # Calcualte new delta and phasemask
                    Delta,N,M = SLM.get_delta(
                        xm=xm[:nbr_active_traps], ym=ym[:nbr_active_traps])
                    c_p['phasemask'] = SLM.GSW(
                        N, M, Delta,
                        nbr_iterations=c_p['SLM_iterations'])
                    c_p['phasemask_updated'] = True
                    c_p['new_phasemask'] = False
                    # Update the number of traps and their position

                    SLM_loc_to_trap_loc(xm=xm[:nbr_active_traps],
                        ym=ym[:nbr_active_traps])
                    print(c_p['traps_absolute_pos'])
                    print(c_p['traps_relative_pos'])

                    c_p['traps_occupied'] =\
                        [False for i in range(len(c_p['traps_absolute_pos'][0]))]

                else:
                    # All traps already in position, no need to calculate phasemask
                    nbr_active_traps = max_nbr_traps
                # Acknowledge that a new phasemask was recived

            time.sleep(1)


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

                    if max(np.abs(self.temperature_history -
                        c_p['set_temperature']))< self.max_diff:

                        c_p['temperature_stable'] = True

                    else:
                        c_p['temperature_stable'] = False

                    time.sleep(1) # We do not need to update the temperature very often
                self.temperature_controller.turn_off_output()


class TkinterDisplay:

    def __init__(self, window, window_title,):
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
        start_threads()

        self.window.mainloop()
    def create_buttons(self,top):
        def get_y_separation(start=50, distance=40):
            # Simple generator to avoid printing all the y-positions of the
            # buttons

            index = 0
            while True:
                yield start + (distance * index)
                index += 1

        global c_p

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
        self.temperature_label.config(text=temperature_text)
        position_text = 'x: '+str(c_p['motor_current_pos'][0])+\
            ' y: '+str(c_p['motor_current_pos'][1])+\
            ' z: '+str(c_p['motor_current_pos'][2])
        self.position_label.config(text=position_text)
    def resize_display_image(self, img):
        img_size = np.shape(img)
        #print(img_size)
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
        video_name = c_p['recording_path'] + '/moive-' + \
            str(now.hour) + '-' + str(now.minute) + '-' + str(now.second)+'.avi'
        video = VideoWriter(video_name, fourcc,
            float(c_p['framerate']),
            (image_width, image_height), isColor=False)
        return video, video_name
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
               video,video_name = self.create_video_writer()
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
           print('Capture sequence finished', image_count,
                'Images captured in ', end-start, 'seconds. \n FPS is ',
                image_count/(end-start))


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
       global c_p
       while c_p['continue_capture']: # Change to continue tracking?

            if c_p['tracking_on']:

                   '''
                   We are in full frame mode looking for a particle
                   '''
                   time.sleep(0.2)
                   x,y = fpt.find_particle_centers(copy.copy(image),
                                                    threshold=c_p['particle_threshold'],
                                                    particle_size_threshold=c_p['particle_size_threshold'],
                                                    bright_particle=c_p['bright_particle'])
                   c_p['particle_centers'] = [x,y]

                   # Find the closest particles
                   if len(x)>0: # Check that there are particles present

                       min_index_trap,min_index_particle = find_closest_unoccupied()
                       if min_index_particle is not None:
                           c_p['target_trap_pos'] = [c_p['traps_relative_pos'][0][min_index_trap],c_p['traps_relative_pos'][1][min_index_trap]]
                           c_p['target_particle_center'] = [c_p['particle_centers'][0][min_index_particle],c_p['particle_centers'][1][min_index_particle]]
                           c_p['motor_movements'][0] = -(c_p['target_trap_pos'][0] - c_p['target_particle_center'][0]) # Note: Sign of this depends on setup
                           c_p['motor_movements'][1] = c_p['target_trap_pos'][1] - c_p['target_particle_center'][1]

                           # If there is a trapped particle then we do not want to move very far so we accidentally lose it
                           if True in c_p['traps_occupied']:
                               c_p['xy_movement_limit'] = 40
                           else:
                               c_p['xy_movement_limit'] = 1200
                       if c_p['traps_occupied'].count(True)>8:
                           c_p['z_movement'] = 40
                           c_p['return_z_home'] = False
                           print("LIFTING TIME!")
                           if  c_p['AOI'][1]-c_p['AOI'][0]>900:
                               zoom_in(margin=120)
                       else:
                           if c_p['AOI'][1]-c_p['AOI'][0]<900:
                               zoom_out()
                   else:
                       c_p['target_particle_center'] = []

                   # If there are no untrapped particles in the frame, go search for some.
                   if False in c_p['traps_occupied']:
                       c_p['return_z_home'] = True
                   if len(x) == c_p['traps_occupied'].count(True) and False in c_p['traps_occupied']:
                       search_for_particles()
                       # No untrapped particles
                   if False not in c_p['traps_occupied']:
                           c_p['new_phasemask'] = True
            time.sleep(0.3) # Needed to prevent this thread from running too fast


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
    # else:
    #     # Use defult center
    #     c_p['AOI'] = [0,2*half_image_width,0,2*half_image_width]
    print('Setting AOI to ',c_p['AOI'])

    # Inform the camera and display thread about the updated AOI
    c_p['new_AOI_camera'] = True
    c_p['new_AOI_display'] = True

    # Update trap relative position
    c_p['traps_relative_pos'][0] = [x - c_p['AOI'][0] for x in c_p['traps_absolute_pos'][0]]
    c_p['traps_relative_pos'][1] = [y - c_p['AOI'][2] for y in c_p['traps_absolute_pos'][1]]
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
    nbr_traps = len(c_p['traps_absolute_pos'][0])
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
        if dist_to_trap<=c_p['movement_threshold']:
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


def zoom_in(margin=50):
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

    c_p['framerate'] = 100 # Todo fix this so that it is better
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
    global c_p
    tmp_x = [x * c_p['slm_to_pixel'] + c_p['slm_x_center'] for x in xm]
    tmp_y = [y * c_p['slm_to_pixel'] + c_p['slm_y_center'] for y in ym]
    tmp = np.asarray([tmp_x, tmp_y])
    c_p['traps_absolute_pos'] = tmp
    print(c_p['traps_absolute_pos'][0] )
    update_traps_relative_pos()
############### Main script starts here ####################################
c_p = get_default_c_p()
# Create camera and set defaults
cam = TC.get_camera()
cam.set_defaults(left=c_p['AOI'][0], right=c_p['AOI'][1], top=c_p['AOI'][2], bot=c_p['AOI'][3], n_frames=1)
exposure_time = TC.find_exposure_time(cam) # automagically finds a decent exposure time
print('Exposure time = ', exposure_time)
#TC.set_exposure(cam,85)
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
