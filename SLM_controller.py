# App for controlling and displaying the SLM
import matplotlib.pyplot as plt
import numpy as np
import threading,time,cv2,queue,tkinter,os
from tkinter import messagebox
from tkinter import *
from functools import partial
import PIL.Image, PIL.ImageTk
from tkinter.ttk import *
import SLM
from read_dict_from_file import ReadFileToExperimentList
from tkinter.filedialog import askopenfilename
# TODO: Put some of the experiment parameters, such as slm_y_center, in a separate
# file which both this module and automagic experiments have access to.
def get_default_c_p(
    SLM_iterations = 2,
    phasemask_width = 1080,
    phasemask_height = 1080,
    ):
    '''
    Function for creating starting parameters of phasemask
    '''
    c_p = {
        'new_phasemask' : False, # True if the phasemask is to be recalculated
        'phasemask_updated' : False, # True if the phasemask image needs to be updated
        'SLM_iterations' : int(SLM_iterations),
        'trap_separation' : 20e-6,
        'phasemask_width' : int(phasemask_width),
        'phasemask_height' : int(phasemask_height),
        'experiment_running' : True,
        'phasemask_position' : 2340,
        'SLM_algorithm' : 'GSW',
        'd0x':-115e-6,
        'd0y':-115e-6,
        'd0z':0,
        'slm_x_center': 558,#700-142,# needs to be recalibrated if camera is moved.
        # This is the position of the 0th order of the SLM (ie where the trap)
        # with xm=ym=0 is located in camera coordinates
        'slm_y_center': 576,#605-29,
        'slm_to_pixel':5000000, # Basler
        #4550000.0, #thorlabs
        'x_comp':3.2e4,
        'y_comp':9e4,
        'dx':20e-6,
        'dy':20e-6,
        'nbr_SLM_rows':1,
        'nbr_SLM_columns':1,
        'use_LGO':[True],
        'LGO_order':-8,

        # TODO add display option for traps locations
    }
    c_p['phasemask'] = np.zeros((c_p['phasemask_height'],c_p['phasemask_width']))

    c_p['traps_absolute_pos'] = np.zeros((2,1)) # This will need updating

    # Position of first trap
    c_p['traps_absolute_pos'][0][0] = 678
    c_p['traps_absolute_pos'][1][0] = 465

    c_p['xm'] = [c_p['d0x']]
    c_p['ym'] = [c_p['d0y']]
    c_p['zm'] = [c_p['d0z']]

    return c_p


def start_threads():
    """
    Function for starting all the threads, can only be called once
    """
    global thread_list
    global c_p
    slm_thread =CreateSLMThread(1,'Thread-SLM')
    slm_thread.start()
    print('SLM thread started')


def update_xm_ym():
    global c_p
    c_p['xm'], c_p['ym'] = SLM.get_xm_ym_rect(
            nbr_rows=c_p['nbr_SLM_rows'],
            nbr_columns=c_p['nbr_SLM_columns'],
            dx=c_p['dx'],
            dy=c_p['dy'],
            d0x=c_p['d0x'],
            d0y=c_p['d0y'])
    update_trap_locs()


def update_trap_locs():
    global c_p
    # If units are pixels, then translate to "SLM coordinates"
    if min(c_p['xm']) >= 1:
        print(c_p['xm'])
        c_p['xm'] = pixels_to_SLM_locs(c_p['xm'], 0)
    if min(c_p['ym']) >= 1:
        c_p['ym'] = pixels_to_SLM_locs(c_p['ym'], 1)
    SLM_loc_to_trap_loc(c_p['xm'], c_p['ym'])


class CreateSLMThread(threading.Thread):
    '''
    Thread for calculating the new phasemasks in the background.
    '''
    def __init__(self,threadID,name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.setDaemon(True)
        update_xm_ym()

    def run(self):
        global c_p
        update_xm_ym()
        c_p['zm'] = np.ones(len(c_p['xm'])) * c_p['d0z']

        Delta, N, M = SLM.get_delta(xm=c_p['xm'],
            ym=c_p['ym'],
            zm=c_p['zm'],
            use_LGO=c_p['use_LGO'],
            order = c_p['LGO_order'])
        self.generate_phasemask(Delta, N, M)
        c_p['phasemask_updated'] = True

        while c_p['experiment_running']:
            if c_p['new_phasemask']:
                # Calcualte new delta and phasemask
                c_p['zm'] = np.ones(len(c_p['xm'])) * c_p['d0z']
                Delta,N,M = SLM.get_delta(xm=c_p['xm'],
                    ym=c_p['ym'],
                    zm=c_p['zm'],
                    use_LGO=c_p['use_LGO'],
                    order=c_p['LGO_order'],
                    x_comp=c_p['x_comp'],
                    y_comp=c_p['y_comp'])
                self.generate_phasemask(Delta,N,M)

                # Let the other threads know that a new phasemask has been calculated
                c_p['phasemask_updated'] = True
                c_p['new_phasemask'] = False
            time.sleep(1)
    def generate_phasemask(self,Delta,N,M):
        global c_p
        if c_p['SLM_algorithm'] == 'GSW':
            c_p['phasemask'] = SLM.GSW(N, M, Delta,
                nbr_iterations=c_p['SLM_iterations'])
        elif c_p['SLM_algorithm'] == 'GS':
            c_p['phasemask'] = SLM.GS(N, M, Delta,
                nbr_iterations=c_p['SLM_iterations'])


    def calculate_trap_position():
        '''
        Function for converting trap positions in the phasemask to positions on the
        screen. Needs  to be calibrated for each setup. Therefore not implemented yet.
        '''
        pass


class TkinterDisplay:

    def __init__(self, window, window_title,):
         self.window = window
         self.window.title(window_title)

         # Create a canvas that can fit the above video source size
         self.window.geometry('700x500')

         self.canvas_width = 240
         self.canvas_height = 200
         self.canvas = tkinter.Canvas(
             window, width=self.canvas_width, height=self.canvas_height)
         self.canvas.place(x=0, y=0)

         self.create_buttons()
         # After it is called once, the update method will be automatically called every delay milliseconds
         self.delay = 200

         self.create_SLM_window(SLM_window)
         self.create_indicators()
         self.update()
         start_threads()
         self.window.mainloop()
         self.mini_image = np.zeros((120,100,3))
    def create_SLM_window(self, _class):
        try:
            if self.new.state() == "normal":
                self.new.focus()
        except:
            self.new = tkinter.Toplevel(self.window)
            self.SLM_Window = _class(self.new)
    def create_trap_image(self, trap_x=[], trap_y=[], particle_x=[], particle_y=[], AOI=[0,1200,0,1000]):

        # Define new mini-image
        mini_image = np.zeros((200,240,3))
        scale_factor = 5
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
                x = int(round(x/scale_factor))
                y = int(round(y/scale_factor))
                if 1 <= x <= 239 and 1 <= y <= 199:
                    mini_image[y-1:y+1,x-1:x+1,2] = 255

        # Draw the AOI
        # l = int(round(AOI[0]/10))  # left
        # r = int(round(AOI[1]/10))  # right
        # u = int(round(AOI[2]/10))  # up
        # d = int(round(AOI[3]/10))  # down

        l = int(round(AOI[2]/scale_factor))  # left
        r = int(round(AOI[3]/scale_factor))  # right
        u = int(round(AOI[0]/scale_factor))  # up
        d = int(round(AOI[1]/scale_factor))  # down

        # TODO make it so that it handles edges better
        try:
            mini_image[l,u:d,:] = 255  # Left edge
            mini_image[l:r,u,:] = 255  # Upper edge
            mini_image[r,u:d,:] = 255  # Right edge
            mini_image[l:r,d,:] = 255  # Bottom edge
        except:
            mini_image[0,0:-1,:] = 255  # Left edge
            mini_image[0:-1,0,:] = 255  # Upper edge
            mini_image[-1,0:-1,:] = 255  # Right edge
            mini_image[0:-1,-1,:] = 255  # Bottom edge

        self.mini_image = mini_image.astype('uint8')

    def create_algorithm_selection(self, x_pos, y_pos):
        self.selected_algorithm = StringVar()
        self.selected_algorithm.set('GSW')
        self.gsw_button = Radiobutton(self.window, text='GSW', value='GSW', variable=self.selected_algorithm)
        self.gs_button = Radiobutton(self.window, text='GS', value='GS', variable=self.selected_algorithm)
        self.gsw_button.place(x=x_pos, y=y_pos)
        self.gs_button.place(x=x_pos+50, y=y_pos)

    def create_LGO_selection(self, x_pos, y_pos):
        self.toggle_LGO = BooleanVar()
        self.toggle_LGO.set(False)
        self.LGO_on_button = Radiobutton(self.window, text='LGO on', value=True, variable=self.toggle_LGO)
        self.LGO_off_button = Radiobutton(self.window, text='LGO off', value=False, variable=self.toggle_LGO)
        self.LGO_on_button.place(x=x_pos, y=y_pos)
        self.LGO_off_button.place(x=x_pos+80, y=y_pos)

    def read_positions_from_file(self):
        '''
        Reads xm, ym from a .txt file and puts them in ...
        '''
        global c_p
        filepath = askopenfilename()
        # TODO make it so that we can handle exceptions from the file better here.
        # Bring up a confirmation menu for the schedule perhaps?
        experiment_list = ReadFileToExperimentList(filepath)
        if experiment_list is not None and len(experiment_list) > 0:
            print(experiment_list[0])
            for key in experiment_list[0]:
                # TODO: Fix some problems here
                c_p[key] = experiment_list[0][key]
        else:
            print('Invalid or empty file.')
        update_trap_locs()

    def create_buttons(self):
        def get_y_separation(start=5,distance=40):
            index = 0
            while True:
                yield start + (distance * index)
                index += 1

        iterations_entry = tkinter.Entry(self.window,bd=5)
        dx_entry = tkinter.Entry(self.window,bd=5)
        dy_entry = tkinter.Entry(self.window,bd=5)
        nbr_trap_rows_entry = tkinter.Entry(self.window,bd=5)
        nbr_trap_columns_entry = tkinter.Entry(self.window,bd=5)
        d0x_entry = tkinter.Entry(self.window,bd=5)
        d0y_entry = tkinter.Entry(self.window,bd=5)
        d0z_entry = tkinter.Entry(self.window,bd=5)
        LGO_order_entry = tkinter.Entry(self.window,bd=5)

        def get_entry(tkinter_entry,type='int'):
            entry = tkinter_entry.get()
            tkinter_entry.delete(0,last=10000)
            try:
                if type == 'int':
                    return int(entry)
                if type == 'float':
                    return float(entry)
                else:
                    return None
            except:
                print('Cannot perform conversion')
                return None
        def update_from_entry(tkinter_entry,key,type='int',bounds=[-np.inf,np.inf], new_mask=False,scale=1):
            entry = get_entry(tkinter_entry,type)
            if entry is not None and entry>bounds[0] and entry<bounds[1]:
                c_p[key] = entry*scale
                update_xm_ym()
                c_p['new_phasemask'] = new_mask
            else:
                print('Value out of bounds')

        set_iterations = lambda : update_from_entry(iterations_entry, type='int', key='SLM_iterations', bounds=[0,1000])
        set_dx = lambda : update_from_entry(dx_entry, type='float', key='dx', bounds=[0,1200],scale=1)
        set_dy = lambda : update_from_entry(dy_entry, type='float', key='dy', bounds=[0,1200],scale=1)

        set_SLM_rows = lambda : update_from_entry(nbr_trap_rows_entry, type='int', key='nbr_SLM_rows', bounds=[0,1000])
        set_SLM_columns = lambda : update_from_entry(nbr_trap_columns_entry, type='int', key='nbr_SLM_columns',bounds=[0,1000])
        set_d0x = lambda : update_from_entry(d0x_entry, type='float', key='d0x', bounds=[1, 1280], scale=1)# bounds=[-200, 200], scale=1e-6)
        set_d0y = lambda : update_from_entry(d0y_entry, type='float', key='d0y', bounds=[1, 1080], scale=1)
        set_d0z = lambda : update_from_entry(d0z_entry, type='float', key='d0z', bounds=[-200, 200], scale=1e-10)
        set_LGO_order = lambda : update_from_entry(LGO_order_entry, type='int', key='LGO_order', bounds=[-200, 200], scale=1)

        SLM_Iterations_button = tkinter.Button(self.window, text ='Set SLM iterations', command=set_iterations)
        set_dx_button = tkinter.Button(self.window, text='Set particle separation -x ', command=set_dx)
        set_dy_button = tkinter.Button(self.window, text='Set particle separation -y ', command=set_dy)

        SLM_rows_button = tkinter.Button(self.window, text='Set SLM rows', command=set_SLM_rows)
        SLM_columns_button = tkinter.Button(self.window, text='Set SLM columns', command=set_SLM_columns)
        set_d0x_button = tkinter.Button(self.window, text='Set d0x', command=set_d0x)
        set_d0y_button = tkinter.Button(self.window, text='Set d0y', command=set_d0y)
        set_d0z_button = tkinter.Button(self.window, text='Set d0z', command=set_d0z)


        set_LGO_order_button = tkinter.Button(self.window, text='Set LGO order', command=set_LGO_order)

        recalculate_mask_button = tkinter.Button(self.window, text='Recalculate mask', command=recalculate_mask)

        read_positions_button = tkinter.Button(self.window, text='load positions from file', command=self.read_positions_from_file)

        y_position = get_y_separation()
        y_position_2 = get_y_separation() # for second column

        x_position = 310
        x_position_2 = 500

        # Column 1
        recalculate_mask_button.place(x=x_position,y=y_position.__next__())

        iterations_entry.place(x=x_position,y=y_position.__next__())
        SLM_Iterations_button.place(x=x_position,y=y_position.__next__())

        dx_entry.place(x=x_position,y=y_position.__next__())
        set_dx_button.place(x=x_position,y=y_position.__next__())

        dy_entry.place(x=x_position,y=y_position.__next__())
        set_dy_button.place(x=x_position,y=y_position.__next__())

        nbr_trap_rows_entry.place(x=x_position,y=y_position.__next__())
        SLM_rows_button.place(x=x_position,y=y_position.__next__())

        nbr_trap_columns_entry.place(x=x_position,y=y_position.__next__())
        SLM_columns_button.place(x=x_position,y=y_position.__next__())

        # Column 2
        d0x_entry.place(x=x_position_2,y=y_position_2.__next__())
        set_d0x_button.place(x=x_position_2,y=y_position_2.__next__())

        d0y_entry.place(x=x_position_2,y=y_position_2.__next__())
        set_d0y_button.place(x=x_position_2,y=y_position_2.__next__())

        d0z_entry.place(x=x_position_2,y=y_position_2.__next__())
        set_d0z_button.place(x=x_position_2,y=y_position_2.__next__())

        LGO_order_entry.place(x=x_position_2,y=y_position_2.__next__())
        set_LGO_order_button.place(x=x_position_2,y=y_position_2.__next__())

        self.create_algorithm_selection(x_position_2, y_position_2.__next__())
        self.create_LGO_selection(x_position_2, y_position_2.__next__())

        read_positions_button.place(x=x_position_2,y=y_position_2.__next__())

    def create_indicators(self):
            global c_p
            position_text = 'Current trap separation is: ' + str(c_p['trap_separation'])
            self.position_label = Label(self.window,text=position_text)
            self.position_label.place(x=10,y=240)


            setup_text = 'xms are : ' + str(c_p['xm']) # TODO change so this is
            # written in console instead
            setup_text += '\n yms are : ' + str(c_p['ym'])
            self.info_label = Label(self.window,text=setup_text)
            self.info_label.place(x=10,y=410)

    def update_indicators(self):
        '''
        Helper function for updating on-screen indicators
        '''
        position_text = 'Current dx is: ' + str(c_p['dx'])+' px. Current dy is: ' + str(c_p['dy']) + ' px'
        position_text += '\n Number of iterations set to: ' +str(c_p['SLM_iterations'])
        if c_p['SLM_algorithm'] == 'GS':
            position_text += '\n Using Grechbgerg-Saxton algorithm'
        else:
            position_text += '\n Using Weighted Grechbgerg-Saxton algorithm'
        self.position_label.config(text=position_text)

        setup_text = 'x-positions are : ' + str(c_p['traps_absolute_pos'][0])
        setup_text += '\n y-positions are : ' + str(c_p['traps_absolute_pos'][1]) #c_p['ym'])
        setup_text += '\n LGO order set to: ' +str(c_p['LGO_order'])
        self.info_label.config(text=setup_text)

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
         self.update_indicators()
         c_p['SLM_algorithm'] = self.selected_algorithm.get()
         c_p['use_LGO'] = [self.toggle_LGO.get()]

         if c_p['phasemask_updated']:
             self.SLM_Window.update()
             c_p['phasemask_updated'] = False
         self.create_trap_image(trap_x=c_p['traps_absolute_pos'][0],
            trap_y=c_p['traps_absolute_pos'][1])

         self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(self.mini_image, mode='RGB'))
         self.canvas.create_image(0, 0, image = self.photo, anchor = tkinter.NW) # need to use a compatible image type

         self.window.after(self.delay, self.update)
    #
    def __del__(self):
         c_p['experiment_running'] = False


class SLM_window(Frame):
    global c_p
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.master = master
        # Todo, make it possible to set this wherever
        self.master.geometry("1920x1080+1920+0")#("1080x1080+2340+0")#("1920x1080+2340+0")
        self.pack(fill=BOTH, expand=1)
        self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(c_p['phasemask']))

        self.img = Label(self, image=self.photo )
        self.img.place(x=420, y=0)
        self.img.image = self.photo
        ####
        self.delay = 500
        self.update()
    def update(self):
        # This implementation does work but is perhaps a tiny bit janky
        self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(c_p['phasemask']))
        del self.img.image
        self.img = Label(self,image=self.photo)
        self.img.image = self.photo
        self.img.place(x=420, y=0) # Do not think this is needed


def recalculate_mask():
    c_p['new_phasemask'] = True


def pixels_to_SLM_locs(locs, axis):
    return [pixel_to_SLM_loc(x, axis) for x in locs]

def pixel_to_SLM_loc(loc, axis):
    '''
    Function for converting from PIXELS to SLM locations.
    '''
    global c_p
    if axis != 0 and axis != 1:
        print('cannot perform conversion, incorrect choice of axis')
        return locs
    offset = c_p['slm_x_center'] if not axis else c_p['slm_y_center']
    return (loc - offset) / c_p['slm_to_pixel']

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
    print(tmp)


if __name__ == '__main__':
    c_p = get_default_c_p()
    T_D = TkinterDisplay(tkinter.Tk(), "SLM controlpanel")
