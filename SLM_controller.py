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
def get_default_control_parameters(
    SLM_iterations = 30,
    phasemask_width = 1080,
    phasemask_height = 1080,
    ):
    '''
    Function for creating starting parameters of phasemask
    '''
    control_parameters = {
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
        'dx':20e-6,
        'dy':20e-6,
        'nbr_SLM_rows':2,
        'nbr_SLM_columns':1,
        'use_LGO':[True]
    }
    control_parameters['phasemask'] = np.zeros((control_parameters['phasemask_height'],control_parameters['phasemask_width']))

    control_parameters['traps_absolute_pos'] = np.zeros((2,1)) # This will need updating

    # Position of first trap
    control_parameters['traps_absolute_pos'][0][0] = 678
    control_parameters['traps_absolute_pos'][1][0] = 465

    control_parameters['xm'] = [control_parameters['d0x']]
    control_parameters['ym'] = [control_parameters['d0y']]

    return control_parameters
def start_threads():
    """
    Function for starting all the threads, can only be called once
    """
    global thread_list
    global control_parameters
    slm_thread =CreateSLMThread(1,'Thread-SLM')
    slm_thread.start()
    print('SLM thread started')

class CreateSLMThread(threading.Thread):
    def __init__(self,threadID,name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.setDaemon(True)
        self.update_xm_ym()
    def update_xm_ym(self):
        global control_parameters
        control_parameters['xm'],control_parameters['ym']= SLM.get_xm_ym_rect(
                nbr_rows=control_parameters['nbr_SLM_rows'],
                nbr_columns=control_parameters['nbr_SLM_columns'],
                dx=control_parameters['dx'],
                dy=control_parameters['dy'],
                d0x=control_parameters['d0x'],
                d0y=control_parameters['d0y'])
        #control_parameters['use_LGO'] = [True for x in control_parameters['xm']]
    def run(self):
        global control_parameters
        self.update_xm_ym()
        Delta,N,M = SLM.get_delta(xm=control_parameters['xm'], ym=control_parameters['ym'], use_LGO=control_parameters['use_LGO'])
        self.generate_phasemask(Delta,N,M)
        control_parameters['phasemask_updated'] = True

        while control_parameters['experiment_running']:
            if control_parameters['new_phasemask']:
                # Calcualte new delta and phasemask
                self.update_xm_ym()
                Delta,N,M = SLM.get_delta(xm=control_parameters['xm'], ym=control_parameters['ym'], use_LGO=control_parameters['use_LGO'])
                self.generate_phasemask(Delta,N,M)

                # Let the other threads know that a new phasemask has been calculated
                control_parameters['phasemask_updated'] = True
                control_parameters['new_phasemask'] = False
            time.sleep(1)
    def generate_phasemask(self,Delta,N,M):
        global control_parameters
        if control_parameters['SLM_algorithm'] == 'GSW':
            control_parameters['phasemask'] = SLM.GSW(N, M, Delta,
                nbr_iterations=control_parameters['SLM_iterations'])
        elif control_parameters['SLM_algorithm'] == 'GS':
            control_parameters['phasemask'] = SLM.GS(N, M, Delta,
                nbr_iterations=control_parameters['SLM_iterations'])


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
         self.window.geometry('500x650')
         self.create_buttons()
         # After it is called once, the update method will be automatically called every delay milliseconds
         self.delay = 200

         self.create_SLM_window(SLM_window)
         self.create_indicators()
         self.update()
         start_threads()
         self.window.mainloop()
    def create_SLM_window(self, _class):
        try:
            if self.new.state() == "normal":
                self.new.focus()
        except:
            self.new = tkinter.Toplevel(self.window)
            self.SLM_Window = _class(self.new)
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


    def create_buttons(self):
        def get_y_separation(start=50,distance=40):
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
        def update_from_entry(tkinter_entry,key,type='int',bounds=[-np.inf,np.inf],new_mask=False,scale=1):
            entry = get_entry(tkinter_entry,type)
            if entry is not None and entry>bounds[0] and entry<bounds[1]:
                control_parameters[key] = entry*scale
                control_parameters['new_phasemask'] = new_mask
            else:
                print('Value out of bounds')
        set_iterations = lambda : update_from_entry(iterations_entry, type='int', key='SLM_iterations', bounds=[0,1000])
        set_dx = lambda : update_from_entry(dx_entry, type='float', key='dx', bounds=[0,100],scale=1e-6)
        set_dy = lambda : update_from_entry(dy_entry, type='float', key='dy', bounds=[0,100],scale=1e-6)

        set_SLM_rows = lambda : update_from_entry(nbr_trap_rows_entry, type='int', key='nbr_SLM_rows', bounds=[0,1000])
        set_SLM_columns = lambda : update_from_entry(nbr_trap_columns_entry, type='int', key='nbr_SLM_columns',bounds=[0,1000])
        set_d0x = lambda : update_from_entry(d0x_entry, type='float', key='d0x', bounds=[-200, 200], scale=1e-6)
        set_d0y = lambda : update_from_entry(d0y_entry, type='float', key='d0y', bounds=[-200, 200], scale=1e-6)

        SLM_Iterations_button = tkinter.Button(self.window, text ='Set SLM iterations', command = set_iterations)
        set_dx_button = tkinter.Button(self.window, text ='Set particle separation -x ', command = set_dx)
        set_dy_button = tkinter.Button(self.window, text ='Set particle separation -y ', command = set_dy)

        SLM_rows_button = tkinter.Button(self.window, text ='Set SLM rows', command = set_SLM_rows)
        SLM_columns_button = tkinter.Button(self.window, text ='Set SLM columns', command = set_SLM_columns)
        set_d0x_button = tkinter.Button(self.window, text ='Set d0x', command = set_d0x)
        set_d0y_button = tkinter.Button(self.window, text ='Set d0y', command = set_d0y)

        recalculate_mask_button = tkinter.Button(self.window, text ='Recalculate mask', command = recalculate_mask)
        y_position = get_y_separation()
        y_position_2 = get_y_separation() # for second column

        x_position = 50
        x_position_2 = 300

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

        self.create_algorithm_selection(x_position_2, y_position_2.__next__())
        self.create_LGO_selection(x_position_2, y_position_2.__next__())

    def create_indicators(self):
            global control_parameters
            position_text = 'Current trap separation is: ' + str(control_parameters['trap_separation'])
            self.position_label = Label(self.window,text=position_text)
            self.position_label.place(x=10,y=500)


            setup_text = 'xms are : ' + str(control_parameters['xm'])
            setup_text += '\n yms are : ' + str(control_parameters['ym'])
            self.info_label = Label(self.window,text=setup_text)
            self.info_label.place(x=10,y=570)


    def update_indicators(self):
        '''
        Helper function for updating on-screen indicators
        '''
        position_text = 'Current dx is: ' + str(control_parameters['dx']*1e6)+'  Current dy is: ' + str(control_parameters['dy']*1e6)
        position_text += '\n Number of iterations set to: ' +str(control_parameters['SLM_iterations'])
        if control_parameters['SLM_algorithm'] == 'GS':
            position_text += '\n Using Grechbgerg-Saxton algorithm'
        else:
            position_text += '\n Using Weighted Grechbgerg-Saxton algorithm'
        self.position_label.config(text=position_text)

        setup_text = 'xms are : ' + str(control_parameters['xm'])
        setup_text += '\n yms are : ' + str(control_parameters['ym'])
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
         control_parameters['SLM_algorithm'] = self.selected_algorithm.get()
         control_parameters['use_LGO'] = [self.toggle_LGO.get()]
         # if self.toggle_LGO.get():
         #     print('LGO on')
         # else:
         #     print('LGO off')
         if control_parameters['phasemask_updated']:
             self.SLM_Window.update()
             control_parameters['phasemask_updated'] = False
         self.window.after(self.delay, self.update)
    #
    def __del__(self):
         control_parameters['experiment_running'] = False
class SLM_window(Frame):
    global control_parameters
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.master = master
        # Todo, make it possible to set this wherever
        self.master.geometry("1920x1080+1920+0")#("1080x1080+2340+0")#("1920x1080+2340+0")
        self.pack(fill=BOTH, expand=1)
        self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(control_parameters['phasemask']))

        self.img = Label(self, image=self.photo )
        self.img.place(x=420, y=0)
        self.img.image = self.photo
        ####
        self.delay = 500
        self.update()
    def update(self):
        # This implementation does work but is perhaps a tiny bit janky
        self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(control_parameters['phasemask']))
        del self.img.image
        self.img = Label(self,image=self.photo)
        self.img.image = self.photo
        self.img.place(x=420, y=0) # Do not think this is needed

def recalculate_mask():
    control_parameters['new_phasemask'] = True
if __name__ == '__main__':
    control_parameters = get_default_control_parameters()
    T_D = TkinterDisplay(tkinter.Tk(), "SLM controlpanel")
