import threading, cv2, os, time
from cv2 import VideoWriter, VideoWriter_fourcc
from datetime import datetime # To label the videos with date and time
import ThorlabsCam as TC # My own helper functions for the camera
import numpy as np
import matplotlib.pyplot as plt

def get_default_c_p(recording_path=None):
    '''
    Dictionary containing primarily parameters used for specifying the
    experiment and synchronizing the program threads. For instance turning
    on/off the video recording by changing the parameter c_p['recording'] to
    True/False.
    '''

    if recording_path is None:
        # Will save to C by default.
        recording_path = 'C:/Users/marti/Desktop'#'C:'
    c_p = {
        'recording_path': recording_path,
        'program_running': True,  # True if camera etc should keep updating
        'zoomed_in': False,  # Keeps track of whether the image is cropped or
        # not
        'recording': False,  # True if recording is on
        'AOI': [0, 1200, 0, 1000], # Thorlabs camera [0,1200,0,1000]
        'new_AOI_camera': False,
        'new_AOI_display': False,
        'framerate': 10,
        'tracking_on': False,
        'exposure_time':80, # ExposureTime
        'new_video':False,
        'recording_duration':3000,
        'camera_model':'ThorlabsCam',
        'measurement_name':'', # Name of measurement to use when saving data.

    }

    return c_p


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
          exposure_time = TC.find_exposure_time(self.cam, targetIntensity=100) # automagically finds a decent exposure time
          c_p['exposure_time'] = exposure_time
          print('Exposure time = ', exposure_time)
      else:
          # Get a basler camera
          tlf = pylon.TlFactory.GetInstance()
          self.cam = pylon.InstantCamera(tlf.CreateFirstDevice())
          self.cam.Open()
          image = np.zeros((672,512,1))
      self.setDaemon(True) # Helps the program quit correctly

   def __del__(self):
        if c_p['camera_model'] == 'basler':
            self.cam.Close()
        else:
            self.cam.close()

   def create_video_writer(self):
        '''
        Funciton for creating a VideoWriter.
        If no frames are written to the video then the video will still be
        created but it will not be possible to view it.
        '''
        global c_p
        now = datetime.now()
        fourcc = VideoWriter_fourcc(*'MJPG')
        image_width = c_p['AOI'][1] - c_p['AOI'][0]
        image_height = c_p['AOI'][3] - c_p['AOI'][2]

        print(image_width,image_height)
        video_name = c_p['recording_path'] + '/video-'+ c_p['measurement_name'] + \
            '-' + str(now.hour) + '-' + str(now.minute) + '-' + str(now.second)+'.avi'

        experiment_info_name =c_p['recording_path'] + '/data-' + c_p['measurement_name'] + \
            str(now.hour) + '-' + str(now.minute) + '-' + str(now.second)
        print('Image width,height,framerate',image_width,image_height,int(c_p['framerate']))
        video = VideoWriter(video_name, fourcc,
            c_p['framerate'], # Format cannot handle high framerates
            (image_height, image_width), isColor=False)
            #(image_width, image_height), isColor=False)
        return video, experiment_info_name

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
          TC.set_exposure(self.cam, c_p['exposure_time'])
          # Grab one example image
          global image
          image = self.cam.grab_image(n_frames=1)
          image_count = 0
          # Start livefeed from the camera

          # Setting  maximum framerate of capture.
          self.cam.start_live_video(
               framerate=str(c_p['framerate']) + 'hertz' )

          start = time.time()

          # Create an array to store the images which have been captured in
          if not video_created:
              video, experiment_info_name = self.create_video_writer()
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
              c_p['framerate'] = self.cam.ResultingFrameRate.GetValue()
              c_p['framerate'] = round(float(c_p['framerate']), 1)
              print('Read framerate to ', c_p['framerate'], ' fps.')
          except:
              print('Exposure time not accepted by camera')
          # Grab one example image
          image_count = 0

          global image
          self.cam.StartGrabbing()

          start = time.time()

          # Create an array to store the images which have been captured in
          if not video_created:
              video, experiment_info_name = self.create_video_writer()
              video_created = True
          # Start continously capturing images now that the camera parameters have been set
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


   def run(self):
       # This function will run when the thread is started.
       if c_p['camera_model'] == 'ThorlabsCam':
           self.thorlabs_capture()
       elif c_p['camera_model'] == 'basler':
           self.basler_capture()


def set_AOI(half_image_width=50, left=None, right=None, up=None, down=None):
    '''
    Function for changing the Area Of Interest for the camera to the box specified by
    left,right,top,bottom
    Assumes global access to c_p
    '''
    global c_p

    # Do not want motors to be moving when changing AOI!
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

################# EXAMPLE OF MAIN PROGRAM BELOW ###############################

# Get control parameters
c_p = get_default_c_p()

# Create a image which all of the threads have access to
global image

if c_p['camera_model'] == 'ThorlabsCam':
    image = np.zeros((c_p['AOI'][1]-c_p['AOI'][0], c_p['AOI'][3]-c_p['AOI'][2], 1))
else:
    image = np.zeros((672,512,1))

# Create and start camera thread
camera_thread = CameraThread(1, 'Thread-camera')
camera_thread.start()
print('Camera thread started')

exp_duration = 10
time.sleep(0.5)
start = time.time()
while time.time() - start < exp_duration:
    plt.imshow(image[:,:,0])
    plt.pause(0.5)
    c_p['recording'] = True

c_p['recording'] = False
time.sleep(0.1)

# NOTE: To quit the camera thread c_p['program_running'] should be set to false.
# If it is not then there is a risk that it will be hard to kill the program.
c_p['program_running'] = False
camera_thread.join()
