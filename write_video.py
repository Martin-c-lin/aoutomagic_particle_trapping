# import numpy as np
# import cv2
# # Define the codec and create VideoWriter object
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('output.avi',fourcc, 10.0, (640,480))
#
# for i in range(100):
#     frame = np.random.rand(640,480,3)
#     print(np.shape(frame))
#     # write the flipped frame
#     out.write(frame)
#
# # Release everything if job is finished
# out.release()
import numpy as np
from cv2 import VideoWriter, VideoWriter_fourcc

width = 1280
height = 720
FPS = 24
seconds = 10

fourcc = VideoWriter_fourcc(*'MP42')
video = VideoWriter('./output.avi', fourcc, float(FPS), (width, height))
tmp = np.zeros([height, width, 3],dtype=np.uint8)
print(np.shape(tmp))
print(np.shape(tmp[:,:,0]))
for _ in range(FPS*seconds):
    # frame = np.random.randint(0, 256,
    #                           (height, width, 3),
    #                           dtype=np.uint8)
    frame = np.random.randint(0, 256,(height, width, 1))
    tmp[:,:,0]= np.reshape(frame,[height, width])
    tmp[:,:,1]= np.reshape(frame,[height, width])
    tmp[:,:,2]= np.reshape(frame,[height, width])
    video.write(tmp)
print(np.shape(frame))
video.release()
