import numpy as np
import cv2
# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 10.0, (640,480))

for i in range(100):
    frame = np.random.rand(640,480,3)
    print(np.shape(frame))
    # write the flipped frame
    out.write(frame)

# Release everything if job is finished
out.release()
