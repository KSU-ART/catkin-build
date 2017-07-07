import cv2
import json
import os, os.path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image
from scipy import ndimage, misc


print("OpenCV Version: {}".format(cv2.__version__))

def number_of_frames(vidcap):
    total = int(vidcap.get(cv2.CAP_PROP_FRAME_COUNT)) - 1
    print 'Number of frames:', total
    return total

def video_to_frames(vidcap, dirname, vid_frames, tot_frames, run):

    if run == True:
        success, image = vidcap.read()

        count = 0
        success = True
        while success:
            success, frame = vidcap.read()
            if success == 1:
                # cv2.imwrite(os.path.join(dirname, "%d.jpg" % count), frame)
                cv2.imwrite(os.path.join(dirname, str(count + tot_frames)+".jpg"), frame)
            count += 1

        if count - 1 == vid_frames:
            print('mp4 to jpg successful')
        elif count -1 < vid_frames:
            print('mp4 to jpg not succesful. Not enough pictures')
        elif count -1 > vid_frames:
            print('mp4 to jpg not succesful. Too many pictures')
        print count -1

def manipulate_images_from_folder(folder, w, h, frames, n=0):
    global action
    print n
    print frames
    for filename in range(n, frames):
        image = cv2.imread(os.path.join(folder,str(filename)+".jpg"))  # cv2.resize( , ( int(w*k), int(h*k) ))
        # shape = (np.shape(image))
        # Create window with freedom of dimensions 
        cv2.namedWindow("rumba_image",  cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("rumba_image", xy_coordinates)
        # keep looping until the mouse is pressed
        while True:
            # display the image and wait for a keypress
            cv2.imshow("rumba_image", image)
            key = cv2.waitKey(1) & 0xFF
            clone = image.copy()
            # if the mouse click is detected, break from the loop
            if action == True:
                action = False
                if coords[1][0] == coords[0][0] and coords[1][1] == coords[0][1]:
                    print ('Vector cannot have length of 0. Input again:')
                    continue
                else:
                    # vector
                    # (x2-x1,y2-y1)
                    LENGTH = np.sqrt( (np.power( (coords[1][0] - coords[0][0]), 2)) + (np.power( (coords[1][1] - coords[0][1]), 2)) )
                    # distance
                    # sqrt((x2-x1)^2 + (y2-y1)^2)
                    units[filename] = ( (coords[1][0] - coords[0][0])/ LENGTH , -(coords[1][1] - coords[0][1])/ LENGTH )
                    print 'image', filename, 'out of', frames - 1, ':', units[filename]

                    cv2.line(clone, (coords[0][0], coords[0][1]) , (coords[1][0],coords[1][1]) ,(255,0,0),1)
                    cv2.namedWindow("new_roomba_image",  cv2.WINDOW_NORMAL)
                    cv2.imshow("new_roomba_image", clone)
                    k = cv2.waitKey(0) & 0xFF
                    if k == 13:  # Enter key to stop
                        break
                    elif k == 8:  # normally -1 returned,so don't print it
                        continue
                    else:
                        print k # else print its value
                        break
        
        
def xy_coordinates(event, x, y, flags, param):
    global action
    action = False
    if event == cv2.EVENT_LBUTTONDOWN:
        # units['xy1'].append((x, y))
        coords[0]=(x,y)
    elif event == cv2.EVENT_LBUTTONUP:
        # units['xy2'].append((x, y))
        coords[1]=(x,y)
        action = True

    
units, action = ({}, False)
run = True

fname = 'units.txt'
units = {}
start = 0

if os.path.isfile(fname):
    with open(fname) as outfile:
        units = json.load(outfile)
        start = len(units)
        print 'The json file already contains ', start,  'entries'

coords = [(0,0),(0,0)]

dirname_read = os.getcwd() + "/videos_train"
dirname_write = os.getcwd() + "/img_train"
video_name = 'video12.mp4'

tot_pics = len([name for name in os.listdir(dirname_write) if os.path.isfile(os.path.join(dirname_write, name))])
print ('the current number of  pictures in the dataset is ', tot_pics)
# print (os.path.join(dirname_read,str(video_name)))

vidcap = cv2.VideoCapture(os.path.join(dirname_read,str(video_name)))

if vidcap.isOpened():
    print video_name, ' succesfully loaded'
    # get vcap property
    width = vidcap.get(cv2.CAP_PROP_FRAME_WIDTH)   # float
    height = vidcap.get(cv2.CAP_PROP_FRAME_HEIGHT) # float
    print 'width: ', width
    print 'height: ', height
    # (720, 960, 3)
else:
    print(video_name, ' not loaded')



# # # uncomment here to label # # #

vid_frames = number_of_frames(vidcap)
video_to_frames(vidcap, dirname_write, vid_frames, tot_pics, run)
vidcap.release()

tot_new_pics = len([name for name in os.listdir(dirname_write) if os.path.isfile(os.path.join(dirname_write, name))])
print 'the current number of pictures in the dataset after processing', video_name, 'is ', tot_new_pics

manipulate_images_from_folder(dirname_write, width, height, tot_new_pics, start)

with open('units.txt', 'w') as outfile:
    json.dump(units, outfile, indent = 4)