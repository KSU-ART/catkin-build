import os
import cv2
import numpy as np
from orieNet import SqueezeNet
from keras.applications.imagenet_utils import preprocess_input, decode_predictions
from keras.preprocessing import image

model = SqueezeNet()

n_pic = 74
folder = '/img_train'
directory = os.getcwd() + (os.path.join(folder,str(n_pic)+".jpg"))
# image = cv2.imread(os.path.join(folder,str(filename)+".jpg"))
###
cap = cv2.VideoCapture(1)

while(True):
    # Capture frame-by-frame
    ret, image = cap.read()
    # x = image.img_to_array(img)
    x = np.expand_dims(image, axis=0)
    # x = preprocess_input(x)
    
    preds = model.predict(x)

    X = 320 + int(100*(preds[0][0]))
    Y = 240 + int(100*(preds[0][1]))
    cv2.namedWindow('frame',  cv2.WINDOW_NORMAL)
    cv2.line(image, (320, 240), (X, Y), (255,0,0), 1)
    cv2.imshow('frame',image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

'''
img = image.load_img(directory) # target_size=(227, 227))
print 'directory: ', 
x = image.img_to_array(img)
x = np.expand_dims(x, axis=0)
x = preprocess_input(x)

preds = model.predict(x)
print 'Direction vector:', preds[0][1]

cv2.namedWindow("Predicted: ",  cv2.WINDOW_NORMAL)
image = cv2.imread(os.getcwd() + (os.path.join(folder,str(n_pic)+".jpg")) )
key = cv2.waitKey(1) & 0xFF
X = 320 + int(10*(preds[0][0]))
Y = 240 + int(10*(preds[0][1]))
print 'X: ', X
print 'Y: ', Y
cv2.line(image, (320, 240), (X, Y), (255,0,0), 1)

cv2.imshow("Predicted: ",  image)
cv2.waitKey(0) & 0xFF7
'''