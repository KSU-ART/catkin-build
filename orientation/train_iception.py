import os
import json
import numpy as np
import PIL
import cv2
import random
from keras.callbacks import ModelCheckpoint, EarlyStopping
from keras.optimizers import Adam
from orieNet import SqueezeNet
from keras.applications.imagenet_utils import _obtain_input_shape
from keras import backend as K
from keras.layers import Input, Convolution2D, MaxPooling2D, Activation, concatenate, Dropout, GlobalAveragePooling2D, warnings, Lambda
from keras.models import Model, load_model
from keras.engine.topology import get_source_inputs
from keras.utils import get_file
from keras.utils import layer_utils
from keras.preprocessing import image
from keras.applications.inception_v3 import InceptionV3

def load_X_train_data(n_test, shuffled_indices, x_tot, batch_size, b, remaining):
    # Chose path to the folder containing the training data in .jpg format:
    # train_data_path = '/media/mat/Seagate\ Backup\ Plus\ Drive/img_train'

    print 'Loading Train Data...'
    X_train, X_test = load_jpg_images(train_data_path, shuffled_indices, x_tot, n_test, batch_size, b, remaining)

    print 'Shape of train images array: ', X_train.shape
    print 'Shape of test images array: ', X_test.shape
    return X_train, X_test

def load_Y_data(shuffled_indices, num_x, n_test, batch_size, b, remaining):
    fname = 'units.txt'
    # json_path = '/media/mat/Seagate Backup Plus Drive'
    with open(fname) as outfile:
        units = json.load(outfile)
        entries = len(units)
        print 'The json file contains ', entries, 'entries'
    # print units['0'][0]
    if num_x != entries:
        if num_x > entries:
            print 'Error: There are ', entries, ' entries in the json file, but only ', num_x, ' pictures in the dataset'
            return
        if num_x < entries:
            print 'Error: There are ', num_x,' pictures in the dataset, but only ', entries, ' entries in the json file'
            return
    if num_x == entries:
        if remaining == False:
            Y_train = np.empty((batch_size-n_test,2))
            Y_test = np.empty((n_test,2))

            p_count = 0
            for i in range(b*batch_size, (b*batch_size) + batch_size - n_test):
                entry = units[str(shuffled_indices[i])]
                if entry is not None:
                    Y_train[p_count]=entry
                    p_count = p_count + 1
                else:
                    print 'while loading train labels, found a None at position ', shuffled_indices[i], 'in the units.txt file'
                    return
            p_count = 0
            for i in range(b*batch_size + batch_size - n_test,b*batch_size + batch_size):
                entry = units[str(shuffled_indices[i])]
                if entry is not None:
                    Y_test[p_count]=entry
                    p_count = p_count + 1
                else:
                    print 'while loading test labels, found a None at position ', shuffled_indices[i], 'in the units.txt file'
                    return
            print'Shape of train labels: ', Y_train.shape
            print'Shape of test labels: ', Y_test.shape

        else:
            Y_train = np.empty((batch_size-n_test,2))
            Y_test = np.empty((n_test,2))

            p_count = 0
            for i in range(x_tot - batch_size, x_tot - n_test):
                entry = units[str(shuffled_indices[i])]
                if entry is not None:
                    Y_train[p_count]=entry
                    p_count = p_count + 1
                else:
                    print 'while loading train labels, found a None at position ', shuffled_indices[i], 'in the units.txt file'
                    return
            p_count = 0
            for i in range(x_tot - n_test, x_tot):
                entry = units[str(shuffled_indices[i])]
                if entry is not None:
                    Y_test[p_count]=entry
                    p_count = p_count + 1
                else:
                    print 'while loading test labels, found a None at position ', shuffled_indices[i], 'in the units.txt file'
                    return
            print'Shape of train labels: ', Y_train.shape
            print'Shape of test labels: ', Y_test.shape
    return Y_train, Y_test

def load_jpg_images(folder, shuffled_indices, x_tot, n_test, batch_size, b, remaining):
    X_train = np.empty((batch_size-n_test,480, 640, 3))
    # X_train = np.empty((480, 640, 0))
    X_test = np.empty((n_test,480, 640, 3))

    if remaining == False:
        p_count = 0
        for i in range(b*batch_size, (b*batch_size) + batch_size - n_test):
            img = cv2.imread(os.path.join(folder, str(shuffled_indices[i])+".jpg"))
            # img = np.array(PIL.Image.open(os.path.join(folder, str(shuffled_indices[i])+".jpg")))/255
            if img is not None:
                X_train[p_count] = img
                p_count = p_count+1
            else:
                print 'while loading test images, found a None'
                return
        p_count = 0
        for i in range(b*batch_size + batch_size - n_test,b*batch_size + batch_size):
            img = cv2.imread(os.path.join(folder, str(shuffled_indices[i])+".jpg"))
            if img is not None:
                X_test[p_count] = img
                p_count = p_count+1
            else:
                print 'while loading test images, found a None'
                return
    else:
        p_count = 0
        for i in range(x_tot - batch_size, x_tot - n_test):
            img = cv2.imread(os.path.join(folder, str(shuffled_indices[i])+".jpg"))
            if img is not None:
                X_train[p_count] = img
                p_count = p_count+1
            else:
                print 'while loading test images, found a None'
                return
        p_count = 0
        for i in range(x_tot - n_test, x_tot):
            img = cv2.imread(os.path.join(folder, str(shuffled_indices[i])+".jpg"))
            if img is not None:
                X_test[p_count] = img
                p_count = p_count+1
            else:
                print 'while loading test images, found a None'
                return
    
    return X_train, X_test
def train(X_train, Y_train, X_test, Y_test, b):
    nb_epoch = 6
    # create the base pre-trained model
    base_model = InceptionV3(weights=None, include_top=False)
    x = base_model.output
    x = Convolution2D(2, (1, 1), padding='valid', name='conv2')(x)
    x = GlobalAveragePooling2D()(x)
    predictions = Lambda(lambda x: K.l2_normalize(x, axis=1), name='unit_vector_normalization')(x)


    # this is the model we will train
    model = Model(inputs=base_model.input, outputs=predictions)
    model.load_weights('inception_weights.h5')
    # first: train only the top layers (which were randomly initialized)
    # i.e. freeze all convolutional InceptionV3 layers
    for layer in base_model.layers:
        layer.trainable = False

    # compile the model (should be done *after* setting layers to non-trainable)
    model.compile(loss='mean_absolute_error', optimizer='nadam')

    # train the model on the new data for a few epochs
    model.fit(X_train, Y_train, batch_size=40, epochs=nb_epoch, verbose=1,validation_data=(X_test, Y_test))
    model.save('inception_weights.h5')

    # at this point, the top layers are well trained and we can start fine-tuning
    # convolutional layers from inception V3. We will freeze the bottom N layers
    # and train the remaining top layers.

    # let's visualize layer names and layer indices to see how many layers
    # we should freeze:
    for i, layer in enumerate(base_model.layers):
        print(i, layer.name)

    # we chose to train the top 2 inception blocks, i.e. we will freeze
    # the first 249 layers and unfreeze the rest:
    for layer in model.layers[:249]:
        layer.trainable = False
    for layer in model.layers[249:]:
        layer.trainable = True

    # we need to recompile the model for these modifications to take effect
    # we use SGD with a low learning rate
    from keras.optimizers import SGD
    model.compile(optimizer=SGD(lr=0.0001, momentum=0.9), loss='mean_absolute_error')

    # we train our model again (this time fine-tuning the top 2 inception blocks
    # alongside the top Dense layers

    score = model.fit(X_train, Y_train, batch_size=40, epochs=nb_epoch, verbose=1,validation_data=(X_test, Y_test))
    # model.fit_generator(X_train, Y_train, epochs=nb_epoch, verbose=1,validation_data=(X_test, Y_test))
    model.save('inception_weights.h5')
    return score

# To load:
percentage_test = 0.1
train_data_path = os.path.join(os.getcwd(), 'img_train')
x_tot = len([name for name in os.listdir(train_data_path) if os.path.isfile(os.path.join(train_data_path, name))])
print 'The dataset contains ', x_tot, 'images'

shuffled_indices = np.arange(x_tot)
np.random.shuffle(shuffled_indices)

for r in range(10):
    print r+1,': ',shuffled_indices[r]


b = 0
batch_size = 400
print 'Mini-batch size: ', batch_size
n_test = int(percentage_test*batch_size)
print 'validation data size: ', n_test

while (b+1)*batch_size < x_tot:
    print 'b = ', b
    X_train, X_test = load_X_train_data(n_test, shuffled_indices, x_tot, batch_size, b, remaining=False)
    print 'First 10 values in array of shuffled indeces: ', shuffled_indices[0:9]

    Y_train, Y_test = load_Y_data(shuffled_indices, x_tot, n_test, batch_size, b, remaining=False)

    # To train:
    score = train(X_train, Y_train, X_test, Y_test, b)
    b = b+1
b=b-1
n_test = int(percentage_test*(x_tot- (b)*batch_size))
print 'Last phase, b+1 = ', b+1
print 'Mini-batch size: ', x_tot - (b)*batch_size
print 'validation data size: ', n_test
X_train, X_test = load_X_train_data(n_test, shuffled_indices, x_tot, (x_tot - (b)*batch_size), b, remaining=True)

Y_train, Y_test = load_Y_data(shuffled_indices, x_tot, n_test, (x_tot - (b)*batch_size), b, remaining=True)

# To train:

score = train(X_train, Y_train, X_test, Y_test, b+1)
print type(score)
print score