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
from keras.layers import Input, Convolution2D, MaxPooling2D, Activation, concatenate, Dropout, GlobalAveragePooling2D, warnings
from keras.models import Model, load_model
from keras.engine.topology import get_source_inputs
from keras.utils import get_file
from keras.utils import layer_utils
from keras.preprocessing import image

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
        pcount = 0
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
    #datagen = ImageDataGenerator(rotation_range=45,width_shift_range=0.2, height_shift_range=0.2)
    #datagen.fit(X_train)

    model = SqueezeNet()
    nb_epoch = 20

    model.load_weights('partly_trained.h5')
    model.compile(loss='mean_absolute_error', optimizer='nadam') #, metrics=['hinge'])

    #model.fit_generator(datagen.flow(X_train, Y_train, batch_size=32),
    #                samples_per_epoch=len(X_train), nb_epoch=nb_epoch)
    print 'Info: '
    print 'Shape of test image array: ', X_test.shape
    print 'Shape of test label array: ', Y_test.shape
    print 'first 10 values of test labels: '
    for r in range(10):
        print r+1,': ', Y_test[r]

    print 'Shape of train image array: ', X_train.shape
    print 'Shape of train label array: ', Y_train.shape
    print 'first 10 values of test labels: '
    for r in range(10):
        print r+1,': ', Y_train[r]

    model.fit(X_train, Y_train, batch_size=(b+1)*30, epochs=nb_epoch, verbose=1,validation_data=(X_test, Y_test))
    
    model.save('partly_trained.h5')

    score = model.evaluate(X_test, Y_test)
    print('___________________________________________')
    print('model'+str(b+1)+':')
    print 'shape of score: ', score.shape
    # print('Test loss:', score[0])
    # print('error:', str((1.-score[1])*100)+'%')

    return score

def evaluate(X_train, Y_train, X_test, Y_test, eval_all=False):

    ###evaluate models in the weights directory,
    ###defaults to only models with 'best'

    # for i in os.listdir('weights'):
    #     if '.h5' in i:
    #         if eval_all:
    #             evaluations.append(i)
    #         else:
    #             if 'Best' in i:
    #                 evaluations.append(i)
    # print(evaluations)
    model = SqueezeNet()
    # for run, i in enumerate(evaluations):
    model.load_weights(os.path.join('weights',i))
    model.compile(loss='categorical_crossentropy', optimizer='adam',
                metrics=['categorical_accuracy'])
    score = model.evaluate(X_test, Y_test,verbose=0)
    print('--------------------------------------')
    print('model'+str(run)+':')
    print('Test loss:', score[0])
    print('error:', str((1.-score[1])*100)+'%')


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
batch_size = 300
print 'Mini-batch size: ', batch_size
n_test = int(percentage_test*batch_size)
print 'validation data size: ', n_test

while (b+1)*batch_size < x_tot:
    print 'Phase ', b
    X_train, X_test = load_X_train_data(n_test, shuffled_indices, x_tot, batch_size, b, remaining=False)
    print 'First 10 values in array of shuffled indeces: ', shuffled_indices[0:9]

    Y_train, Y_test = load_Y_data(shuffled_indices, x_tot, n_test, batch_size, b, remaining=False)

    # To train:
    score = train(X_train, Y_train, X_test, Y_test, b)
    b = b+1
b=b-1
n_test = int(percentage*(x_tot- (b)*batch_size))
print 'Last phase ', b+1
print 'Mini-batch size: ', x_tot - (b)*batch_size
print 'validation data size: ', n_test
X_train, X_test = load_X_train_data(n_test, shuffled_indices, x_tot, (x_tot - (b)*batch_size), b, remaining=True)

Y_train, Y_test = load_Y_data(shuffled_indices, x_tot, n_test, (x_tot - (b)*batch_size), b, remaining=True)

# To train:

score = train(X_train, Y_train, X_test, Y_test, b+1)
print type(score)
print score
# evaluate_ensemble(True)
# evaluate(X_train, Y_train, X_test, Y_test, eval_all=False)
# test_model()



# def evaluate_ensemble(Best=True):
    
#     ###loads and evaluates an ensemle from the models in the model folder.

#     model_dirs = []
#     for i in os.listdir('weights'):
#         if '.h5' in i:
#             if not Best:
#                 model_dirs.append(i)
#             else:
#                 if 'Best' in i:
#                     model_dirs.append(i)

#     preds = []
#     model = SqueezeNet()
#     for mfile in model_dirs:
#         print(os.path.join('weights',mfile))
#         model.load_weights(os.path.join('weights',mfile))
#         yPreds = model.predict(X_test, batch_size=128, verbose=0)
#         preds.append(yPreds)

#     weighted_predictions = np.zeros((X_test.shape[0], 10), dtype='float64')
#     weight = 1./len(preds)
#     for prediction in preds:
#         weighted_predictions += weight * prediction
#     y_pred =weighted_predictions

#     print(type(Y_test))
#     print(type(y_pred))
#     Y_test = tf.convert_to_tensor(Y_test)
#     y_pred = tf.convert_to_tensor(y_pred)
#     print(type(Y_test))
#     print(type(y_pred))

#     loss = metrics.categorical_crossentropy(Y_test, y_pred)
#     acc = metrics.categorical_accuracy(Y_test, y_pred)
#     sess = tf.Session()
#     print('--------------------------------------')
#     print('ensemble')
#     print('Test loss:', loss.eval(session=sess))
#     print('error:', str((1.-acc.eval(session=sess))*100)+'%')
#     print('--------------------------------------')