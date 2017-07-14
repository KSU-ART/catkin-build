import os
import json
import numpy as np
import PIL
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

def load_X_train_data():
    # Chose path to the folder containing the training data in .jpg format:
    # train_data_path = '/media/mat/Seagate\ Backup\ Plus\ Drive/img_train'

    train_data_path = os.path.join(os.getcwd(), 'img_train')
    x_tot = len([name for name in os.listdir(train_data_path) if os.path.isfile(os.path.join(train_data_path, name))])
    print ('The dataset contains ', x_tot, 'images')

    print('Loading Train Data: ')
    X_train, X_name_of_each_train = load_jpg_images(train_data_path, x_tot)
    print(len(X_train))
    X_train = np.array(X_train, dtype=np.float16)

    print('Shape of train images array: ', X_train.shape)
    return X_train, X_name_of_each_train, x_tot

def load_Y_data(num_x):
    fname = 'units.txt'
    json_path = '/media/mat/Seagate Backup Plus Drive'
    with open(fname) as outfile:
        units = json.load(outfile)
        entries = len(units)
        print ('The json file contains ', entries, 'entries')
    # print units['0'][0]
    if num_x != entries:
        print ('The number of entries in the json file do not match the number of pictures in the dataset')
        return
    if num_x == entries:
        vec = np.zeros([entries, 2])
        for i in range(entries):
            vec [i] = units[str(i)]
    return vec

def load_jpg_images(folder, n):
    images = []
    filenames = []
    for i in range(n):
        img = np.array(PIL.Image.open(os.path.join(folder, str(i)+".jpg")))/255
        if img is not None:
            images.append(img)
            filenames.append(i)
    return images, filenames

def train(X_train, Y_train, X_test, Y_test):
    #datagen = ImageDataGenerator(rotation_range=45,width_shift_range=0.2, height_shift_range=0.2)
    #datagen.fit(X_train)

    # print y
    # print y[0]
    # print y[0][0]

    model = SqueezeNet()
    nb_epoch = 5

    model.load_weights('partly_trained.h5')
    model.compile(loss='mean_squared_error', optimizer='nadam', metrics=['mean_squared_error'])

    #model.fit_generator(datagen.flow(X_train, Y_train, batch_size=32),
    #                samples_per_epoch=len(X_train), nb_epoch=nb_epoch)
    model.fit(X_train, Y_train, batch_size=2, epochs=nb_epoch, verbose=1,validation_data=(X_test, Y_test))
    
    model.save('partly_trained.h5')

    score = model.evaluate(X_test, Y_test,verbose=0)
    # print('___________________________________________')
    # print('model'+str(run)+':')
    # print('Test loss:', score[0])
    # print('error:', str((1.-score[1])*100)+'%')

    return score

def evaluate_ensemble(Best=True):
    
    ###loads and evaluates an ensemle from the models in the model folder.

    model_dirs = []
    for i in os.listdir('weights'):
        if '.h5' in i:
            if not Best:
                model_dirs.append(i)
            else:
                if 'Best' in i:
                    model_dirs.append(i)

    preds = []
    model = SqueezeNet()
    for mfile in model_dirs:
        print(os.path.join('weights',mfile))
        model.load_weights(os.path.join('weights',mfile))
        yPreds = model.predict(X_test, batch_size=128, verbose=0)
        preds.append(yPreds)

    weighted_predictions = np.zeros((X_test.shape[0], 10), dtype='float64')
    weight = 1./len(preds)
    for prediction in preds:
        weighted_predictions += weight * prediction
    y_pred =weighted_predictions

    print(type(Y_test))
    print(type(y_pred))
    Y_test = tf.convert_to_tensor(Y_test)
    y_pred = tf.convert_to_tensor(y_pred)
    print(type(Y_test))
    print(type(y_pred))

    loss = metrics.categorical_crossentropy(Y_test, y_pred)
    acc = metrics.categorical_accuracy(Y_test, y_pred)
    sess = tf.Session()
    print('--------------------------------------')
    print('ensemble')
    print('Test loss:', loss.eval(session=sess))
    print('error:', str((1.-acc.eval(session=sess))*100)+'%')
    print('--------------------------------------')

def evaluate(X_train, Y_train, X_test, Y_test, eval_all=False):

    ###evaluate models in the weights directory,
    ###defaults to only models with 'best'

    for i in os.listdir('weights'):
        if '.h5' in i:
            if eval_all:
                evaluations.append(i)
            else:
                if 'Best' in i:
                    evaluations.append(i)
    print(evaluations)
    model = SqueezeNet()
    for run, i in enumerate(evaluations):
        model.load_weights(os.path.join('weights',i))
        model.compile(loss='categorical_crossentropy', optimizer='adam',
                    metrics=['categorical_accuracy'])
        score = model.evaluate(X_test, Y_test,verbose=0)
        print('--------------------------------------')
        print('model'+str(run)+':')
        print('Test loss:', score[0])
        print('error:', str((1.-score[1])*100)+'%')


# To load:
n_test = 200

x, x_name, num_x = load_X_train_data()
y = load_Y_data(num_x)

X_train = np.empty((num_x-n_test,480, 640, 3))
Y_train = np.empty((num_x-n_test,2))
X_test = np.empty((n_test,480, 640, 3))
Y_test = np.empty((n_test,2))

shuffled_indeces = np.random.shuffle(x_name)

# training_idx, test_idx = shuffled_indeces[:num_x-n_test], shuffled_indeces[n_test:]
# X_train, X_test = x[training_idx,:], x[test_idx,:]
# Y_train, Y_test = y[training_idx,:], y[test_idx,:]

for i in range(num_x-n_test):
    X_train[i] = x[x_name[i]]
    Y_train[i] = y[x_name[i]]
for i in range(num_x-n_test,n_test):
    X_test[i] = x[x_name[i]]
    Y_test[i] = y[x_name[i]]

print('Shape of train images array: ', X_train.shape)
print('Shape of train labels: ', Y_train.shape)
print('Shape of test images array: ', X_test.shape)
print('Shape of test labels: ', Y_test.shape)
del x
del y

# # # #

# c = list(zip(x, y, x_name))
# random.shuffle(c)
# x_data, y_data, indeces = zip(*c)
# X_train = np.array(x_data[:1166])
# Y_train = np.array(y_data[:1166])
# X_test = np.array(x_data[1166:])
# Y_test = np.array(y_data[1166:])


# To train:
train(X_train, Y_train, X_test, Y_test)

# evaluate_ensemble(True)
evaluate(X_train, Y_train, X_test, Y_test, eval_all=False)
# test_model()