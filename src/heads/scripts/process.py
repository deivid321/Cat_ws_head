import numpy as np
from skimage import color, exposure, transform
from skimage import io
from numpy import genfromtxt
import os
import glob
from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.layers.convolutional import Conv2D, ZeroPadding2D, Convolution2D
from keras.layers.pooling import MaxPooling2D
from keras.optimizers import SGD, RMSprop
import keras as keras
from keras.optimizers import SGD

NUM_CLASSES = 3
IMG_SIZE = 100

sk = 0
def preprocess_img(img):
    # central square crop
    min_side = 150
    centre = img.shape[0] // 2, img.shape[1] // 2
    img = img[centre[0] - min_side // 2:centre[0] + min_side // 2,
          centre[1] - min_side // 2:centre[1] + min_side // 2,
          :]
    # rescale to standard size
    img = transform.resize(img, (IMG_SIZE, IMG_SIZE))
    global sk
    stri = 'tmp/work' + str(sk) + '.jpeg'
    #io.imsave(stri, img)
    sk = sk + 1
    return img


def cnn_model():
    model = Sequential()

    model.add(ZeroPadding2D((1, 1), input_shape=(IMG_SIZE, IMG_SIZE, 3)))
    model.add(Convolution2D(64, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1, 1)))
    model.add(Convolution2D(64, 3, 3, activation='relu'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2)))

    model.add(ZeroPadding2D((1, 1)))
    model.add(Convolution2D(128, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1, 1)))
    model.add(Convolution2D(128, 3, 3, activation='relu'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2)))

    model.add(ZeroPadding2D((1, 1)))
    model.add(Convolution2D(256, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1, 1)))
    model.add(Convolution2D(256, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1, 1)))
    model.add(Convolution2D(256, 3, 3, activation='relu'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2)))

    model.add(ZeroPadding2D((1, 1)))
    model.add(Convolution2D(512, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1, 1)))
    model.add(Convolution2D(512, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1, 1)))
    model.add(Convolution2D(512, 3, 3, activation='relu'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2)))

    model.add(ZeroPadding2D((1, 1)))
    model.add(Convolution2D(512, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1, 1)))
    model.add(Convolution2D(512, 3, 3, activation='relu'))
    model.add(ZeroPadding2D((1, 1)))
    model.add(Convolution2D(512, 3, 3, activation='relu'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2)))

    model.add(Flatten())
    model.add(Dense(4096, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(4096, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(NUM_CLASSES, activation='linear', init="normal"))

    return model


root_dir = 'images/'
imgs = []
labels = []

all_img_paths = sorted(glob.glob(os.path.join(root_dir, '*.jpeg')))
paths = sorted(all_img_paths)
for img_path in paths:
    img = preprocess_img(io.imread(img_path))
    imgs.append(img)

X = np.array(imgs, dtype='float32')
# print X
Y = genfromtxt('yy.csv', delimiter=',')

model = cnn_model()
sgd = SGD(lr=0.1, decay=1e-6, momentum=0.9, nesterov=True)
# let's train the model using RMSprop
lr = 0.001
rms = RMSprop(lr=lr, rho=0.9)
model.compile(loss='mean_squared_error',
              optimizer=rms,
              metrics=['accuracy'])
#model.compile(optimizer=sgd, loss='categorical_crossentropy', metrics=['accuracy'])  # for vgg model
batch_size = 8
epochs = 30

model.summary()
# embed()

model.fit(X, Y,
          batch_size=batch_size,
          epochs=epochs,
          verbose=1,
          validation_split=0.2
          )

# Load test dataset
X_test = []
Y_test = genfromtxt('tarkim_test/yy.csv', delimiter=',')
root_dir = 'tarkim_test/images/'

imgs = []
all_img_paths = sorted(glob.glob(os.path.join(root_dir, '*.jpeg')))
paths = sorted(all_img_paths)
for img_path in paths:
    imgs.append(preprocess_img(io.imread(img_path)))

X_test = np.array(imgs, dtype='float32')
Y_test = np.array(Y_test)

# predict and evaluate
Y_pred = model.predict(X_test)
acc = np.sum(Y_pred == Y_test) / np.size(Y_pred)
print("Test accuracy = {}".format(acc))
