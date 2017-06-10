import numpy as np
from skimage import color, exposure, transform
from skimage import io
from numpy import genfromtxt
import os
import glob
from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D
from keras.optimizers import SGD
from keras import backend as K
from keras.optimizers import SGD
from keras.callbacks import LearningRateScheduler, ModelCheckpoint
import pandas as pd
from keras.preprocessing.image import ImageDataGenerator, array_to_img, img_to_array, load_img

NUM_CLASSES = 43
IMG_SIZE = 400


def preprocess_img(img):
    # central square crop
    min_side = 200
    centre = img.shape[0] // 2, img.shape[1] // 2
    img = img[centre[0] - min_side // 2:centre[0] + min_side // 2,
          centre[1] - min_side // 2:centre[1] + min_side // 2,
          :]
    # rescale to standard size
    img = transform.resize(img, (IMG_SIZE, IMG_SIZE))
    # io.imsave('work.jpeg', img)
    # roll color axis to axis 0
    #img = np.rollaxis(img, -1)

    return img


def cnn_model():
    model = Sequential()

    model.add(Conv2D(32, (3, 3), padding='same',
                     input_shape=(IMG_SIZE, IMG_SIZE, 3),
                     activation='relu'))
    model.add(Conv2D(32, (3, 3), activation='relu'))
    #model.add(MaxPooling2D(pool_size=(2, 2)))
    #model.add(Dropout(0.2))

    #model.add(Conv2D(64, (3, 3), padding='same',
   #                  activation='relu'))
    #model.add(Conv2D(64, (3, 3), activation='relu'))
    #model.add(MaxPooling2D(pool_size=(2, 2)))
    #model.add(Dropout(0.2))

   # model.add(Conv2D(128, (3, 3), padding='same',
   #                  activation='relu'))
   # model.add(Conv2D(128, (3, 3), activation='relu'))
  #  model.add(MaxPooling2D(pool_size=(2, 2)))
    #model.add(Dropout(0.2))

    #model.add(Flatten())
    #model.add(Dense(512, activation='relu'))
    #model.add(Dropout(0.5))
    #model.add(Dense(NUM_CLASSES, activation='softmax'))
    return model


def lr_schedule(epoch):
    return lr * (0.1 ** int(epoch / 10))


root_dir = 'images/'
imgs = []
labels = []

all_img_paths = glob.glob(os.path.join(root_dir, '*.jpeg'))
np.random.shuffle(all_img_paths)
for img_path in all_img_paths:
    img = preprocess_img(io.imread(img_path))
    # label = get_class(img_path)
    imgs.append(img)
    # labels.append(label)

X = np.array(imgs, dtype='float32')
#print X
Y = genfromtxt('yy.csv', delimiter=',')
#print Y

model = cnn_model()

# let's train the model using SGD + momentum
lr = 0.01
sgd = SGD(lr=lr, decay=1e-6, momentum=0.9, nesterov=True)
model.compile(loss='categorical_crossentropy',
              optimizer=sgd,
              metrics=['accuracy'])

batch_size = 32
epochs = 30

model.fit(X, Y,
          batch_size=batch_size,
          epochs=epochs,
          verbose=1,
          validation_split=0.2
          )


# Load test dataset
X_test = []
y_test = genfromtxt('tarkim_test/yy.csv', delimiter=',')
i = 0
root_dir = 'tarkim_test/images/'
imgs = []

all_img_paths = glob.glob(os.path.join(root_dir, '*.jpeg'))
for img_path in all_img_paths:
    X_test.append(preprocess_img(io.imread(img_path)))

X_test = np.array(X_test)
y_test = np.array(y_test)

# predict and evaluate
y_pred = model.predict_classes(X_test)
acc = np.sum(y_pred == y_test) / np.size(y_pred)
print("Test accuracy = {}".format(acc))
# Make one hot targets
# Y = np.eye(NUM_CLASSES, dtype='uint8')[labels]
