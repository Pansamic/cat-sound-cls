import os
import numpy as np
from sklearn.preprocessing import LabelEncoder, StandardScaler
from keras.models import Sequential, save_model, load_model
from keras.layers import Dense
import librosa
import tensorflow as tf

# Check if a GPU is available
if tf.test.gpu_device_name():
    print('GPU found')
else:
    print("No GPU found")


# Set the path to the dataset
root_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
TRAIN_AUDIO_PATH = os.path.join(root_path, "mfcc", "train")
TEST_AUDIO_PATH = os.path.join(root_path, "mfcc", "test")
MODEL_FILE = "cat_sound_classification_model.h5"

# Define the number of classes and the size of the input feature vector
NUM_CLASSES = len(os.listdir(TRAIN_AUDIO_PATH))
FRAME_SIZE = 1024
FRAME_COUNT = 200
MFCC_COUNT_PER_FRAME = 13
INPUT_SIZE = MFCC_COUNT_PER_FRAME * FRAME_COUNT

# Define the list of cat breeds
BREEDS = sorted(os.listdir(TRAIN_AUDIO_PATH))

def load_data(audio_path):
    files = []
    labels = []
    for breed in BREEDS:
        breed_folder_path = os.path.join(audio_path, breed)
        for filename in os.listdir(breed_folder_path):
            file_path = os.path.join(breed_folder_path, filename)
            fin = open(file_path, "rb")
            binary_data = fin.read()
            features = np.frombuffer(binary_data, dtype=np.float32)
            fin.close()
            if features is not None and np.all(np.isfinite(features)):
                files.append(features)
                labels.append(breed)
    if not files:
        print("No valid audio files found in the dataset.")
        exit()
    return np.array(files), np.array(labels)


# Load the training and testing data
train_files, train_labels = load_data(TRAIN_AUDIO_PATH)
test_files, test_labels = load_data(TEST_AUDIO_PATH)

# Encode the labels as integers
label_encoder = LabelEncoder()
train_labels_encoded = label_encoder.fit_transform(train_labels)
test_labels_encoded = label_encoder.transform(test_labels)

# Scale the input features
scaler = StandardScaler()
# Check if `train_files` contains any infinite or NaN values
if not np.all(np.isfinite(train_files)):
    # Replace infinite values with a finite number, such as the maximum finite value in the data
    max_finite_value = 32768.0
    train_files[~np.isfinite(train_files)] = max_finite_value
train_files_scaled = scaler.fit_transform(train_files.reshape(-1, INPUT_SIZE))
test_files_scaled = scaler.transform(test_files.reshape(-1, INPUT_SIZE))

print(train_files_scaled.shape)
print(train_labels_encoded.shape)

# Create a Keras model with three dense layers and softmax activation
model = Sequential()
model.add(Dense(256, activation='relu', input_shape=(INPUT_SIZE,)))
model.add(Dense(128, activation='relu'))
model.add(Dense(NUM_CLASSES, activation='softmax'))

# Compile the model
model.compile(loss='sparse_categorical_crossentropy',
              optimizer='adam', metrics=['accuracy'])

# Train the model
model.fit(train_files_scaled, train_labels_encoded,
          epochs=100, batch_size=32, validation_split=0.2)

# Evaluate the model on the testing data
test_loss, test_accuracy = model.evaluate(test_files_scaled, test_labels_encoded)
print(f"Test accuracy: {test_accuracy}")

# Save the trained model to a file
save_model(model, MODEL_FILE)
print(f"Model saved to {MODEL_FILE}")

# Load the model from the file
loaded_model = load_model(MODEL_FILE)

# Use the loaded model to predict new samples
for breed in BREEDS:
    breed_folder_path = os.path.join(TEST_AUDIO_PATH, breed)
    for filename in os.listdir(breed_folder_path):
        file_path = os.path.join(breed_folder_path, filename)
        fin = open(file_path, "rb")
        binary_data = fin.read()
        features = np.frombuffer(binary_data, dtype=np.float32)
        fin.close()
        if features is not None:
            features = np.expand_dims(features, axis=0)
            prediction = loaded_model.predict(features)[0]
            predicted_breed = label_encoder.inverse_transform([np.argmax(prediction)])
            print(f"File {filename} is predicted to be a {predicted_breed[0]}")
