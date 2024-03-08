import os
import sys
import numpy as np
root_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
TRAIN_AUDIO_PATH = os.path.join(root_path, "mfcc", "train")
TEST_AUDIO_PATH = os.path.join(root_path, "mfcc", "test")
BREEDS = sorted(os.listdir(TRAIN_AUDIO_PATH))

f = open(f"{TEST_AUDIO_PATH}/Angry/angry_1.mfcc", "rb")
binary_data = f.read()
features = np.frombuffer(binary_data, dtype=np.float32)
print(f"Shape: {features.shape}")
print(f"Features: {features}")
np.save(f"{root_path}/validation.npy", features)