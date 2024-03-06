import os
import numpy as np
import glob
import matplotlib.pyplot as plt



# Define a function to read binary data from a file, decode it, and plot
def decode_and_plot(root_path, relative_path, file_path):
    # Read binary data from file
    try:
        with open(file_path, 'rb') as file:
            binary_data = file.read()

        # Decode the binary data into float32
        decoded_data = np.frombuffer(binary_data, dtype=np.float32).reshape(200,13)
        # print("shape of decoded_data: ", decoded_data.shape)
        if not np.all(np.isfinite(decoded_data)):
            print(f"Invalid features found in {file_path}")
        # Create a line plot
        plt.figure()
        plt.plot(decoded_data)
        plt.title('MFCC values')
        plt.xlabel('Index')
        plt.ylabel('Value')
        fig_path = f"{root_path}/{mfccfig_folder_name}/{relative_path}"
        fig_path = fig_path.replace(".mfcc", ".png")
        if not os.path.exists(os.path.dirname(fig_path)):
            os.makedirs(os.path.dirname(fig_path), exist_ok=True)
        plt.savefig(fig_path)
        plt.close()
        file.close()
    except FileNotFoundError:
        print(f"The file {file_path} was not found. Please check the file path and try again.")

mfccfig_folder_name = "mfccfig"
mfcc_folder_name = "mfcc"

root_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
raw_root_path = f"{root_path}/{mfcc_folder_name}"
mfccfig_root_path = f"{root_path}/{mfccfig_folder_name}"

for filepath in glob.glob(f"{root_path}/{mfcc_folder_name}/**/*.mfcc", recursive=True):
    print(filepath)
    relative_path = os.path.relpath(filepath, raw_root_path)
    decode_and_plot(root_path, relative_path, filepath)
