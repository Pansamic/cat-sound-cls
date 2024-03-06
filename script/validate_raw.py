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

        # Decode the binary data into int16_t
        decoded_data = np.frombuffer(binary_data, dtype=np.int16)
        # Create a line plot
        plt.figure()
        plt.plot(decoded_data)
        plt.title('audio wave')
        plt.xlabel('Index')
        plt.ylabel('Value')
        fig_path = f"{root_path}/{wavefig_folder_name}/{relative_path}"
        fig_path = fig_path.replace(".dat", ".png")
        if not os.path.exists(os.path.dirname(fig_path)):
            os.makedirs(os.path.dirname(fig_path), exist_ok=True)
        plt.savefig(fig_path)
        plt.close()
        file.close()
    except FileNotFoundError:
        print(f"The file {file_path} was not found. Please check the file path and try again.")

wavefig_folder_name = "wavefig"
raw_folder_name = "raw"

root_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
raw_root_path = f"{root_path}/{raw_folder_name}"
wavefig_root_path = f"{root_path}/{wavefig_folder_name}"

for filepath in glob.glob(f"{root_path}/{raw_folder_name}/**/*.dat", recursive=True):
    print(filepath)
    relative_path = os.path.relpath(filepath, raw_root_path)
    decode_and_plot(root_path, relative_path, filepath)
