import os
import serial
import struct
from progressbar import progressbar

FRAME_SIZE = 1024
FRAME_COUNT = 200
MFCC_COUNT_PER_FRAME = 13

# Function to find all .dat files recursively in the "../raw" directory
def find_dat_files(directory):
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(".dat"):
                yield os.path.join(root, file)

# Function to calculate the hop size for splitting the data
def calculate_hop_size(file_size):
    total_elements = file_size // 2  # 2 bytes per int16
    return (total_elements - FRAME_SIZE) // (FRAME_COUNT - 1)


# Function to send a slice of data and wait for a response
def send_slice_and_wait(ser, slice_data):
    ser.write(slice_data)
    response = ser.read(MFCC_COUNT_PER_FRAME * 4)  # 4 bytes per float32
    # return struct.unpack('f'*MFCC_COUNT_PER_FRAME, response)
    return response

# Main execution
if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=None)
    file_path = os.path.dirname(os.path.realpath(__file__))
    root_path = os.path.dirname(file_path)
    dat_files = list(find_dat_files(f'{root_path}/raw'))
    print(f"frame size: {FRAME_SIZE}")
    print(f"frame count: {FRAME_COUNT}")
    print(f"Found {len(dat_files)} .dat files. Processing...")
    file_count = 1
    for dat in dat_files:
        relative_path = os.path.relpath(dat, f"{root_path}/raw")
        mfcc_path = os.path.join(root_path, "mfcc", relative_path)
        mfcc_path = mfcc_path.replace(".dat", ".mfcc")

        print(f"Processing {file_count}/{len(dat_files)} | {dat}")
        file_size = os.path.getsize(dat)
        print(f"File size: {file_size} bytes")
        hop_size = calculate_hop_size(file_size)
        fin = open(dat, 'rb')
        if not os.path.exists(os.path.dirname(mfcc_path)):
            os.makedirs(os.path.dirname(mfcc_path), exist_ok=True)
        if not os.path.exists(mfcc_path):
            fout = open(mfcc_path, 'xb')
        else:
            fout = open(mfcc_path, 'wb')
        for i in progressbar(range(FRAME_COUNT)):
            start_pos = i * hop_size * 2  # 2 bytes per int16
            fin.seek(start_pos)
            slice_data = fin.read(FRAME_SIZE * 2)
            mfcc = send_slice_and_wait(ser, slice_data)
            fout.write(mfcc)
        fin.close()
        fout.close()
        file_count += 1
    ser.close()
