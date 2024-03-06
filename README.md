# cat sound classification training model

## folder description

* audio: wav format audio.
* raw: int16_t format raw data(mono channel).
* mfcc: MFCC data of each audio. calculated by MCU.
* script: python scripts for model training, data validation and so on.
* src: C/C++ source file for data extraction and MFCC calculation.

## how to use scripts

First install python environment.

```bash
pip3 install -r requirements.txt
```
* script/gen_validation.py: generate validation numpy data file for stm32cubeide cube-ai.
* script/mfcc.py: communicate with MCU through serial port to get MFCCs from MCU.
* script/validate_raw.py: visualize the int16 raw data, validation of raw data extraction.
* script/validate_mfcc.py: list all MFCC numbers and check invalid numbers.
* script/train.py: train nn model to classify different types of cat sound.