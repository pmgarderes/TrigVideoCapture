# 🧪 TrigVideoCapture Toolbox

This is a Python-based toolbox for high-speed video acquisition with **FLIR/Blackfly USB3 cameras** using the **Spinnaker SDK (PySpin)**. It supports:
Camera live display and aquisition with trigger and metadata information(DAQmx or arduino) 
A Python-based toolbox using  **FLIR/Blackfly** camera, initially for neuroscience experiments

- ✅ Real-time video display and recording
- ✅ Timestamp logging for each frame
- ✅ Dual analog input support:
  - `NI DAQmx` (e.g. PCIe-6351)
  - OR Arduino-based serial decoding of trial/stimulus signals
- ✅ Auto-generated filenames and safe overwrite checks
- ✅ CSV metadata logging
- ✅ Optional split-recording (WIP)

---

## 🚀 Usage Modes

| Mode              | Input Device | Notes                                                                 |
|-------------------|--------------|-----------------------------------------------------------------------|
| `main` (default)  | NI DAQ       | Reads analog line continuously (e.g. voltage-encoded trial number)    |
| `arduino-version` | Arduino      | Reads decoded trial/stim values via serial from Arduino Mega 2560     |

---

## 🧰 Requirements

- Python 3.8+ (conda recommended)
- [FLIR Spinnaker SDK](https://www.flir.com/products/spinnaker-sdk/)
- PySpin (installed from SDK)
- OpenCV
- numpy
- tkinter (bundled with Python)
- For DAQ version: `nidaqmx`
- For Arduino version: standard `pyserial`


---

## 📦 Installation

We recommend creating a conda environment:
in an anaconda command prompt, type: 

```bash
conda create -n TrigVideoCapture python=3.9
conda activate TrigVideoCapture
cd PathToThetoolbox
pip install -r requirements.txt
```

---


## 🚀  Use

Camera is fully parametrized in Spinnaker SDK GUI (frame rate, exposure time, etc... ) 
After activation of the environment in the anaconda command prompt :

```bash
conda activate TrigVideoCapture
cd PathToThetoolbox
```

run the file accorindng to your setup

```bash
python Display_Record_Arduino_AIO.py
```

or

```bash
python Display_Record_DAQmx_AIO.py
```

The software will prompt you to choose a folder and filename to save output. 
A window will open with 20Hz dipslay, and camera video is saved at the set framerate ( set in spinnaker SDK) an additional metadata file is daved along with the video with the same name 

to exit the program, simply press "q" with the video display window active 
emergency stop the program by pressing Ctrl-C in the command bash  
