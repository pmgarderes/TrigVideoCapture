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

## Feldman Lab - installation cabling 

##
Vout ( adafruit MCP4 4725) ----> Arduino AO0 + Arduino ground
TDT start signal ---> Arduino Digital port 2 + Arduino ground
                ++ --->  Camera green cable ( line3)             
---

## 🧰 Requirements

- Python 3.10 (conda recommended)
- [FLIR Spinnaker SDK](https://www.flir.com/products/spinnaker-sdk/)
- PySpin (installed from SDK)      ---- >> SEE BELOW additional instruction
- # installed with requirements: 
  - OpenCV
  - numpy
  - tkinter (bundled with Python)
  - For DAQ version: `nidaqmx`
  - For Arduino version: standard `pyserial`


---

## 📦 Installation

We recommend creating a conda environment  :
in an anaconda command prompt ( !!! start promtp as administrator on windows !!!), type: 

```bash
conda create -n TrigVideoCapture python=3.10
conda activate TrigVideoCapture
cd PathToThetoolbox
# recommend installing spinnaker sdk at that moment 
pip install "YOURPATH\spinnaker_python-4.2.0.88-cp310-cp310-win_amd64.whl"
# test installation pf PySpin
python -c "import PySpin; print('PySpin OK')"
```


---


```bash 
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
