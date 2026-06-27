# 🧪 TrigVideoCapture Toolbox

A Python-based toolbox for high-speed video acquisition with **FLIR / Blackfly USB3 cameras** using the **Spinnaker SDK / PySpin**.

Initially developed for neuroscience experiments, this toolbox supports live camera display, video acquisition, frame timestamp logging, and trial/stimulus metadata logging from either **NI DAQmx** or an **Arduino serial decoder**.

---

## ✅ Features

* Real-time video display
* Video recording from FLIR / Blackfly cameras
* Support for **1 or 2 cameras** with the flexible Arduino script
* Timestamp logging for each acquired frame
* Metadata logging to CSV
* Arduino-based serial decoding of trial/stimulus signals
* Optional NI DAQmx-based analog acquisition
* Auto-generated filenames
* Optional split-recording mode *(WIP)*

---

## 🚀 Usage Modes

| Script                                  | Input Device           | Camera Support            | Notes                                |
| --------------------------------------- | ---------------------- | ------------------------- | ------------------------------------ |
| `Display_Record_Arduino_1or2Cam_AIO.py` | Arduino serial decoder | 1 or 2 cameras            | Recommended flexible Arduino version |
| `Display_Record_Arduino_AIO.py`         | Arduino serial decoder | 2 cameras                 | Older fixed dual-camera version      |
| `Display_Record_DAQmx_AIO.py`           | NI DAQmx               | Depends on script version | Reads analog input continuously      |

---

## 🔌 Feldman Lab — Minimal Hardware Connections

### Camera acquisition

| Source / Device           | Destination          | Notes                                                |
| ------------------------- | -------------------- | ---------------------------------------------------- |
| **Camera USB 3.0**        | Acquisition computer | Plug directly into a USB 3.0 port for full bandwidth |
| **Second camera USB 3.0** | Acquisition computer | Only needed for two-camera setup                     |

For a **single-camera setup**, connect only one camera.

For a **two-camera setup**, connect both cameras before starting the Python script.

---

### Arduino trial/stimulus metadata

| Source / Device                    | Destination             | Notes                                        |
| ---------------------------------- | ----------------------- | -------------------------------------------- |
| **Vout** from Adafruit MCP4725 DAC | Arduino `A0`            | Analog packet encoding trial/stimulus values |
| **TDT Start Signal**               | Arduino digital pin `2` | Rising edge starts packet decoding           |
| **TDT / DAC GND**                  | Arduino `GND`           | Required shared ground                       |
| **Arduino USB**                    | Acquisition computer    | Used for serial communication with Python    |

The Arduino decodes the analog packet and sends lines such as:

```text
TRIAL:12,STIM:3
```

The Python script reads these values through the serial port and stores them in the frame metadata CSV.

---

### Optional LED illumination control

| Source / Device      | Destination                 | Notes                           |
| -------------------- | --------------------------- | ------------------------------- |
| Arduino `D9`         | LED 1 driver / MOSFET input | PWM output for LED 1            |
| Arduino `D10`        | LED 2 driver / MOSFET input | PWM output for LED 2            |
| Arduino `A2`         | Potentiometer 1 wiper       | Controls LED 1 intensity        |
| Arduino `A4`         | Potentiometer 2 wiper       | Controls LED 2 intensity        |
| Arduino `5V` / `GND` | Potentiometer side pins     | Potentiometer reference voltage |
| Arduino `GND`        | LED driver ground           | Required shared ground          |

The Arduino script also uses a door interlock:

| Source / Device | Destination           | Notes               |
| --------------- | --------------------- | ------------------- |
| Door switch     | Arduino `D8` to `GND` | Uses `INPUT_PULLUP` |

If no door switch is used during testing, connect:

```text
Arduino D8 → GND
```

Otherwise the Arduino will interpret the door as open and keep the LEDs off.

---

## 🧰 Requirements

* Python 3.10
* Conda recommended
* FLIR Spinnaker SDK
* PySpin, installed from the Spinnaker SDK Python wheel
* OpenCV
* NumPy
* tkinter, usually bundled with Python
* For Arduino version: `pyserial`
* For DAQ version: `nidaqmx`

---

## 📦 Installation

This has only been tested on Windows machines, but should in principle run on other OS

You first need to install Spinnaker (https://www.teledynevisionsolutions.com/support/support-center/software-firmware-downloads/iis/spinnaker-sdk-download/spinnaker-sdk--download-files/)
Currently( June 2026) the download includes an .exe to install Spinnaker and some compressed wheel files  that allows to install SDK for python (e.g. spinnaker_python-3.2.0.65-cp310-cp310-win_amd64.whl)

During the installation of Spinnaker, installing only the spinView program is sufficicent, it will allow you to choose the camera parameters (exposure, frame rate etc...) 


Create and activate a conda environment. ( I use miniconda) 

On Windows, open an **Anaconda Prompt as administrator**:
and type all of the following: 

```bash
conda create -n TrigVideoCapture python=3.10 -y
conda activate TrigVideoCapture
```

```bash
python -m pip install --upgrade pip setuptools wheel
python -m pip install numpy==1.23.5 pandas pyserial opencv-python
Install the Spinnaker Python wheel.
```


install PySpin, the FLIR SDK, from the wheeler . it needs to match your OS (win_amd64) , python in the conda (3.10), and normally version of Spinnaker just installed (3.2.0.65)0

Example: Change the path to wherever your wheel is.
```bash
pip install "C:\...\USER\download\spinnaker_python-3.2.0.65-cp310-cp310-win_amd64.whl"
```

Test PySpin installation:

```bash
python -c "import PySpin; print('PySpin file:', PySpin.__file__); print('Has System:', hasattr(PySpin, 'System')); print(PySpin.System)"
```

Then install the remaining Python requirements:

```bash
pip install -r requirements.txt
```

---

## ⚙️ Camera Configuration

Camera parameters such as:

* frame rate
* exposure time
* gain
* image size / ROI
* trigger mode, if used

should be configured in the **Spinnaker SDK GUI** before running the acquisition script.

The Python script records video at the camera framerate configured in Spinnaker.

---

## 🚀 Running the Toolbox

Activate the environment and go in the folder where this github code lives: 

```bash
conda activate TrigVideoCapture
cd PathToTheToolbox
```

### Recommended Arduino version: 1 or 2 cameras

```bash
python Display_Record_Arduino_1or2Cam_AIO.py
```

This script automatically detects the number of connected cameras.

* If one camera is connected, it records one video.
* If two or more cameras are connected, it records the first two cameras.
* Each camera is saved as a separate video file.

Example output files:

```text
MySession_cam0.avi
MySession_cam1.avi
MySession.csv
MySession_Analog.csv
```

For a single-camera setup, only `MySession_cam0.avi` is created.

---

### Older Arduino dual-camera version

```bash
python Display_Record_Arduino_AIO.py
```

This version expects two cameras.

---

### NI DAQmx version

```bash
python Display_Record_DAQmx_AIO.py
```

Use this version if metadata are acquired through an NI DAQ device instead of the Arduino serial decoder.

---

## 💾 Output Files

When the program starts, it will ask you to:

1. choose an output folder
2. enter a base filename

The toolbox then creates:

| File                   | Description                        |
| ---------------------- | ---------------------------------- |
| `BASE_NAME_cam0.avi`   | Video from camera 0                |
| `BASE_NAME_cam1.avi`   | Video from camera 1, if present    |
| `BASE_NAME.csv`        | Frame-by-frame metadata            |
| `BASE_NAME_Analog.csv` | Arduino-decoded trial/stimulus log |

The metadata CSV contains frame timestamps and trial/stimulus values decoded from Arduino serial communication.

---

## 🛑 Stopping Acquisition

To stop recording normally:

```text
Press q while the video display window is active
```

For emergency stop:

```text
Press Ctrl+C in the command prompt
```

---

## ⚠️ Notes

* Use a direct USB 3.0 connection for each camera when possible.
* Avoid USB hubs unless they are known to support the required bandwidth.
* Make sure the Arduino COM port and baudrate in the Python script match the Arduino.
* The Arduino script uses:

```cpp
Serial.begin(115200);
```

so the Python baudrate should usually be:

```python
SERIAL_BAUDRATE = 115200
```

* On Windows, check the Arduino COM port in Device Manager.
* The flexible 1-or-2-camera script includes safe overwrite checks for the actual output video names.
