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

```bash
conda create -n camrec python=3.9
conda activate camrec
pip install -r requirements.txt
