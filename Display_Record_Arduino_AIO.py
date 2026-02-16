import PySpin
import cv2
import numpy as np
import time
import csv
import os
import gc
import threading
import serial
import tkinter as tk
from tkinter import filedialog, simpledialog, messagebox
import os
# import nidaqmx
# from nidaqmx.constants import AcquisitionType


# =======================
# USER PARAMETERS
# =======================
DEFAULT_START_DIR = r"C:\Users\Behavior4\Documents\Camera_Recordings"
# OUTPUT_DIR = r"C:\Users\Behavior4\Documents\Camera_Recordings\PMtest\Day1"
# BASE_NAME = "PMtest2"
DISPLAY_FPS = 20.0  # Live display

ARDUINO_COMPORT = 6;
# not in use 
# DAQ_DEVICE = "Dev2"
# DAQ_CHANNEL = "ai1"
# DAQ_SAMPLING_RATE = 10000  # Hz

# =======================
# GUI FOLDER + NAME PROMPT
# =======================
root = tk.Tk()
root.withdraw()  # Hide the main tkinter window

# Folder selection
OUTPUT_DIR = filedialog.askdirectory(title="Select Output Folder", initialdir=DEFAULT_START_DIR)
if not OUTPUT_DIR:
    print("No folder selected. Aborting.")
    exit()

# Base name prompt
BASE_NAME = simpledialog.askstring("Base Name", "Enter a base name for files:")
if not BASE_NAME:
    print("No base name entered. Aborting.")
    exit()

# =======================
# FILE PATHS
# =======================
VIDEO_FILENAME = BASE_NAME + ".avi"
FRAME_CSV_FILENAME = BASE_NAME + ".csv"
ANALOG_CSV_FILENAME = BASE_NAME + "_Analog.csv"

video_path = os.path.join(OUTPUT_DIR, VIDEO_FILENAME)
frame_csv_path = os.path.join(OUTPUT_DIR, FRAME_CSV_FILENAME)
analog_csv_path = os.path.join(OUTPUT_DIR, ANALOG_CSV_FILENAME)

# =======================
# CHECK FOR EXISTING FILES
# =======================
existing_files = [f for f in [video_path, frame_csv_path, analog_csv_path] if os.path.exists(f)]
if existing_files:
    msg = "The following files already exist:\n\n" + "\n".join(existing_files) + "\n\nOverwrite?"
    if not messagebox.askyesno("Confirm Overwrite", msg):
        print("❌ Aborting to prevent overwrite.")
        exit()
        
        

# =======================
# HELPER FUNCTIONS
# =======================
def convert_to_numpy(image, processor):
    image_converted = processor.Convert(image, PySpin.PixelFormat_BGR8)
    return np.array(image_converted.GetNDArray())

def get_line3_state(cam):
    nodemap = cam.GetNodeMap()
    line_selector = PySpin.CEnumerationPtr(nodemap.GetNode("LineSelector"))
    line_selector.SetIntValue(line_selector.GetEntryByName("Line3").GetValue())
    line_status = PySpin.CBooleanPtr(nodemap.GetNode("LineStatus"))
    return line_status.GetValue()

# =======================
# ARDUINO  BACKGROUND THREAD
# =======================
class SerialDecoderReader(threading.Thread):
    def __init__(self, port='COM4', baudrate=115200):
        super().__init__()
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self.running = False
        self.lock = threading.Lock()
        self.timestamps = []
        self.trials = []
        self.stims = []

    def run(self):
        self.running = True
        print("Serial decoding thread started.")
        while self.running:
            if self.serial_port.in_waiting:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line.startswith("TRIAL:"):
                    try:
                        parts = line.split(',')
                        trial = int(parts[0].split(':')[1])
                        stim = int(parts[1].split(':')[1])
                        now = time.time()
                        with self.lock:
                            self.timestamps.append(now)
                            self.trials.append(trial)
                            self.stims.append(stim)
                        print(f"[{now:.3f}] Trial={trial}, Stim={stim}")
                    except Exception as e:
                        print(f"Error decoding serial line: {line} | {e}")

    def stop(self):
        self.running = False
        self.serial_port.close()
        
    def get_latest_values(self):
        with self.lock:
            if self.timestamps:
                return self.trials[-1], self.stims[-1]
            else:
                return None, None  # Or 0, 0 if preferred

    def get_data(self):
        with self.lock:
            return list(self.timestamps), list(self.trials), list(self.stims)
            
# =======================
# MAIN FUNCTION
# =======================
def main():
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    video_path = os.path.join(OUTPUT_DIR, VIDEO_FILENAME)
    frame_csv_path = os.path.join(OUTPUT_DIR, FRAME_CSV_FILENAME)
    analog_csv_path = os.path.join(OUTPUT_DIR, ANALOG_CSV_FILENAME)

  
    # -------------------------------
    # Start Serial Decoder Thread (Arduino)
    # -------------------------------
    try:
        cport = 'COm' + str(ARDUINO_COMPORT) 
        #serial_thread = SerialDecoderReader(port='COm6', baudrate=115200)  # ✅ adjust COM port if needed!
        serial_thread = SerialDecoderReader(port=cport, baudrate=115200)  # COM port in parameters 
        serial_thread.start()
        print("Serial decoder thread started.")
    except Exception as e:
        print(f"Failed to start serial thread: {e}")
        serial_thread = None

  
    # -------------------------------
    # Initialize Camera
    # -------------------------------
    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()
    if cam_list.GetSize() == 0:
        print("No camera detected.")
        cam_list.Clear()
        system.ReleaseInstance()
        return
    cam = cam_list[0]
    cam.Init()

    # Reduce latency: use NewestOnly
    s_node_map = cam.GetTLStreamNodeMap()
    handling_mode = PySpin.CEnumerationPtr(s_node_map.GetNode("StreamBufferHandlingMode"))
    newest_only = handling_mode.GetEntryByName("NewestOnly")
    handling_mode.SetIntValue(newest_only.GetValue())

    processor = PySpin.ImageProcessor()
    processor.SetColorProcessing(PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR)

    try:
        print("Starting video acquisition...")
        cam.BeginAcquisition()

        # Prepare CSV and Video
        csv_file = open(frame_csv_path, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["FrameID", "Timestamp_ns", "Line3_State", "Trial", "Stimulus"])

        video_writer = None
        frame_count = 0
        last_display = time.time()
        display_interval = 1.0 / DISPLAY_FPS
        dropped_frame_count = 0

        while True:
            try:
                image_result = cam.GetNextImage(1000)
                if not image_result.IsIncomplete():
                    frame = convert_to_numpy(image_result, processor)

                    if video_writer is None:
                        height, width = frame.shape[:2]
                        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                        video_writer = cv2.VideoWriter(video_path, fourcc, cam.AcquisitionFrameRate.GetValue(), (width, height))
                        print(f"Recording video to {video_path}")

                    video_writer.write(frame)

                    # Log digital input + timestamp
                    line3_state = get_line3_state(cam)
                    trial, stim = (serial_thread.get_latest_values() if serial_thread else (None, None))
                    csv_writer.writerow([frame_count, image_result.GetTimeStamp(), int(line3_state), trial, stim])
                    frame_count += 1

                    now = time.time()
                    if now - last_display >= display_interval:
                        cv2.imshow("Live Camera Feed", frame)
                        last_display = now
                else: 
                    print("⚠️ Dropped frame!")
                    dropped_frame_count += 1
                    continue
                    
                image_result.Release()
                del image_result

            except PySpin.SpinnakerException as ex:
                print(f"Acquisition error: {ex}")
                break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Stopping acquisition.")
                break
                
        # Force window close (sometimes needed in tkinter + OpenCV environments)
        cv2.destroyAllWindows()
        time.sleep(0.5)  # Give time for window to close
        cam.EndAcquisition()

    finally:
        try:
            cam.EndAcquisition()
        except:
            pass
        print(f"⚠️ Total dropped frames during session: {dropped_frame_count}")
        # Stop serial thread and save its data
        if serial_thread:
            serial_thread.stop()
            serial_thread.join()
            print("Serial decoding stopped.")

        serial_timestamps, serial_trials, serial_stims = serial_thread.get_data()
        with open(analog_csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp_s", "Trial", "Stimulus"])
            for t, tr, st in zip(serial_timestamps, serial_trials, serial_stims):
                writer.writerow([t, tr, st])

        if video_writer:
            video_writer.release()
        csv_file.close()



        # Camera shutdown
        cam.DeInit()
        del cam
        cam_list.Clear()
        del cam_list
        system.ReleaseInstance()
        del system

        cv2.destroyAllWindows()
        gc.collect()

        print(f"Recording saved: {video_path}")
        print(f"Frame log saved: {frame_csv_path}")
        # print(f"Analog log saved: {analog_csv_path}")
        
                # === Post-check: compare recorded video frame count to log ===

        cap = cv2.VideoCapture(video_path)
        video_frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()

        csv_frame_count = frame_count  # Already tracked during recording

        if video_frame_count != csv_frame_count:
            print(f"⚠️ MISMATCH: Video has {video_frame_count} frames, CSV logged {csv_frame_count}.")
        else:
            print(f"✅ Frame counts match: {video_frame_count} frames.")

if __name__ == "__main__":
    main()
