import PySpin
import cv2
import numpy as np
import time
import csv
import os
import gc
import threading
#import serial
import tkinter as tk
from tkinter import filedialog, simpledialog, messagebox
import os
import nidaqmx
from nidaqmx.constants import AcquisitionType


# =======================
# USER PARAMETERS
# =======================
DEFAULT_START_DIR = r"C:\Users\Behavior4\Documents\Camera_Recordings"
# OUTPUT_DIR = r"C:\Users\Behavior4\Documents\Camera_Recordings\PMtest\Day1"
# BASE_NAME = "PMtest2"
DISPLAY_FPS = 20.0  # Live display
DAQ_DEVICE = "Dev2"
DAQ_CHANNEL = "ai1"
DAQ_SAMPLING_RATE = 10000  # Hz

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
# DAQ BACKGROUND THREAD
# =======================
class AnalogReader(threading.Thread):
    def __init__(self, device="Dev2", channel="ai1", rate=10000):
        super().__init__()
        self.device = device
        self.channel = channel
        self.rate = rate
        self.running = False
        self.lock = threading.Lock()
        self.timestamps = []
        self.values = []

    def run(self):
        try:
            with nidaqmx.Task() as task:
                task.ai_channels.add_ai_voltage_chan(f"{self.device}/{self.channel}")
                task.timing.cfg_samp_clk_timing(rate=self.rate,
                                                sample_mode=AcquisitionType.CONTINUOUS)

                self.running = True
                print("DAQ task started (basic read).")
                while self.running:
                    samples = task.read(number_of_samples_per_channel=100, timeout=1.0)
                    now = time.time()
                    with self.lock:
                        self.timestamps.extend([now + i / self.rate for i in range(len(samples))])
                        self.values.extend(samples)
                    # print(f"Read {len(samples)} samples at t={now:.3f}")

        except Exception as e:
            print(f"DAQ thread error: {e}")

    def stop(self):
        self.running = False

    def get_data(self):
        with self.lock:
            return list(self.timestamps), list(self.values)

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
    # Start Analog Reader Thread
    # -------------------------------
    analog_thread = AnalogReader(DAQ_DEVICE, DAQ_CHANNEL, DAQ_SAMPLING_RATE)
    analog_thread.start()
    print(f"Started analog recording on {DAQ_DEVICE}/{DAQ_CHANNEL} at {DAQ_SAMPLING_RATE} Hz.")

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
        csv_writer.writerow(["FrameID", "Timestamp_ns", "Line3_State"])

        video_writer = None
        frame_count = 0
        last_display = time.time()
        display_interval = 1.0 / DISPLAY_FPS

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
                    csv_writer.writerow([frame_count, image_result.GetTimeStamp(), int(line3_state)])
                    frame_count += 1

                    now = time.time()
                    if now - last_display >= display_interval:
                        cv2.imshow("Live Camera Feed", frame)
                        last_display = now

                image_result.Release()
                del image_result

            except PySpin.SpinnakerException as ex:
                print(f"Acquisition error: {ex}")
                break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Stopping acquisition.")
                break

        cam.EndAcquisition()

    finally:
        try:
            cam.EndAcquisition()
        except:
            pass

        analog_thread.stop()
        analog_thread.join()
        print("Analog acquisition stopped.")

        if video_writer:
            video_writer.release()
        csv_file.close()

        # Save analog data
        analog_timestamps, analog_values = analog_thread.get_data()
        with open(analog_csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp_s", "AnalogValue_V"])
            for t, v in zip(analog_timestamps, analog_values):
                writer.writerow([t, v])

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
        print(f"Analog log saved: {analog_csv_path}")

if __name__ == "__main__":
    main()
