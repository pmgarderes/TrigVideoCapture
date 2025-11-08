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

# --- helper: resize & stack side-by-side (keeps aspect ratio) ---
def _resize_to_height(img, h=480):
    if img is None: 
        return None
    oh, ow = img.shape[:2]
    if oh <= 0 or ow <= 0:
        return None
    w = int(ow * (h / oh))
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)

def _black_like(img, h=480):
    # fallback canvas if we never got a frame yet
    return np.zeros((h, int(h*4/3), 3), dtype=np.uint8)  # 4:3 default-ish


def _stack_side_by_side(img0, img1, target_h=480):
    f0 = _resize_to_height(img0, target_h) if img0 is not None else None
    f1 = _resize_to_height(img1, target_h) if img1 is not None else None
    if f0 is not None and f1 is not None:
        return np.hstack([f0, f1])
    return f0 if f0 is not None else f1

# optional: overlay quick text (fps, drops, etc.)
def _put_hud(img, text, org=(10,25)):
    if img is None: return
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)


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
        serial_thread = SerialDecoderReader(port='COm6', baudrate=115200)  # ✅ adjust COM port if needed!
        serial_thread.start()
        print("Serial decoder thread started.")
    except Exception as e:
        print(f"Failed to start serial thread: {e}")
        serial_thread = None

     
    # -------------------------------
    # Initialize Cameras (2 cams)
    # -------------------------------
    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()
    num_cams = cam_list.GetSize()
    if num_cams < 2:
        print(f"⚠️ Need 2 cameras; detected {num_cams}. Aborting.")
        cam_list.Clear()
        system.ReleaseInstance()
        return

    # Pick first two cams (or sort by serial, etc.)
    cam0 = cam_list[0]
    cam1 = cam_list[1]
    for cam in (cam0, cam1):
        cam.Init()
        # Reduce latency and avoid queue buildup
        s_node_map = cam.GetTLStreamNodeMap()
        handling_mode = PySpin.CEnumerationPtr(s_node_map.GetNode("StreamBufferHandlingMode"))
        newest_only = handling_mode.GetEntryByName("NewestOnly")
        handling_mode.SetIntValue(newest_only.GetValue())
        # Optional: lock FPS if needed
        try:
            if PySpin.IsAvailable(cam.AcquisitionFrameRateEnable) and PySpin.IsWritable(cam.AcquisitionFrameRateEnable):
                cam.AcquisitionFrameRateEnable.SetValue(True)
        except:
            pass

    processor = PySpin.ImageProcessor()
    processor.SetColorProcessing(PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR)

    # Derive per-camera file names
    VIDEO_FILENAME0 = BASE_NAME + "_cam0.avi"
    VIDEO_FILENAME1 = BASE_NAME + "_cam1.avi"
    video_path0 = os.path.join(OUTPUT_DIR, VIDEO_FILENAME0)
    video_path1 = os.path.join(OUTPUT_DIR, VIDEO_FILENAME1)

    # Combined CSV for both cameras
    frame_csv_path = os.path.join(OUTPUT_DIR, FRAME_CSV_FILENAME)  # keep your name; now includes both cams

    # Prepare CSV
    csv_file = open(frame_csv_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["CamID", "FrameID", "Timestamp_ns", "Line3_State", "Trial", "Stimulus"])

    # Prepare writers and counters
    video_writer0 = None
    video_writer1 = None
    frame_count0 = 0
    frame_count1 = 0
    dropped0 = 0
    dropped1 = 0

    print("Starting acquisition on both cameras...")
    cam0.BeginAcquisition()
    cam1.BeginAcquisition()
    
    cv2.namedWindow("Live Cam0", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Live Cam1", cv2.WINDOW_NORMAL)
    # (optional) position them
    cv2.moveWindow("Live Cam0", 50, 50)
    cv2.moveWindow("Live Cam1", 700, 50)

    last_frame0 = None
    last_frame1 = None


    last_display = time.time()
    display_interval = 1.0 / DISPLAY_FPS

    def grab_write_one(cam, cam_id, vw, frame_count, dropped_count):
        """Grab one frame from a camera; returns (vw, frame_count, dropped_count, frame_for_display_or_None)"""
        try:
            img = cam.GetNextImage(5)  # short timeout; we poll both cams quickly
            if img.IsIncomplete():
                dropped_count += 1
                img.Release()
                return vw, frame_count, dropped_count, None
            frame = convert_to_numpy(img, processor)

            if vw is None:
                h, w = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                # Use each cam's own reported FPS if available; fallback to DISPLAY_FPS
                try:
                    fps = cam.AcquisitionFrameRate.GetValue()
                except:
                    fps = DISPLAY_FPS
                vw = cv2.VideoWriter((video_path0 if cam_id==0 else video_path1), fourcc, fps, (w, h))
                print(f"Recording cam{cam_id} to", (video_path0 if cam_id==0 else video_path1))

            vw.write(frame)

            # Log digital input + timestamp
            try:
                line3_state = get_line3_state(cam)
            except:
                line3_state = 0
            trial, stim = (serial_thread.get_latest_values() if serial_thread else (None, None))
            csv_writer.writerow([cam_id, frame_count, img.GetTimeStamp(), int(line3_state), trial, stim])

            frame_count += 1
            img.Release()
            return vw, frame_count, dropped_count, frame

        except PySpin.SpinnakerException:
            # treat as no frame in this poll
            return vw, frame_count, dropped_count, None

    try:
        while True:
            # Round-robin grab to keep both pipes flowing
            video_writer0, frame_count0, dropped0, frame0 = grab_write_one(cam0, 0, video_writer0, frame_count0, dropped0)
            video_writer1, frame_count1, dropped1, frame1 = grab_write_one(cam1, 1, video_writer1, frame_count1, dropped1)
            
            # After  compute frame0 and frame1 (may be None if incomplete/timeout):
            if frame0 is not None:
                last_frame0 = frame0
            if frame1 is not None:
                last_frame1 = frame1
            
            # Display at a fixed cadence
            now = time.time()
            if now - last_display >= display_interval:
                # Use cached frames; if still None, show a black placeholder
                disp0 = _resize_to_height(last_frame0, 480)
                if disp0 is None: 
                    disp0 = _black_like(None, 480)

                disp1 = _resize_to_height(last_frame1, 480)
                if disp1 is None: 
                    disp1 = _black_like(None, 480)

                cv2.imshow("Live Cam0", disp0)
                cv2.imshow("Live Cam1", disp1)

                last_display = now


            # Quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Stopping acquisition.")
                break

    finally:
        
        # Close display
        cv2.destroyAllWindows()
        time.sleep(0.2)

        # End acquisition (safe even if already stopped)
        for cam in (cam0, cam1):
            try: cam.EndAcquisition()
            except: pass

        # Writers/CSV
        if video_writer0: video_writer0.release()
        if video_writer1: video_writer1.release()
        csv_file.close()

        # Serial thread
        if serial_thread:
            try:
                serial_thread.stop()
                serial_thread.join()
            except: pass
            serial_timestamps, serial_trials, serial_stims = serial_thread.get_data()
            with open(analog_csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["Timestamp_s", "Trial", "Stimulus"])
                for t, tr, st in zip(serial_timestamps, serial_trials, serial_stims):
                    w.writerow([t, tr, st])

        # Deinit cams and system
        for cam in (cam0, cam1):
            try: cam.DeInit()
            except: pass
        del cam0, cam1
        cam_list.Clear()
        del cam_list
        system.ReleaseInstance()
        del system

        # Summary
        print(f"⚠️ Dropped frames — Cam0: {dropped0}, Cam1: {dropped1}")
        print(f"Recording saved:\n  cam0: {video_path0}\n  cam1: {video_path1}")
        print(f"Frame log saved: {frame_csv_path}")


if __name__ == "__main__":
    main()

