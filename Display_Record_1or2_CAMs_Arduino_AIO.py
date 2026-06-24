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

# =======================
# USER PARAMETERS
# =======================
DEFAULT_START_DIR = r"C:\Users\Behavior4\Documents\Camera_Recordings"
DISPLAY_FPS = 20.0  # Live display FPS

SERIAL_PORT = "COM9"
SERIAL_BAUDRATE = 115200


# =======================
# GUI FOLDER + NAME PROMPT
# =======================
root = tk.Tk()
root.withdraw()  # Hide the main tkinter window

# Folder selection
OUTPUT_DIR = filedialog.askdirectory(
    title="Select Output Folder",
    initialdir=DEFAULT_START_DIR
)

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
FRAME_CSV_FILENAME = BASE_NAME + ".csv"
ANALOG_CSV_FILENAME = BASE_NAME + "_Analog.csv"

frame_csv_path = os.path.join(OUTPUT_DIR, FRAME_CSV_FILENAME)
analog_csv_path = os.path.join(OUTPUT_DIR, ANALOG_CSV_FILENAME)


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


def _resize_to_height(img, h=480):
    if img is None:
        return None

    oh, ow = img.shape[:2]

    if oh <= 0 or ow <= 0:
        return None

    w = int(ow * (h / oh))
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)


def _black_like(img=None, h=480):
    return np.zeros((h, int(h * 4 / 3), 3), dtype=np.uint8)


def _put_hud(img, text, org=(10, 25)):
    if img is None:
        return

    cv2.putText(
        img,
        text,
        org,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA
    )


# =======================
# ARDUINO BACKGROUND THREAD
# =======================
class SerialDecoderReader(threading.Thread):
    def __init__(self, port="COM9", baudrate=115200):
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
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode("utf-8").strip()

                    if line.startswith("TRIAL:"):
                        try:
                            parts = line.split(",")

                            trial = int(parts[0].split(":")[1])
                            stim = int(parts[1].split(":")[1])
                            now = time.time()

                            with self.lock:
                                self.timestamps.append(now)
                                self.trials.append(trial)
                                self.stims.append(stim)

                            print(f"[{now:.3f}] Trial={trial}, Stim={stim}")

                        except Exception as e:
                            print(f"Error decoding serial line: {line} | {e}")

            except Exception:
                break

    def stop(self):
        self.running = False

        try:
            self.serial_port.close()
        except Exception:
            pass

    def get_latest_values(self):
        with self.lock:
            if self.timestamps:
                return self.trials[-1], self.stims[-1]
            else:
                return None, None

    def get_data(self):
        with self.lock:
            return (
                list(self.timestamps),
                list(self.trials),
                list(self.stims)
            )


# =======================
# MAIN FUNCTION
# =======================
def main():
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    system = None
    cam_list = None
    cams = []

    serial_thread = None
    csv_file = None

    video_writers = []
    frame_counts = []
    dropped = []
    last_frames = []
    video_paths = []

    processor = None

    try:
        # -------------------------------
        # Detect cameras first
        # -------------------------------
        system = PySpin.System.GetInstance()
        cam_list = system.GetCameras()

        num_cams = cam_list.GetSize()

        if num_cams < 1:
            print("⚠️ No camera detected. Aborting.")
            return

        # Use one camera if only one is detected.
        # Use two cameras if two or more are detected.
        n_use = min(num_cams, 2)

        print(f"Detected {num_cams} camera(s). Using {n_use} camera(s).")

        # -------------------------------
        # Build real output paths
        # -------------------------------
        video_paths = [
            os.path.join(OUTPUT_DIR, BASE_NAME + f"_cam{i}.avi")
            for i in range(n_use)
        ]

        # -------------------------------
        # CHECK FOR EXISTING FILES
        # Fixed overwrite bug:
        # check the actual video names: BASE_NAME_cam0.avi, BASE_NAME_cam1.avi
        # -------------------------------
        files_to_check = video_paths + [frame_csv_path, analog_csv_path]

        existing_files = [
            f for f in files_to_check
            if os.path.exists(f)
        ]

        if existing_files:
            msg = (
                "The following files already exist:\n\n"
                + "\n".join(existing_files)
                + "\n\nOverwrite?"
            )

            if not messagebox.askyesno("Confirm Overwrite", msg):
                print("❌ Aborting to prevent overwrite.")
                return

        # -------------------------------
        # Start Serial Decoder Thread
        # -------------------------------
        try:
            serial_thread = SerialDecoderReader(
                port=SERIAL_PORT,
                baudrate=SERIAL_BAUDRATE )
            serial_thread.start()
            print(f"Serial decoder thread started on {SERIAL_PORT} at {SERIAL_BAUDRATE} baud.")
        except Exception as e:
            print(f"Failed to start serial thread: {e}")
            serial_thread = None

        # -------------------------------
        # Initialize cameras
        # -------------------------------
        cams = [cam_list[i] for i in range(n_use)]

        for i, cam in enumerate(cams):
            cam.Init()

            # Reduce latency and avoid queue buildup
            s_node_map = cam.GetTLStreamNodeMap()
            handling_mode = PySpin.CEnumerationPtr(
                s_node_map.GetNode("StreamBufferHandlingMode")
            )
            newest_only = handling_mode.GetEntryByName("NewestOnly")
            handling_mode.SetIntValue(newest_only.GetValue())

            # Optional: lock FPS if available
            try:
                if (
                    PySpin.IsAvailable(cam.AcquisitionFrameRateEnable)
                    and PySpin.IsWritable(cam.AcquisitionFrameRateEnable)
                ):
                    cam.AcquisitionFrameRateEnable.SetValue(True)
            except Exception:
                pass

            print(f"Initialized cam{i}")

        processor = PySpin.ImageProcessor()
        processor.SetColorProcessing(
            PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR
        )

        # -------------------------------
        # Prepare CSV
        # -------------------------------
        csv_file = open(frame_csv_path, "w", newline="")
        csv_writer = csv.writer(csv_file)

        csv_writer.writerow([
            "CamID",
            "FrameID",
            "Timestamp_ns",
            "Line3_State",
            "Trial",
            "Stimulus"
        ])

        # -------------------------------
        # Prepare writers and counters
        # -------------------------------
        video_writers = [None] * n_use
        frame_counts = [0] * n_use
        dropped = [0] * n_use
        last_frames = [None] * n_use

        # -------------------------------
        # Start acquisition
        # -------------------------------
        print("Starting acquisition...")

        for cam in cams:
            cam.BeginAcquisition()

        # -------------------------------
        # Display windows
        # -------------------------------
        for i in range(n_use):
            win_name = f"Live Cam{i}"
            cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
            cv2.moveWindow(win_name, 50 + i * 650, 50)

        last_display = time.time()
        display_interval = 1.0 / DISPLAY_FPS

        def grab_write_one(cam, cam_id, vw, frame_count, dropped_count):
            """
            Grab one frame from one camera.

            Returns:
                vw, frame_count, dropped_count, frame_for_display_or_None
            """

            try:
                # Short timeout because we poll cameras sequentially
                img = cam.GetNextImage(5)

                if img.IsIncomplete():
                    dropped_count += 1
                    img.Release()
                    return vw, frame_count, dropped_count, None

                frame = convert_to_numpy(img, processor)

                # Create video writer on first valid frame
                if vw is None:
                    h, w = frame.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*"MJPG")

                    try:
                        fps = cam.AcquisitionFrameRate.GetValue()
                    except Exception:
                        fps = DISPLAY_FPS

                    vw = cv2.VideoWriter(
                        video_paths[cam_id],
                        fourcc,
                        fps,
                        (w, h)
                    )

                    print(f"Recording cam{cam_id} to {video_paths[cam_id]}")
                    print(f"cam{cam_id} video FPS = {fps:.2f}")

                vw.write(frame)

                # Log digital input + timestamp
                try:
                    line3_state = get_line3_state(cam)
                except Exception:
                    line3_state = 0

                if serial_thread:
                    trial, stim = serial_thread.get_latest_values()
                else:
                    trial, stim = None, None

                csv_writer.writerow([
                    cam_id,
                    frame_count,
                    img.GetTimeStamp(),
                    int(line3_state),
                    trial,
                    stim
                ])

                frame_count += 1

                img.Release()

                return vw, frame_count, dropped_count, frame

            except PySpin.SpinnakerException:
                # Treat as no frame available during this poll
                return vw, frame_count, dropped_count, None

        # -------------------------------
        # Main acquisition loop
        # -------------------------------
        while True:
            # Poll each available camera
            for i, cam in enumerate(cams):
                (
                    video_writers[i],
                    frame_counts[i],
                    dropped[i],
                    frame
                ) = grab_write_one(
                    cam,
                    i,
                    video_writers[i],
                    frame_counts[i],
                    dropped[i]
                )

                if frame is not None:
                    last_frames[i] = frame

            # Display at a fixed cadence
            now = time.time()

            if now - last_display >= display_interval:
                for i in range(n_use):
                    disp = _resize_to_height(last_frames[i], 480)

                    if disp is None:
                        disp = _black_like(None, 480)

                    _put_hud(
                        disp,
                        f"Cam{i} | Frames: {frame_counts[i]} | Drops: {dropped[i]}"
                    )

                    cv2.imshow(f"Live Cam{i}", disp)

                last_display = now

            # Quit
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("Stopping acquisition.")
                break

    finally:
        # -------------------------------
        # Close display
        # -------------------------------
        try:
            cv2.destroyAllWindows()
            time.sleep(0.2)
        except Exception:
            pass

        # -------------------------------
        # End acquisition
        # -------------------------------
        for cam in cams:
            try:
                cam.EndAcquisition()
            except Exception:
                pass

        # -------------------------------
        # Release video writers
        # -------------------------------
        for vw in video_writers:
            try:
                if vw:
                    vw.release()
            except Exception:
                pass

        # -------------------------------
        # Close frame CSV
        # -------------------------------
        try:
            if csv_file:
                csv_file.close()
        except Exception:
            pass

        # -------------------------------
        # Serial thread + analog CSV
        # -------------------------------
        if serial_thread:
            try:
                serial_thread.stop()
                serial_thread.join(timeout=2)
            except Exception:
                pass

            try:
                serial_timestamps, serial_trials, serial_stims = serial_thread.get_data()

                with open(analog_csv_path, "w", newline="") as f:
                    w = csv.writer(f)
                    w.writerow(["Timestamp_s", "Trial", "Stimulus"])

                    for t, tr, st in zip(
                        serial_timestamps,
                        serial_trials,
                        serial_stims
                    ):
                        w.writerow([t, tr, st])

            except Exception as e:
                print(f"Could not save analog CSV: {e}")

        # -------------------------------
        # Deinit cameras
        # -------------------------------
        for cam in cams:
            try:
                cam.DeInit()
            except Exception:
                pass

        try:
            del cams
        except Exception:
            pass

        # -------------------------------
        # Clear cam list and release system
        # -------------------------------
        try:
            if cam_list:
                cam_list.Clear()
        except Exception:
            pass

        try:
            if system:
                system.ReleaseInstance()
        except Exception:
            pass

        try:
            gc.collect()
        except Exception:
            pass

        # -------------------------------
        # Summary
        # -------------------------------
        if dropped:
            print("⚠️ Dropped frames:")
            for i, d in enumerate(dropped):
                print(f"  Cam{i}: {d}")

        if frame_counts:
            print("Recorded frames:")
            for i, n in enumerate(frame_counts):
                print(f"  Cam{i}: {n}")

        if video_paths:
            print("Recording saved:")
            for i, path in enumerate(video_paths):
                print(f"  cam{i}: {path}")

        print(f"Frame log saved: {frame_csv_path}")
        print(f"Analog log saved: {analog_csv_path}")


if __name__ == "__main__":
    main()