import cv2
import numpy as np
import time
import threading

class V4L2Camera:
    """
    A class to interface with a V4L2 camera using OpenCV's VideoCapture
    configured to capture the raw, hardware-encoded JPEG stream directly.
    This class is designed to be thread-safe.
    """
    V4L2_EXPOSURE_MANUAL = 1
    V4L2_EXPOSURE_AUTO = 3
    
    # Mapping from human-readable names to OpenCV property IDs
    CV_PROP_MAP = {
        'brightness': cv2.CAP_PROP_BRIGHTNESS,
        'contrast': cv2.CAP_PROP_CONTRAST,
        'saturation': cv2.CAP_PROP_SATURATION,
        'hue': cv2.CAP_PROP_HUE,
        'gamma': cv2.CAP_PROP_GAMMA,
        'gain': cv2.CAP_PROP_GAIN,
        'sharpness': cv2.CAP_PROP_SHARPNESS,
        'auto_exposure': cv2.CAP_PROP_AUTO_EXPOSURE,
        'exposure_absolute': cv2.CAP_PROP_EXPOSURE
    }

    def __init__(self, device_id, width, height, framerate, logger, initial_controls=None):
        self.logger = logger
        # Use a re-entrant lock to make the class thread-safe.
        # This prevents race conditions when methods are called from different threads
        # (e.g., image capture timer vs. parameter service callback).
        self.lock = threading.RLock()
        
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)

        if not self.cap or not self.cap.isOpened():
            raise RuntimeError(f"Failed to open video device /dev/video{device_id}")

        # Set MJPEG format
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        if not self.cap.set(cv2.CAP_PROP_FOURCC, fourcc):
            self.logger.warn("Failed to set FOURCC to MJPG. Camera might not support it.")

        # Set resolution and framerate
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, framerate)

        # ** THE IMPORTANT PART **
        # Disable automatic conversion from JPEG to BGR
        # This makes `read()` return the raw JPEG data
        if not self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0):
             self.logger.warn("Failed to disable RGB conversion. Raw JPEG capture may not work.")
        
        # Give camera time to apply settings
        time.sleep(1.0)

        # Apply initial camera control parameters
        if initial_controls:
            self.logger.info("Applying initial camera controls...")
            self.set_controls(initial_controls)

        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        actual_fourcc_int = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        actual_fourcc_str = "".join([chr((actual_fourcc_int >> 8 * i) & 0xFF) for i in range(4)])

        self.logger.info(f"V4L2 Camera {device_id} opened. Requested: {width}x{height}@{framerate}fps MJPG. "
                         f"Actual: {int(actual_width)}x{int(actual_height)}@{actual_fps:.2f}fps {actual_fourcc_str}")

        if 'MJPG' not in actual_fourcc_str:
             self.logger.warn(f"Camera did not select MJPG format (current: {actual_fourcc_str}). "
                              "Direct JPEG capture may not work as expected.")
        
        self.fps = actual_fps if actual_fps > 0 else float(framerate)

    def set_controls(self, controls):
        """Sets multiple camera controls from a dictionary."""
        with self.lock:
            for name, value in controls.items():
                if name not in self.CV_PROP_MAP:
                    self.logger.warn(f"Control '{name}' is not a recognized camera property.")
                    continue

                prop_id = self.CV_PROP_MAP[name]
                success = self.cap.set(prop_id, float(value)) # Most OpenCV properties expect float
                
                if not success:
                    self.logger.warn(f"Failed to set control '{name}' to {value}")

    def get_all_controls(self):
        """
        Gets the current values of all supported camera controls from the hardware
        and returns them in a dictionary with node-friendly keys.
        """
        with self.lock:
            current_controls = {}
            if not self.is_opened():
                self.logger.warn("Cannot get controls, camera is not open.")
                return current_controls
            
            for name, prop_id in self.CV_PROP_MAP.items():
                value = self.cap.get(prop_id)
                if value is None or value < 0: # Some drivers return -1 for unsupported controls
                    continue

                # Remap keys and values to match ROS parameter names and types where needed.
                if name == 'auto_exposure':
                    # OpenCV's V4L2 backend returns 3 for AUTO and 1 for MANUAL.
                    current_controls['auto_exposure'] = int(value)
                elif name == 'exposure_absolute':
                    # Remap 'exposure_absolute' to 'exposure_time' to match the ROS parameter.
                    current_controls['exposure_time'] = int(value)
                else:
                    current_controls[name] = int(value)
            return current_controls

    def read_jpeg(self):
        """
        Reads a frame from the camera. Since CAP_PROP_CONVERT_RGB is false,
        this should return the raw JPEG data.
        """
        with self.lock:
            if not self.is_opened(): return None
            ret, frame = self.cap.read()
            if not ret:
                self.logger.warn("Failed to read frame from camera.")
                return None
            
            # When CAP_PROP_CONVERT_RGB is 0 and format is MJPG, OpenCV returns
            # the raw JPEG data as a 2D numpy array. The shape can vary between
            # backends/drivers (e.g., (N, 1) or (1, N)).
            # We check if it's a 2D array and then convert to bytes, which is robust.
            if frame is not None and frame.ndim == 2:
                return frame.tobytes()
            else:
                self.logger.error("Read frame but it was not in the expected raw JPEG format. "
                                    f"Shape: {frame.shape if frame is not None else 'None'}, "
                                    f"Dtype: {frame.dtype if frame is not None else 'None'}")
                return None

    def release(self):
        with self.lock:
            if self.cap and self.cap.isOpened():
                self.cap.release()
                self.logger.info("V4L2 camera released.")

    def is_opened(self):
        # This method is called by others within a lock, but can also be called externally.
        with self.lock:
            return self.cap and self.cap.isOpened()