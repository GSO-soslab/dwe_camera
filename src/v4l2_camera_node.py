#!/usr/bin/env python3

import rospy
import v4l2
import fcntl
import select
import mmap
import time
# import ctypes # No longer needed for buffer access

# Removed CvBridge and Image unless you uncomment raw publishing
from dynamic_reconfigure.server import Server
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage # Image removed
from dwe_camera.msg import CamParameters
# Ensure this matches the generated config from the MODIFIED cfg file
from dwe_camera.cfg import CamSettingsConfig # <--- Make sure this is correct after rebuild

# --- Manually define missing V4L2 constants if needed ---
# From /usr/include/linux/videodev2.h (or similar)
try:
    _ = v4l2.V4L2_BUF_FLAG_ERROR # Check if it exists
except AttributeError:
    rospy.logwarn("v4l2 module missing V4L2_BUF_FLAG_ERROR, defining manually (0x00000040).")
    v4l2.V4L2_BUF_FLAG_ERROR = 0x00000040

# Define MJPEG if missing (value from videodev2.h)
try:
    _ = v4l2.V4L2_PIX_FMT_MJPEG
except AttributeError:
     rospy.logwarn("v4l2 module missing V4L2_PIX_FMT_MJPEG, defining manually (0x47504A4D).")
     v4l2.V4L2_PIX_FMT_MJPEG = 0x47504A4D # 'MJPG'

# --- Helper Function to Decode FourCC ---
def fourcc_int_to_str(fourcc_int):
    """ Converts an integer FourCC code to a 4-character string. """
    if not isinstance(fourcc_int, int) or fourcc_int < 0:
        return "N/A"
    try:
        # Use standard C-style conversion: int -> 4 bytes -> string
        return "".join([chr((fourcc_int >> 8 * i) & 0xFF) for i in range(4)])
    except Exception:
        # Fallback if conversion fails (e.g., non-ASCII)
        return f"0x{fourcc_int:X}"

# Mapping from config/msg names or internal keys to V4L2 Control IDs
# Based on v4l2-ctl -L output for your camera
V4L2_CONTROL_MAP = {
    # User Controls (match cfg/msg names)
    'brightness': v4l2.V4L2_CID_BRIGHTNESS,
    'contrast': v4l2.V4L2_CID_CONTRAST,
    'saturation': v4l2.V4L2_CID_SATURATION,
    'hue': v4l2.V4L2_CID_HUE,
    'gamma': v4l2.V4L2_CID_GAMMA,
    'gain': v4l2.V4L2_CID_GAIN,
    'sharpness': v4l2.V4L2_CID_SHARPNESS,
    'auto_exposure': v4l2.V4L2_CID_EXPOSURE_AUTO, # Map msg field name
    '_v4l2_exposure_auto_id': v4l2.V4L2_CID_EXPOSURE_AUTO, # Internal key for lookup
    'exposure': v4l2.V4L2_CID_EXPOSURE_ABSOLUTE, # Manual exposure time
}

# Define the V4L2 values for exposure modes for this specific camera
# Based on v4l2-ctl output: "1: Manual Mode", "3: Aperture Priority Mode"
V4L2_EXPOSURE_MANUAL_MODE_VALUE = 1
V4L2_EXPOSURE_AUTO_MODE_VALUE = 3 # Aperture Priority is the 'auto' mode here

class V4L2CameraNode:
    def __init__(self):
        rospy.init_node('v4l2_camera_node')
        rospy.loginfo("Initializing V4L2 Camera Node...")

        # --- Get ROS Parameters ---
        self.device_name = rospy.get_param('~video/device', '/dev/video0')
        self.width = rospy.get_param('~video/width', 800)
        self.height = rospy.get_param('~video/height', 600)
        self.fps = rospy.get_param('~video/fps', 15)
        self.frame_id = rospy.get_param('~video/frame_id', 'v4l2_camera')
        self.num_buffers = rospy.get_param('~video/buffers', 4)
        self.initial_auto_exposure_mode = rospy.get_param('~video/auto_exposure', V4L2_EXPOSURE_AUTO_MODE_VALUE)
        self.initial_exposure_time = rospy.get_param('~video/exposure_time', 156)
        # REMOVED: settings_publish_rate parameter

        # --- Publishers ---
        self.compressed_pub = rospy.Publisher("~image/compressed", CompressedImage, queue_size=self.num_buffers)
        self.cam_settings_pub = rospy.Publisher("~camera_settings", CamParameters, queue_size=self.num_buffers)

        # --- Subscriber ---
        self.cam_settings_sub = rospy.Subscriber("~set_cam_settings", CamParameters, self.cam_settings_callback)

        # --- V4L2 Initialization ---
        self.vd = None
        self.buffers = []
        self.is_streaming = False
        self.supported_controls = {}
        self.control_details = {}
        # REMOVED: self.settings_timer = None

        try:
            # Initialize V4L2 (opens device, queries controls, etc.)
            self.init_v4l2()

            # Set initial auto exposure mode based on ROS param AFTER init_v4l2
            self.set_initial_exposure_mode_from_param()

            # Set initial manual exposure time (only takes effect if mode is manual)
            self.set_initial_exposure_time_from_param()
        except (IOError, OSError, ValueError, AttributeError) as e:
            rospy.logfatal(f"Failed to initialize V4L2 device {self.device_name}: {e}", exc_info=True)
            rospy.signal_shutdown("V4L2 Initialization Failed")
            return
        except Exception as e:
            rospy.logfatal(f"Unexpected error during V4L2 initialization: {e}", exc_info=True)
            rospy.signal_shutdown("V4L2 Initialization Failed")
            return

        # --- Dynamic Reconfigure ---
        self.srv = Server(CamSettingsConfig, self.dynamic_config_callback)

        # --- Start Capture Thread ---
        from threading import Thread
        self.capture_thread = Thread(target=self.capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()

        rospy.loginfo("V4L2 Camera Node Initialized Successfully.")
        rospy.on_shutdown(self.shutdown)


    # --- init_v4l2, query_controls, query_menu_items ---
    # --- get_v4l2_control, set_v4l2_control ---
    # (These methods remain the same as the previous correct version)
    def init_v4l2(self):
        rospy.loginfo(f"Opening device: {self.device_name}")
        # Ensure previous device is closed if re-initializing (though not typical)
        if self.vd and not self.vd.closed:
             self.vd.close()
        self.vd = open(self.device_name, 'rb+', buffering=0)

        # 1. Query Capabilities
        caps = v4l2.v4l2_capability()
        fcntl.ioctl(self.vd, v4l2.VIDIOC_QUERYCAP, caps)
        if not (caps.capabilities & v4l2.V4L2_CAP_VIDEO_CAPTURE):
            raise ValueError("Device does not support video capture.")
        if not (caps.capabilities & v4l2.V4L2_CAP_STREAMING):
            raise ValueError("Device does not support streaming.")
        # rospy.loginfo(f"Device Name: {caps.driver.decode('utf-8')}")
        # rospy.loginfo(f"Capabilities: 0x{caps.capabilities:X}")

        # 2. List and Select Format (MJPEG)
        fmt_desc = v4l2.v4l2_fmtdesc()
        fmt_desc.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        mjpeg_supported = False
        rospy.logdebug("Checking supported formats...")
        while True:
            try:
                fcntl.ioctl(self.vd, v4l2.VIDIOC_ENUM_FMT, fmt_desc)
                # Use the helper function here
                pixel_format_str = fourcc_int_to_str(fmt_desc.pixelformat)
                rospy.logdebug(f"  Supported format: {fmt_desc.description.decode('utf-8')} "
                               f"({pixel_format_str}, 0x{fmt_desc.pixelformat:X})")
                if fmt_desc.pixelformat == v4l2.V4L2_PIX_FMT_MJPEG:
                    mjpeg_supported = True
                fmt_desc.index += 1
            except IOError: # No more formats
                break
            except Exception as e:
                 rospy.logwarn(f"Error processing format description: {e}")
                 fmt_desc.index += 1 # Try next index anyway

        if not mjpeg_supported:
             # Ensure V4L2_PIX_FMT_MJPEG was defined, otherwise this check is meaningless
             if not hasattr(v4l2, 'V4L2_PIX_FMT_MJPEG'):
                 rospy.logerr("V4L2_PIX_FMT_MJPEG constant not available in v4l2 module!")
             raise ValueError("Device does not support MJPEG format (V4L2_PIX_FMT_MJPEG).")
        rospy.loginfo("MJPEG format supported.")

        # 3. Set Format
        fmt = v4l2.v4l2_format()
        fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        # Try getting current format first
        try:
            fcntl.ioctl(self.vd, v4l2.VIDIOC_G_FMT, fmt)
            # Use the helper function here
            current_format_str = fourcc_int_to_str(fmt.fmt.pix.pixelformat)
            rospy.loginfo(f"Current format: {current_format_str} "
                          f"{fmt.fmt.pix.width}x{fmt.fmt.pix.height}")
        except IOError as e:
            rospy.logwarn(f"Could not get current format (VIDIOC_G_FMT): {e}")
            fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE

        # Set desired format
        fmt.fmt.pix.width = self.width
        fmt.fmt.pix.height = self.height
        fmt.fmt.pix.pixelformat = v4l2.V4L2_PIX_FMT_MJPEG # Use constant
        fmt.fmt.pix.field = v4l2.V4L2_FIELD_ANY
        try:
            fcntl.ioctl(self.vd, v4l2.VIDIOC_S_FMT, fmt)
            # Read back the format that was actually set
            fcntl.ioctl(self.vd, v4l2.VIDIOC_G_FMT, fmt)
            set_width = fmt.fmt.pix.width
            set_height = fmt.fmt.pix.height
            # Use the helper function here
            set_format_str = fourcc_int_to_str(fmt.fmt.pix.pixelformat)
            rospy.loginfo(f"Set format to {set_format_str} {set_width}x{set_height}")
            if self.width != set_width or self.height != set_height:
                 rospy.logwarn(f"Requested {self.width}x{self.height}, but driver set {set_width}x{set_height}")
                 self.width = set_width
                 self.height = set_height
        except IOError as e:
            rospy.logerr(f"Failed to set format (VIDIOC_S_FMT): {e}. Check if resolution/format is supported (`v4l2-ctl --list-formats-ext`)")
            raise

        # 4. Set Framerate (Optional, sometimes part of S_FMT)
        parm = v4l2.v4l2_streamparm()
        parm.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        # Try reading current params first
        try:
             fcntl.ioctl(self.vd, v4l2.VIDIOC_G_PARM, parm)
             current_fps_den = parm.parm.capture.timeperframe.denominator
             current_fps_num = parm.parm.capture.timeperframe.numerator
             if current_fps_num > 0:
                  rospy.loginfo(f"Current framerate: {current_fps_den}/{current_fps_num} "
                                f"({current_fps_den/current_fps_num:.2f} FPS)")
             else:
                  rospy.loginfo("Current framerate denominator is 0.")
        except IOError:
             rospy.logwarn("Could not get current stream parameters (VIDIOC_G_PARM).")

        if parm.parm.capture.capability & v4l2.V4L2_CAP_TIMEPERFRAME:
            parm.parm.capture.timeperframe.numerator = 1
            parm.parm.capture.timeperframe.denominator = self.fps
            try:
                fcntl.ioctl(self.vd, v4l2.VIDIOC_S_PARM, parm)
                rospy.loginfo(f"Requested framerate {self.fps} FPS.")
                # Read back actual framerate
                fcntl.ioctl(self.vd, v4l2.VIDIOC_G_PARM, parm)
                actual_fps_den = parm.parm.capture.timeperframe.denominator
                actual_fps_num = parm.parm.capture.timeperframe.numerator
                if actual_fps_num > 0:
                    self.fps = actual_fps_den / actual_fps_num
                    rospy.loginfo(f"Actual framerate set to: {self.fps:.2f} FPS ({actual_fps_den}/{actual_fps_num})")
                else:
                    rospy.logwarn("Could not read back valid framerate denominator after setting.")
            except IOError as e:
                rospy.logwarn(f"Failed to set framerate (VIDIOC_S_PARM): {e}. Using default.")
        else:
             rospy.loginfo("Device does not support setting framerate via VIDIOC_S_PARM.")


        # 5. Request Buffers
        req = v4l2.v4l2_requestbuffers()
        req.count = self.num_buffers
        req.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        req.memory = v4l2.V4L2_MEMORY_MMAP
        fcntl.ioctl(self.vd, v4l2.VIDIOC_REQBUFS, req)
        if req.count < 1:
             raise IOError("Insufficient buffer memory on device.")
        if req.count < self.num_buffers:
            rospy.logwarn(f"Requested {self.num_buffers} buffers, but only got {req.count}.")
            self.num_buffers = req.count
        rospy.loginfo(f"Allocated {self.num_buffers} buffers.")

        # 6. Query and Map Buffers
        self.buffers = []
        for i in range(self.num_buffers):
            buf = v4l2.v4l2_buffer()
            buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = v4l2.V4L2_MEMORY_MMAP
            buf.index = i
            fcntl.ioctl(self.vd, v4l2.VIDIOC_QUERYBUF, buf)
            # Map buffer
            mm = mmap.mmap(self.vd.fileno(), buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=buf.m.offset)
            self.buffers.append(mm)
            rospy.logdebug(f"Buffer {i}: length={buf.length}, offset={buf.m.offset}") # Removed mapped addr logging


        # 7. Queue Buffers
        for i in range(self.num_buffers):
            buf = v4l2.v4l2_buffer()
            buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = v4l2.V4L2_MEMORY_MMAP
            buf.index = i
            fcntl.ioctl(self.vd, v4l2.VIDIOC_QBUF, buf)
        rospy.loginfo("Queued initial buffers.")

        # 8. Query Supported Controls (calls the separate query_controls method)
        self.query_controls()

        # 9. Start Streaming
        buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(self.vd, v4l2.VIDIOC_STREAMON, buf_type)
        self.is_streaming = True
        rospy.loginfo("Streaming started.")

    def query_controls(self):
        """Query the camera for supported controls and store them."""
        rospy.loginfo("Querying supported V4L2 controls...")
        self.supported_controls = {} # Map name/internal_key -> v4l2_id
        self.control_details = {} # Map v4l2_id -> queryctrl details

        # Iterate through known controls we might care about
        for name_or_key, ctrl_id in V4L2_CONTROL_MAP.items(): # Use items()
            queryctrl = v4l2.v4l2_queryctrl(id=ctrl_id)
            try:
                fcntl.ioctl(self.vd, v4l2.VIDIOC_QUERYCTRL, queryctrl)
                if not (queryctrl.flags & v4l2.V4L2_CTRL_FLAG_DISABLED):
                    # Use the name_or_key from the map as the key in supported_controls
                    self.supported_controls[name_or_key] = ctrl_id
                    self.control_details[ctrl_id] = queryctrl # Store details using ID as key
                    control_name_str = queryctrl.name.decode('utf-8')
                    rospy.loginfo(f"  Found Control: {control_name_str} ({name_or_key}) ID: 0x{ctrl_id:X} "
                                  f"Type: {queryctrl.type} Min: {queryctrl.minimum} Max: {queryctrl.maximum} "
                                  f"Step: {queryctrl.step} Default: {queryctrl.default_value}")
                    if queryctrl.type == v4l2.V4L2_CTRL_TYPE_MENU:
                         self.query_menu_items(queryctrl)
                else:
                    rospy.logdebug(f"  Control '{name_or_key}' (0x{ctrl_id:X}) is disabled.")
            except IOError:
                rospy.logdebug(f"  Control '{name_or_key}' (0x{ctrl_id:X}) not supported.")
            except Exception as e:
                rospy.logwarn(f"  Error querying control '{name_or_key}' (0x{ctrl_id:X}): {e}")

        rospy.loginfo(f"Found {len(self.supported_controls)} supported and enabled controls relevant to this node.")

    def query_menu_items(self, queryctrl):
        """Query menu items for a V4L2_CTRL_TYPE_MENU control."""
        rospy.logdebug(f"    Querying menu items for {queryctrl.name.decode('utf-8')}:")
        for i in range(queryctrl.minimum, queryctrl.maximum + 1):
            querymenu = v4l2.v4l2_querymenu(id=queryctrl.id, index=i)
            try:
                fcntl.ioctl(self.vd, v4l2.VIDIOC_QUERYMENU, querymenu)
                try:
                     # Some drivers use .name (bytes), some use .__u64 (int pointer?)
                     # Try decoding .name first
                     item_name = querymenu.name.decode('utf-8')
                except AttributeError:
                     # Fallback if .name doesn't exist or isn't bytes
                     item_name = f"Index {querymenu.index}" # Generic fallback
                except UnicodeDecodeError:
                     item_name = f"Index {querymenu.index} (non-utf8 name)"


                rospy.logdebug(f"      Index {querymenu.index}: {item_name}")
            except IOError:
                # Don't break, some drivers might have gaps in indices
                rospy.logdebug(f"      Failed to query menu index {i} for control 0x{queryctrl.id:X}")
                # break # Original code broke here, might be better to continue
            except Exception as e:
                 rospy.logwarn(f"      Error querying menu index {i} for control 0x{queryctrl.id:X}: {e}")


    def get_v4l2_control(self, ctrl_id):
        """Gets the value of a V4L2 control."""
        if not self.vd or self.vd.closed:
             rospy.logwarn_throttle(5.0, "Attempted to get control, but V4L2 device not open")
             return None
        try:
            control = v4l2.v4l2_control(id=ctrl_id)
            fcntl.ioctl(self.vd, v4l2.VIDIOC_G_CTRL, control)
            return control.value
        except IOError as e:
            # Only log warning if we expected the control to be there and it's NOT the specific case
            # of trying to get exposure while in auto mode (which might fail legitimately).
            if ctrl_id in self.control_details:
                 current_auto_mode = self.get_v4l2_control(V4L2_CONTROL_MAP.get('_v4l2_exposure_auto_id'))
                 is_getting_exposure_in_auto = (ctrl_id == V4L2_CONTROL_MAP.get('exposure') and
                                                current_auto_mode == V4L2_EXPOSURE_AUTO_MODE_VALUE)

                 if not is_getting_exposure_in_auto:
                    rospy.logwarn_throttle(10.0, f"Failed to get control 0x{ctrl_id:X}: {e}")
                 else:
                     # Log less severely if getting exposure in auto mode fails
                     rospy.logdebug_throttle(10.0, f"Failed to get exposure control (0x{ctrl_id:X}) while in auto mode: {e}. This may be expected.")

            return None
        except Exception as e:
            rospy.logerr(f"Unexpected error getting control 0x{ctrl_id:X}: {e}")
            return None

    def set_v4l2_control(self, ctrl_id, value):
        """Sets the value of a V4L2 control."""
        if not self.vd or self.vd.closed:
             rospy.logwarn("Attempted to set control, but V4L2 device not open")
             return False
        if ctrl_id not in self.control_details:
             known_ids = [v for k, v in V4L2_CONTROL_MAP.items()]
             if ctrl_id in known_ids:
                  rospy.logwarn(f"Attempted to set control 0x{ctrl_id:X} which was not found/enabled during query.")
             else:
                  rospy.logwarn(f"Attempted to set unsupported/unknown control ID 0x{ctrl_id:X}")
             return False

        details = self.control_details[ctrl_id]

        clamped_value = int(value)
        if details.type != v4l2.V4L2_CTRL_TYPE_MENU and details.type != v4l2.V4L2_CTRL_TYPE_BOOLEAN:
             clamped_value = max(details.minimum, min(int(value), details.maximum))
             if clamped_value != int(value):
                  rospy.logwarn(f"Clamping value {value} to {clamped_value} for control 0x{ctrl_id:X} "
                                f"(Min: {details.minimum}, Max: {details.maximum})")
        elif details.type == v4l2.V4L2_CTRL_TYPE_BOOLEAN:
             clamped_value = 1 if value else 0 # Ensure 0 or 1 for bool

        try:
            control = v4l2.v4l2_control(id=ctrl_id, value=clamped_value)
            fcntl.ioctl(self.vd, v4l2.VIDIOC_S_CTRL, control)
            return True
        except IOError as e:
            current_auto_mode = self.get_v4l2_control(V4L2_CONTROL_MAP.get('_v4l2_exposure_auto_id'))
            is_setting_exposure_in_auto = (ctrl_id == V4L2_CONTROL_MAP.get('exposure') and
                                           current_auto_mode == V4L2_EXPOSURE_AUTO_MODE_VALUE)

            if not is_setting_exposure_in_auto:
                rospy.logerr(f"Failed to set control 0x{ctrl_id:X} to {clamped_value}: {e}")
            else:
                rospy.logdebug(f"Ignoring error setting manual exposure (0x{ctrl_id:X}) while in auto mode: {e}")

            return False
        except Exception as e:
            rospy.logerr(f"Unexpected error setting control 0x{ctrl_id:X}: {e}")
            return False

    def capture_loop(self):
        """Main loop to capture frames, get settings, and publish both synchronously."""
        frame_count = 0
        error_count = 0
        max_error_count = 10

        while not rospy.is_shutdown() and self.is_streaming:
            try:
                poller = select.poll()
                poller.register(self.vd, select.POLLIN)
                events = poller.poll(1000) # 1 second timeout

                if not events:
                    rospy.logwarn_throttle(5.0,"Timeout waiting for frame buffer.")
                    error_count += 1
                    if error_count > max_error_count:
                         rospy.logerr("Too many consecutive timeouts. Assuming streaming stopped.")
                         self.is_streaming = False
                         break
                    continue

                for fd, flag in events:
                     if flag & (select.POLLERR | select.POLLHUP | select.POLLNVAL):
                          rospy.logerr(f"Device poll error! Flags: {flag}. Stopping stream.")
                          self.is_streaming = False
                          break
                if not self.is_streaming:
                     break

                # --- Dequeue Buffer ---
                buf = v4l2.v4l2_buffer()
                buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
                buf.memory = v4l2.V4L2_MEMORY_MMAP
                fcntl.ioctl(self.vd, v4l2.VIDIOC_DQBUF, buf)

                # --- Get Timestamp and Settings ---
                timestamp = rospy.Time.now()
                settings_msg = self.get_cam_settings(timestamp) # Pass timestamp

                if buf.flags & v4l2.V4L2_BUF_FLAG_ERROR:
                     rospy.logwarn(f"Dequeued buffer {buf.index} with error flag! Flags: 0x{buf.flags:X}")
                     error_count += 1
                     try:
                         fcntl.ioctl(self.vd, v4l2.VIDIOC_QBUF, buf) # Requeue errored buffer
                     except IOError as q_err:
                         rospy.logerr(f"Failed to re-queue buffer {buf.index} after error: {q_err}")
                         self.is_streaming = False
                     if error_count > max_error_count:
                          rospy.logerr("Too many consecutive buffer errors. Stopping stream.")
                          self.is_streaming = False
                     continue # Skip publishing for this errored frame

                error_count = 0

                # --- Publish Settings ---
                if settings_msg:
                    # Ensure the message header timestamp matches the image timestamp
                    settings_msg.header.stamp = timestamp
                    self.cam_settings_pub.publish(settings_msg)
                else:
                    rospy.logwarn_throttle(5.0, "Failed to get camera settings for frame, not publishing settings.")

                # --- Publish Image ---
                jpeg_data = self.buffers[buf.index][:buf.bytesused]
                if jpeg_data:
                    img_msg = CompressedImage()
                    img_msg.header.stamp = timestamp # Use the same timestamp
                    img_msg.header.seq = frame_count
                    img_msg.header.frame_id = self.frame_id
                    img_msg.format = "jpeg"
                    img_msg.data = jpeg_data
                    self.compressed_pub.publish(img_msg)
                else:
                     rospy.logwarn_throttle(1.0, f"Dequeued buffer {buf.index} has zero bytes used.")

                # --- Requeue Buffer ---
                fcntl.ioctl(self.vd, v4l2.VIDIOC_QBUF, buf)
                frame_count += 1

            except (IOError, OSError) as e:
                if e.errno == 11: # EAGAIN
                    rospy.logwarn_throttle(1.0, "VIDIOC_DQBUF returned EAGAIN.")
                    time.sleep(0.01)
                elif e.errno == 5: # EIO
                     rospy.logerr(f"V4L2 IO Error 5 (EIO) in capture loop. Device likely disconnected. Stopping.")
                     self.is_streaming = False
                else:
                    rospy.logerr(f"V4L2 IO Error {e.errno} in capture loop: {e}")
                    self.is_streaming = False
            except Exception as e:
                rospy.logerr(f"Unexpected error in capture loop: {e}", exc_info=True)
                self.is_streaming = False

        rospy.loginfo("Capture loop finished.")
        self.stop_streaming()

    def set_initial_exposure_mode_from_param(self):
        """Sets the initial auto exposure mode based on ROS param."""
        auto_expo_ctrl_id = self.supported_controls.get('_v4l2_exposure_auto_id')
        if auto_expo_ctrl_id:
            target_v4l2_mode = self.initial_auto_exposure_mode
            valid_modes = [V4L2_EXPOSURE_MANUAL_MODE_VALUE, V4L2_EXPOSURE_AUTO_MODE_VALUE]
            if target_v4l2_mode not in valid_modes:
                 rospy.logwarn(f"Invalid value '{target_v4l2_mode}' for ~video/auto_exposure. Defaulting to Auto.")
                 target_v4l2_mode = V4L2_EXPOSURE_AUTO_MODE_VALUE

            mode_str = "Manual" if target_v4l2_mode == V4L2_EXPOSURE_MANUAL_MODE_VALUE else "Auto"
            current_val = self.get_v4l2_control(auto_expo_ctrl_id)
            if current_val is None:
                 rospy.logwarn("Could not read current exposure mode during initial setup. Attempting set.")
                 self.set_v4l2_control(auto_expo_ctrl_id, target_v4l2_mode)
                 rospy.sleep(0.05)
            elif current_val != target_v4l2_mode:
                 rospy.loginfo(f"Setting initial exposure mode to {mode_str} (V4L2 Mode: {target_v4l2_mode})")
                 self.set_v4l2_control(auto_expo_ctrl_id, target_v4l2_mode)
                 rospy.sleep(0.05)
            else:
                 rospy.loginfo(f"Initial exposure mode already set to {mode_str} (V4L2 Mode: {current_val})")
        else:
            rospy.logwarn("Camera does not support exposure_auto control. Cannot set initial mode.")

    def set_initial_exposure_time_from_param(self):
        """Sets the initial manual exposure time based on ROS param."""
        exposure_ctrl_id = self.supported_controls.get('exposure')
        auto_expo_ctrl_id = self.supported_controls.get('_v4l2_exposure_auto_id')

        if exposure_ctrl_id and auto_expo_ctrl_id:
            # Check the *intended* initial mode, not necessarily the current one if setting failed
            if self.initial_auto_exposure_mode == V4L2_EXPOSURE_MANUAL_MODE_VALUE:
                target_exposure_time = self.initial_exposure_time
                rospy.loginfo(f"Setting initial manual exposure time to {target_exposure_time}")
                self.set_v4l2_control(exposure_ctrl_id, target_exposure_time)
            else:
                rospy.loginfo("Skipping setting initial manual exposure time (initial mode is not manual).")
        elif not exposure_ctrl_id:
             rospy.logwarn("Camera does not support exposure control. Cannot set initial time.")

    def get_cam_settings(self, timestamp):
        """Reads current settings from the camera and returns a CamParameters message."""
        if not self.vd or self.vd.closed:
            rospy.logwarn_throttle(5.0, "Cannot get camera settings, device not open.")
            return None
        if not self.is_streaming:
             rospy.logwarn_throttle(5.0, "Cannot get camera settings, streaming is not active.")
             return None

        settings = CamParameters()
        settings.header.stamp = timestamp
        settings.header.frame_id = self.frame_id

        # Read values for controls supported by *this node* that map to msg fields
        # Get auto exposure mode first, as it affects how we interpret/report exposure
        auto_expo_ctrl_id = self.supported_controls.get('_v4l2_exposure_auto_id')
        current_auto_mode = None
        if auto_expo_ctrl_id:
            current_auto_mode = self.get_v4l2_control(auto_expo_ctrl_id)
            if current_auto_mode is not None and hasattr(settings, 'auto_exposure'):
                 setattr(settings, 'auto_exposure', int(current_auto_mode))
            elif hasattr(settings, 'auto_exposure'):
                 # Default if read fails? Or leave unset? Let's default to Auto
                 setattr(settings, 'auto_exposure', V4L2_EXPOSURE_AUTO_MODE_VALUE)


        for name, ctrl_id in self.supported_controls.items():
            # Skip internal keys and the auto_exposure key we already handled
            if name.startswith('_') or name == 'auto_exposure':
                continue

            # Check if the CamParameters message has this field
            if hasattr(settings, name):
                value = self.get_v4l2_control(ctrl_id)
                if value is not None:
                    # Handle manual exposure - only meaningful if in manual mode
                    if name == 'exposure':
                         if current_auto_mode == V4L2_EXPOSURE_MANUAL_MODE_VALUE:
                              setattr(settings, name, int(value))
                         else:
                              # Report 0 if not in manual mode, as the value is likely stale/invalid
                              setattr(settings, name, 0)
                    else: # Other standard controls
                         setattr(settings, name, int(value))
                else:
                     # Set to a default (e.g., 0) if read fails for standard controls
                     if name != 'exposure':
                          setattr(settings, name, 0)
                     elif name == 'exposure':
                          setattr(settings, name, 0) # Default to 0 if read fails

        # rospy.logdebug(f"get_cam_settings returning: {settings}") # Optional debug print
        return settings


    def cam_settings_callback(self, msg):
        """Callback for the ~set_cam_settings topic."""
        rospy.loginfo("Received camera settings via topic:")
        if not self.vd or self.vd.closed or not self.is_streaming:
            rospy.logwarn("Cannot set camera settings, device not ready or streaming.")
            return

        current_v4l2_mode = None
        auto_expo_ctrl_id = self.supported_controls.get('_v4l2_exposure_auto_id')
        if auto_expo_ctrl_id:
            # Get current mode before making changes
            current_v4l2_mode = self.get_v4l2_control(auto_expo_ctrl_id)
            # Handle mode change request first
            if hasattr(msg, 'auto_exposure'):
                target_v4l2_mode = msg.auto_exposure
                valid_modes = [V4L2_EXPOSURE_MANUAL_MODE_VALUE, V4L2_EXPOSURE_AUTO_MODE_VALUE]
                if target_v4l2_mode in valid_modes:
                    if current_v4l2_mode is not None and current_v4l2_mode != target_v4l2_mode:
                        mode_str = "Manual" if target_v4l2_mode == V4L2_EXPOSURE_MANUAL_MODE_VALUE else "Auto"
                        rospy.loginfo(f"  Topic: Setting exposure mode to {mode_str} (V4L2 Mode: {target_v4l2_mode})")
                        success = self.set_v4l2_control(auto_expo_ctrl_id, target_v4l2_mode)
                        if success:
                            rospy.sleep(0.05)
                            current_v4l2_mode = self.get_v4l2_control(auto_expo_ctrl_id) # Update current mode
                    elif current_v4l2_mode is None:
                         rospy.logwarn("  Topic: Could not read current exposure mode, attempting set anyway.")
                         self.set_v4l2_control(auto_expo_ctrl_id, target_v4l2_mode)
                         rospy.sleep(0.05)
                         current_v4l2_mode = self.get_v4l2_control(auto_expo_ctrl_id)
                else:
                    rospy.logwarn(f"  Topic: Received invalid auto_exposure value: {target_v4l2_mode}. Ignoring.")

        # Handle Other Controls
        for name, ctrl_id in self.supported_controls.items():
            if name.startswith('_') or name == 'auto_exposure':
                continue

            if hasattr(msg, name):
                value = getattr(msg, name)
                if name == 'exposure':
                    # Only attempt to set manual exposure if current mode is Manual
                    if current_v4l2_mode == V4L2_EXPOSURE_MANUAL_MODE_VALUE:
                        rospy.loginfo(f"  Topic: Setting {name} (0x{ctrl_id:X}) to {value}")
                        self.set_v4l2_control(ctrl_id, value)
                    else:
                        rospy.loginfo(f"  Topic: Skipping {name} setting, auto exposure mode is not Manual (current mode: {current_v4l2_mode})")
                else: # Handle other standard controls
                    rospy.loginfo(f"  Topic: Setting {name} (0x{ctrl_id:X}) to {value}")
                    self.set_v4l2_control(ctrl_id, value)

        # REMOVED: publish_current_settings call


    def dynamic_config_callback(self, config, level):
        """Callback for dynamic reconfigure server."""
        if not self.vd or self.vd.closed or not self.is_streaming:
            rospy.logwarn("Cannot set camera settings via dynamic reconfigure, device not ready or streaming.")
            return config

        current_v4l2_mode = None
        auto_expo_ctrl_id = self.supported_controls.get('_v4l2_exposure_auto_id')
        if auto_expo_ctrl_id:
            # Get current mode before potential changes
            current_v4l2_mode = self.get_v4l2_control(auto_expo_ctrl_id)
            # Handle mode change request first
            if 'auto_exposure_mode' in config:
                target_is_auto = config.auto_exposure_mode
                target_v4l2_mode = V4L2_EXPOSURE_AUTO_MODE_VALUE if target_is_auto else V4L2_EXPOSURE_MANUAL_MODE_VALUE
                mode_str = "Auto" if target_is_auto else "Manual"

                if current_v4l2_mode is not None and current_v4l2_mode != target_v4l2_mode:
                    rospy.logdebug(f"  DynConf: Setting exposure mode to {mode_str} (V4L2: {target_v4l2_mode})")
                    success = self.set_v4l2_control(auto_expo_ctrl_id, target_v4l2_mode)
                    if success:
                        rospy.sleep(0.05)
                        current_v4l2_mode = self.get_v4l2_control(auto_expo_ctrl_id) # Update current mode
                elif current_v4l2_mode is None:
                     rospy.logwarn("  DynConf: Could not read current exposure mode, attempting set anyway.")
                     self.set_v4l2_control(auto_expo_ctrl_id, target_v4l2_mode)
                     rospy.sleep(0.05)
                     current_v4l2_mode = self.get_v4l2_control(auto_expo_ctrl_id)

        # Handle Other Controls from config object
        for name in config.keys():
            if name == 'auto_exposure_mode':
                continue

            ctrl_id = self.supported_controls.get(name)
            if ctrl_id is None:
                continue

            value = config[name]

            if name == 'exposure':
                 # Only attempt to set manual exposure if current mode is Manual
                 if current_v4l2_mode == V4L2_EXPOSURE_MANUAL_MODE_VALUE:
                      current_exposure_val = self.get_v4l2_control(ctrl_id)
                      if current_exposure_val is not None and current_exposure_val != int(value):
                           rospy.logdebug(f"  DynConf: Setting {name} (0x{ctrl_id:X}) to {value}")
                           self.set_v4l2_control(ctrl_id, value)
                      elif current_exposure_val is None:
                           rospy.logwarn(f"  DynConf: Could not read current {name}, attempting set anyway.")
                           self.set_v4l2_control(ctrl_id, value)
                 else:
                      rospy.logdebug(f"  DynConf: Skipping {name} setting, auto exposure mode is not Manual (current mode: {current_v4l2_mode})")
            else: # Handle other controls normally
                 current_val = self.get_v4l2_control(ctrl_id)
                 if current_val is not None and current_val != int(value):
                      rospy.logdebug(f"  DynConf: Setting {name} (0x{ctrl_id:X}) to {value}")
                      self.set_v4l2_control(ctrl_id, value)
                 elif current_val is None:
                      rospy.logwarn(f"  DynConf: Could not read current {name}, attempting set anyway.")
                      self.set_v4l2_control(ctrl_id, value)

        # Read back actual values and update config object
        rospy.logdebug("Reading back values for dynamic reconfigure GUI update...")
        if auto_expo_ctrl_id and 'auto_exposure_mode' in config:
             final_v4l2_mode = self.get_v4l2_control(auto_expo_ctrl_id)
             if final_v4l2_mode is not None:
                  config.auto_exposure_mode = (final_v4l2_mode == V4L2_EXPOSURE_AUTO_MODE_VALUE)
             else:
                  rospy.logwarn("  DynConf Readback: Failed to read back auto_exposure_mode state.")

        for name in config.keys():
            if name == 'auto_exposure_mode':
                continue
            ctrl_id = self.supported_controls.get(name)
            if ctrl_id is None:
                continue

            read_value = self.get_v4l2_control(ctrl_id)
            if read_value is not None:
                 # Update config if read value differs, EXCEPT for exposure if in auto mode
                 # (keep the user's desired manual value in the GUI)
                 final_mode_for_exposure_check = self.get_v4l2_control(auto_expo_ctrl_id)
                 if name == 'exposure' and final_mode_for_exposure_check == V4L2_EXPOSURE_AUTO_MODE_VALUE:
                      pass # Don't update config.exposure from readback in auto mode
                 elif int(read_value) != config[name]:
                      config[name] = int(read_value)
            else:
                 # Don't warn if reading exposure failed while in auto mode
                 final_mode_for_exposure_check = self.get_v4l2_control(auto_expo_ctrl_id)
                 if not (name == 'exposure' and final_mode_for_exposure_check == V4L2_EXPOSURE_AUTO_MODE_VALUE):
                      rospy.logwarn(f"  DynConf Readback: Failed to read back {name} (0x{ctrl_id:X})")

        # REMOVED: publish_current_settings call

        return config

    def stop_streaming(self):
        """Stops the V4L2 streaming."""
        if self.is_streaming:
            rospy.loginfo("Stopping V4L2 streaming...")
            try:
                buf_type = v4l2.v4l2_buf_type(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
                fcntl.ioctl(self.vd, v4l2.VIDIOC_STREAMOFF, buf_type)
                self.is_streaming = False
                rospy.loginfo("Streaming stopped.")
            except (IOError, OSError, AttributeError) as e:
                rospy.logerr(f"Error stopping stream: {e}")


    def shutdown(self):
        """Cleans up resources."""
        rospy.loginfo("Shutting down V4L2 Camera Node...")
        # REMOVED: Timer shutdown logic

        self.stop_streaming()

        if hasattr(self, 'capture_thread') and self.capture_thread.is_alive():
             rospy.loginfo("Waiting for capture thread to exit...")
             self.capture_thread.join(timeout=1.0)
             if self.capture_thread.is_alive():
                  rospy.logwarn("Capture thread did not exit cleanly.")

        if self.buffers:
            rospy.loginfo("Unmapping buffers...")
            for mm in self.buffers:
                try:
                    mm.close()
                except Exception as e:
                    rospy.logwarn(f"Error closing memory map: {e}")
            self.buffers = []
            rospy.loginfo("Buffers unmapped.")

        if self.vd and not self.vd.closed:
            rospy.loginfo("Closing V4L2 device...")
            self.vd.close()
            self.vd = None
            rospy.loginfo("Device closed.")
        rospy.loginfo("Shutdown complete.")


if __name__ == '__main__':
    try:
        node = V4L2CameraNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt received. Shutting down.")
    except Exception as e:
        rospy.logfatal(f"Unhandled exception in main: {e}", exc_info=True)
    finally:
        pass # Shutdown handled by rospy.on_shutdown hook