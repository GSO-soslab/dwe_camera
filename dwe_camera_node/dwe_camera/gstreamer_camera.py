import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import sys

# Define the constant manually as a workaround for environment/install issues.
GST_CLOCK_TIME_SECOND = 10**9

class GStreamerCamera:
    """
    A class to interface with a V4L2 camera using GStreamer to capture
    the raw, hardware-encoded JPEG stream directly and allow for dynamic control.
    """
    def __init__(self, device_id, width, height, framerate, logger):
        Gst.init(sys.argv if Gst.is_initialized() else None)
        self.logger = logger
        self.pipeline = None
        self.appsink = None
        self.videosrc = None # Handle to the v4l2src element

        self.pipeline_str = (
            f"v4l2src name=videosrc device=/dev/video{device_id} ! "
            f"image/jpeg,width={width},height={height},framerate={framerate}/1 ! "
            f"appsink name=sink emit-signals=True max-buffers=1 drop=True"
        )
        self.logger.info(f"Using GStreamer pipeline: {self.pipeline_str}")

        try:
            self.pipeline = Gst.parse_launch(self.pipeline_str)
        except GLib.Error as e:
            self.logger.error(f"Failed to create GStreamer pipeline: {e}")
            raise RuntimeError("GStreamer pipeline creation failed.") from e

        self.appsink = self.pipeline.get_by_name('sink')
        if not self.appsink:
            raise RuntimeError("Failed to get 'appsink' from GStreamer pipeline.")
        
        self.videosrc = self.pipeline.get_by_name('videosrc')
        if not self.videosrc:
            raise RuntimeError("Failed to get 'videosrc' from GStreamer pipeline.")

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self.on_error)
        bus.connect("message::warning", self.on_warning)

    def on_error(self, bus, msg):
        err, dbg = msg.parse_error()
        self.logger.error(f"GSTREAMER ERROR from element {msg.src.get_name()}: {err.message}")
        if dbg: self.logger.error(f"GSTREAMER DEBUG INFO: {dbg}")

    def on_warning(self, bus, msg):
        warn, dbg = msg.parse_warning()
        self.logger.warn(f"GSTREAMER WARNING from element {msg.src.get_name()}: {warn.message}")
        if dbg: self.logger.warn(f"GSTREAMER DEBUG INFO: {dbg}")

    def start(self):
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.logger.error("Unable to set the GStreamer pipeline to the playing state.")
            return False
        
        state_change_result, _, _ = self.pipeline.get_state(GST_CLOCK_TIME_SECOND)
        if state_change_result in (Gst.StateChangeReturn.SUCCESS, Gst.StateChangeReturn.ASYNC):
            self.logger.info("GStreamer pipeline started successfully.")
            return True
        else:
            self.logger.error(f"GStreamer pipeline failed to start. State change: {state_change_result}")
            return False

    def set_all_controls(self, controls):
        if not self.videosrc:
            self.logger.warn("Cannot set controls, v4l2src element not available.")
            return
        
        # Create a GStreamer Structure to hold the V4L2 controls.
        # This is the correct method for setting multiple controls atomically.
        gst_controls = Gst.Structure.new_empty("v4l2_extra_controls")
        for name, value in controls.items():
            # All V4L2 controls are integer-based (including enums/menus).
            gst_controls.set_value(name, int(value))
            
        try:
            self.videosrc.set_property('extra-controls', gst_controls)
            # FIX 1: Changed log level to INFO and format for better diagnostics.
            # This will now print a detailed string confirming the exact controls
            # being sent to the GStreamer element.
            self.logger.info(f"Applied GStreamer extra-controls: {gst_controls.to_string()}")
        except GLib.Error as e:
            self.logger.warn(f"Failed to set extra-controls property: {e}")

    def read_jpeg(self):
        """
        Blocks and waits for a new sample from the appsink.
        """
        # FIX 2: Use the direct pull_sample() method instead of emitting the
        # 'pull-sample' signal. This is the more robust and idiomatic way to
        # synchronously retrieve a sample from an appsink.
        sample = self.appsink.pull_sample()

        if sample is None:
            self.logger.debug("GStreamer pull_sample returned None (stream might be ending).")
            return None

        buf = sample.get_buffer()
        if buf is None: return None
        result, map_info = buf.map(Gst.MapFlags.READ)
        if not result: return None
        data = map_info.data[:]
        buf.unmap(map_info)
        return data

    def release(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.logger.info("GStreamer pipeline stopped and released.")
            self.pipeline = None

    def is_opened(self):
        if not self.pipeline: return False
        _, current_state, _ = self.pipeline.get_state(0)
        return current_state == Gst.State.PLAYING