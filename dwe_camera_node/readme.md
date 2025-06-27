# dwe_camera_node/readme.md
### Some Useful Commands

- rqt dynamic reconfigure gui
    ```bash
    ros2 run rqt_reconfigure rqt_reconfigure
    ```
- echo clear terminal
    ```bash
    ros2 topic echo -c /topic_name
    ```
- get usb camera device id
    ```bash
    v4l2-ctl --list-devices
    ```
- get camera supported formats and resolutions
    ```bash
    v4l2-ctl -d /dev/video{i} --list-formats-ext
    ```

### Dependencies
- All Ubuntu / Debian-based systems (including Raspberry Pi):
    
    **Python Packages (pip)**
    ```bash
    pip3 install opencv-python
    ```
    (Note: `cv_bridge` is a standard ROS package and should be installed via `rosdep` or `sudo apt install ros-<distro>-cv-bridge`, not pip).

    **System Libraries (apt)**
    These are required for camera interaction (v4l) and direct, efficient JPEG streaming (GStreamer).
    ```bash
    sudo apt install v4l-utils python3-gi gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0
    ```

- Raspberry Pi also requires:
    ```bash
    sudo apt install libgl1
    ```