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
This package now uses OpenCV's V4L2 backend to capture JPEG streams directly, removing the need for GStreamer.

- All Ubuntu / Debian-based systems (including Raspberry Pi):
    
    **Python Packages (pip)**
    The recommended way to install OpenCV for Python is via pip:
    ```bash
    pip3 install opencv-python
    pip3 insall empy
    pip3 insatll catkin_pkg
    pip3 install numpy
    pip3 install lark

    ```
    (Note: `cv_bridge` is a standard ROS package and should be installed via `rosdep` or `sudo apt install ros-<distro>-cv-bridge`, not pip).

    **System Libraries (apt)**
    ```bash
    sudo apt install v4l-utils
    sudo apt install ros-<distro>-cv-bridge
    sudo apt install ros-<distro>-compressed-image-transport
    ```

- Raspberry Pi also requires:
    ```bash
    sudo apt install libgl1
    ```