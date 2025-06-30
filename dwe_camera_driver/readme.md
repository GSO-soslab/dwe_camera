# dwe_camera_driver/readme.md
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
    
    If realtime apriltag detection is desired
    ```bash   
    pip3 install pupil-apriltags
    ```

    **System Libraries (apt)**
    ```bash
    sudo apt install v4l-utils python3-opencv python3-numpy python3-scipy \
    ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-compressed-image-transport
    ```

- Raspberry Pi also requires:
    ```bash
    sudo apt install libgl1
    ```

### How to run

- Single camera
    ```bash
    ros2 launch dwe_camera_driver camera.launch.py
    ```
- Single camera with realtime apriltag detection
    ```bash
    ros2 launch dwe_camera_driver apriltag_camera.launch.py
    ```
- Dual camera
    ```bash
    ros2 launch dwe_camera_driver multi_camera.launch.py
    ```