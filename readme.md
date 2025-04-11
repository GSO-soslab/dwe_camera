### Some Useful Commands

- rqt dynamic reconfigure gui
    ```
    rosrun rqt_gui rqt_gui -s reconfig
    ```

- echo clear terminal
    ```
    rostopic echo -c /topic_name
    ```
- get usb camera device id
    ```
    v4l2-ctl --list-devices
    ```


### dependency
```
pip3 install cv_bridge
pip3 install opencv-python
pip3 install v4l2-python3
```
- pi4
  ```
  sudo apt-get install libgl1-mesa-glx
  ```

```
sudo apt install v4l-utils
```
