std_msgs/Header header                      #header
bool auto_exposure                          # True if auto exposure is on, False for manual.
int8 brightness                             #[-64, 64, 0]
uint8 contrast                              #[0, 64, 32]
uint8 saturation                            #[0, 128, 64]
int8 hue                                    #[-40, 40, 0]
uint16 gamma                                #[72, 500, 100]
uint8 gain                                  #[0, 100, 0]
uint8 sharpness                             #[0, 6, 3]
uint16 exposure_time                        #[1, 5000, 156]
string video_format                         # Current video format (e.g., 'MJPG', 'YUYV')

bool white_balance_automatic                # True if auto white balance is on.
uint16 white_balance_temperature            # White balance temperature [2800, 6500]. Used when auto is False.
uint8 power_line_frequency                  # Power line frequency setting (0: Disabled, 1: 50Hz, 2: 60Hz)
uint8 backlight_compensation                # Backlight compensation value [0, 20]

# --- The following controls are often unsupported by the OpenCV V4L2 backend ---
# std_msgs/Bool exposure_dynamic_framerate    #[0, 1, 0]