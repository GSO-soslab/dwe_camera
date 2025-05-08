std_msgs/Header header                      #header
int8 brightness                             #[-64, 64, 0]
uint8 contrast                              #[0, 64, 32]
uint8 saturation                            #[0, 128, 64]
int8 hue                                    #[-40, 40, 0]
uint16 gamma                                #[72, 500, 100]
uint8 gain                                  #[0, 100, 0]
uint8 sharpness                             #[0, 6, 3]
uint16 exposure                             #[1, 5000, 156]

# Not Supported in OpenCV
#std_msgs/Bool white_balance_automatic       #[0, 1]
#uint8 power_line_frequency                  #[0, 2, 2]
#uint16 white_balance_temperature            #[2800, 6500, 4600]
#uint8 backlight_compensation                #[0, 2, 1]
#std_msgs/Bool exposure_dynamic_framerate    #[0, 1, 0]

# Not Used
uint8 auto_exposure                         #[0, 3, 3] (1-manual, 3-auto)
