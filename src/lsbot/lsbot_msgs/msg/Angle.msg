# Reported angle by the Floorscan sensor

# Possible Status States
int16 UNSTABLE = 0      # not yet initialized
int16 STABLE = 1        # Initialized and working
int16 UNDEFINED = 2     # Undefined
int16 RECALIBRATE = -10 # Special angle value to recalibrate
float64 MIN_VALUE = 0.     # Minimum Value
float64 MAX_VALUE = 1.57079632679489661923 # Maximum Value

std_msgs/Header header
int16 status        # Current Status based on the Status States
float64 angle         # Angle in radians