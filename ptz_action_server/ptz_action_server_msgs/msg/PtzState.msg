# The device is not currently moving
int8 MODE_IDLE=0

# The device is moving under position control
int8 MODE_POSITION=1

# The device is moving under velocity control
int8 MODE_VELOCITY=2

# The device's current operating mode
int8 mode

# The current position or velocity of a PTZ camera or PTU
float32 pan
float32 tilt
float32 zoom
