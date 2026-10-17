# PTZ position as the camera reports it, polled at state_rate.
# pan: azimuth 0..360 deg, tilt: elevation in deg, both with the device's own signs;
# zoom: ratio (1.0 = 1x) of the visible lens.
Header header
bool connected
bool high_precision   # read through absoluteEx (0.001 deg), else /status (0.1 deg)
bool moving           # changed since the previous poll
float64 pan
float64 tilt
float64 zoom
