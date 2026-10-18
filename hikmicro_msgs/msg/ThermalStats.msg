# Temperatures from one pixel-to-pixel thermometry frame, degrees Celsius.
# Pixel coordinates are in the temperature matrix (width x height).
Header header
uint8 POINT = 0
uint8 REGION = 1
uint8 FRAME = 2
uint8 mode
uint32 width
uint32 height
uint32 x1          # box actually used (inclusive, clamped to the frame)
uint32 y1
uint32 x2
uint32 y2
float32 max
float32 min
float32 mean
uint32 max_x
uint32 max_y
uint32 min_x
uint32 min_y
