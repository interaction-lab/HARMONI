# Top left of image is (0,0), bottom right is max
# Width (x) and height (y) of input image.
uint32  width
uint32  height

# Track ID, usually one ID per face
uint8   id
uint32  center_x
uint32  center_y
# The rectangle around the face
uint32  topleft_x
uint32  topleft_y
uint32  botright_x
uint32  botright_y
float64 confidence
