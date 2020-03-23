# request AUs and/or viseme for the face
# multiple AUs possible, but only one viseme

#the numbers of the action units
string[] aus
#the degree to which to express the AU, in (0,1]
float64[] au_degrees
#the time to make the expression, in ms
int64 au_ms

#specify a side of the face to move (eye and eyebrow movements only)
int32 side
int32 BOTH=0
int32 RIGHT=1
int32 LEFT=2

#the name of the viseme
string[] visemes
#the time to make the viseme, in ms
int64 viseme_ms
#the times (from zero) at which to play the visemes
float32[] times
int64 start

#toggles idle gaze behavior
int32 hold_gaze
int32 IDLE_OFF=1
int32 IDLE_ON=2

#3D gaze target, relative to center of face
# if retarget_gaze is false, ignore gaze_target
bool retarget_gaze
geometry_msgs/Point gaze_target

# velocity to move gaze, in rad/s
float64 gaze_vel
