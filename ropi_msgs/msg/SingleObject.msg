# State enum
uint8 OBJECT_DETECTED=0
uint8 OBJECT_MOVED=1
uint8 OBJECT_LOST=2

string id
uint8 state
float64 elapsed_time
geometry_msgs/Point position
geometry_msgs/Point position_prev

float32 diameter
float32 angle
float32 height