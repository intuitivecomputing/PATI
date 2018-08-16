# State enum
uint8 CURSOR_PRESSED=0
uint8 CURSOR_DRAGGED=1
uint8 CURSOR_RELEASED=2

uint8 id
uint8 state
geometry_msgs/Point cursor
geometry_msgs/Point cursor_prev