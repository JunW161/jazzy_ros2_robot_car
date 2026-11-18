std_msgs/Header header

uint8 IDLE=0
uint8 GOING_TO_PICKUP=1
uint8 WAITING_PICKUP=2
uint8 GOING_TO_DROPOFF=3
uint8 WAITING_DROPOFF=4
uint8 COMPLETED=5
uint8 CANCELED=6
uint8 ERROR=7

string mission_id
uint8 state
string state_name
string pickup_name
string dropoff_name
geometry_msgs/PoseStamped active_goal
float32 distance_remaining
uint16 nav_error_code
string message
