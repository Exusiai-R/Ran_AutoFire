std_msgs/Header header

TargetState target_state
TargetType target_type

float32 pitch
float32 yaw
float32 delta_pitch
float32 delta_yaw
float32 target_distance
float32 original_yout

float32 vel_x
float32 vel_y
float32 vel_yaw

# only compared to zero to determine the controller
# control_id > 0 : right
# control_id < 0 : left
float32 control_id