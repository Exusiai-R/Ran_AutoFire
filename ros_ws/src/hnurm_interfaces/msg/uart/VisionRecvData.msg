std_msgs/Header header

SelfColor self_color
WorkMode work_mode
BulletSpeed bullet_speed

float32 roll
float32 pitch
float32 yaw

float32 vel_x
float32 vel_y
float32 vel_yaw

# only compared to zero to determine the controller
# control_id > 0 : right
# control_id < 0 : left
float32 control_id

float32 game_progress 
float32 stage_remain_time
float32 current_hp
float32 current_enemy_sentry_hp
float32 current_enemy_base_hp
float32 current_virtual_shield    #百分比
float32 current_base_hp 



# float32 stage_remain_time = 0