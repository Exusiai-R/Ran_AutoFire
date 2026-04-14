std_msgs/Header header

# --- 1. 绝对目标指向 ---
float32 target_yaw     # 预测并经过下坠补偿后的目标绝对 Yaw
float32 target_pitch   # 预测并经过下坠补偿后的目标绝对 Pitch

# --- 2. LMTD Water-gun 火控体系 ---
uint8 shoot_flag       # 0: 丢失目标/不作为, 1: 瞄准跟随中, 2: 极佳击穿角(允许开火)
uint16 seq_id          # 自增序号，电控收到后原样打回，用于测算全链路延迟