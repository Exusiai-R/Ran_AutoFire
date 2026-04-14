std_msgs/Header header

# --- 1. 核心姿态数据 (陀螺仪绝对角度) ---
float32 pitch
float32 yaw
float32 roll

# --- 2. 弹道解算依赖 ---
float32 bullet_speed   # 当前实际弹速

# --- 3. 状态与上下文 ---
uint8 enemy_color      
uint8 program_mode     

# --- 4. 延迟闭环反馈 (新增) ---
uint16 ack_seq_id      # 电控回传的序列号 (Echo ID)
uint8  shoot_feedback  # 击发确认：0(未击发), 1(成功击发本帧指令)

# --- 5. 预留与扩展 ---
float32 vel_x
float32 vel_y
float32 vel_z