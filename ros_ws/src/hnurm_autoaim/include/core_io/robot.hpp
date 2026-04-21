#pragma once
#include <cstdint>
enum class ShootMode { IDLE = 0, TRACKING = 1, SHOOT_NOW = 2 };
namespace ProgramMode { enum { AUTOAIM = 0, MANUAL = 1, ENERGY_HIT = 2, NOT_RECEIVED = 3 }; }
namespace EnemyColor { enum { BLUE = 0, RED = 1 }; }
enum class ArmorColor { BLUE = 0, RED = 1, NONE = 2, GRAY = 3 };
struct RobotCmd {
    float yaw = 0.0f; float pitch = 0.0f; float dist = 0.0f;
    ShootMode shoot = ShootMode::IDLE;
    uint16_t detection_info = 0; int car_id = 0; int aim_id = 0;
    float yaw_v = 0.0f; float pitch_v = 0.0f;
};
struct RobotStatus {
    int enemy_color = 0; int program_mode = 0;
    float yaw_compensate = 0.0f; float pitch_compensate = 0.0f;
    float bullet_speed = 28.0f; int last_shoot_aim_id = 0;
};
