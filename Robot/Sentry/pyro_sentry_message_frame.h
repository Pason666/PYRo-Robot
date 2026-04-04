#ifndef __PYRO_SENTRY_MESSAGE_FRAME_H__
#define __PYRO_SENTRY_MESSAGE_FRAME_H__ 

#include <cstdint>

// -------------------- 框架结构 -------------------
struct frame_header
{
    uint8_t sof{};
} __attribute__((packed));

struct frame_tailer
{
    uint16_t crc16{};
} __attribute__((packed));

struct frame_enter
{
    uint8_t enter{};
} __attribute__((packed));

// -------------------- 消息帧结构 -------------------
typedef struct
{
    float vx{};
    float vy{};
    float vz{};
    float wz{};
    uint8_t stuck{};
    float yaw{};
    uint8_t in_aim{};
    uint8_t mode{};
    bool scan{};
} __attribute__((packed)) nav2mcu_data_t;

typedef struct
{
    float shoot_yaw{};
    float shoot_yaw_speed{};
    float shoot_yaw_acceleration{};
    float shoot_pitch{};
    float shoot_pitch_speed{};
    float shoot_pitch_acceleration{};
    uint8_t fire           : 1;
    uint8_t is_single_shot : 1;
    uint8_t target_id      : 6;
    uint8_t aim_state;
} __attribute__((packed)) aim2mcu_data_t;

typedef struct
{
    float curr_yaw{};
    float curr_pitch{};
    float self_v_magnitude{};
    float self_v_angle{};
    float curr_speed{};
    uint8_t shoot_delay{};
    uint8_t state       : 5;
    uint8_t stop_record : 1;
    uint8_t autoaim     : 1;
    uint8_t enemy_color : 1;
} __attribute__((packed)) mcu2aim_data_t;

typedef struct
{
    uint16_t self_hp{};
    uint16_t self_ammo{};
    uint8_t game_state{};
    uint16_t self_base_hp{};
    uint16_t self_outpost_hp{};
    uint16_t game_time{};
} __attribute__((packed)) mcu2nav_data_t;

// -------------------- 帧结构定义 -------------------
typedef struct
{
    frame_header header{};
    nav2mcu_data_t data{};
    frame_tailer tailer{};
} __attribute__((packed)) nav2mcu_msg_t;

typedef struct
{
    frame_header header{};
    aim2mcu_data_t data{};
    frame_tailer tailer{};
} __attribute__((packed)) aim2mcu_msg_t;

typedef struct
{
    frame_header header{};
    mcu2aim_data_t data{};
    frame_tailer tailer{};
    frame_enter enter{};
} __attribute__((packed)) mcu2aim_msg_t;

typedef struct
{
    frame_header header{};
    mcu2nav_data_t data{};
    frame_tailer tailer{};
} __attribute__((packed)) mcu2nav_msg_t;

// -------------------- 裁判系统 -------------------
struct sentry_cmd_t
{
    // 复活相关指令
    uint32_t confirm_resurrection       : 1;  // bit 0: 是否确认复活 (0: 确认不复活，1: 确认复活)
    uint32_t confirm_buy_revive         : 1;  // bit 1: 是否确认兑换立即复活 (0: 不兑换，1: 确认消耗金币兑换)

    // 兑换发弹量相关指令
    uint32_t buy_projectile_allowance   : 11; // bit 2-12: 将要兑换的发弹量值（需单调递增）

    // 远程请求相关指令
    uint32_t remote_buy_projectile_times: 4;  // bit 13-16: 远程兑换发弹量的请求次数（需单调递增且每次仅能增加1）
    uint32_t remote_buy_hp_times        : 4;  // bit 17-20: 远程兑换血量的请求次数（需单调递增且每次仅能增加1）

    // 姿态与机制激活指令
    uint32_t sentry_posture             : 2;  // bit 21-22: 修改当前姿态指令 (1: 进攻, 2: 防御, 3: 移动, 默认为3)
    uint32_t confirm_activate_rune      : 1;  // bit 23: 是否确认使能量机关进入正在激活状态 (1为确认，默认为0)

    // 保留位
    uint32_t reserved                   : 8;  // bit 24-31: 保留位
}; // 替代原 uint32_t sentry_cmd

#endif
