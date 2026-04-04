#include "pyro_hybrid_chassis.h"

namespace pyro
{
bool flag;
void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::leg_retraction_state_t::
    enter(owner *owner)
{
    // owner->_ctx.data.target_pitch_rad = -0.1f; // 设定一个略微向前倾的目标姿态，帮助腿部收回时保持稳定
    // flag = false;
}

void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::leg_retraction_state_t::
    execute(owner *owner)
{

    // if (flag)
    // {
    //     // 1. 轮腿 VMC 姿态维稳控制 (维持 pitch/roll 平衡)
        owner->_ctx.data.target_leg_rad[0] = LEG_LENGTH_MIN_POS + LEG_LENGTH_POS_BUFFER_RAD;
        owner->_ctx.data.target_leg_rad[1] = LEG_LENGTH_MIN_POS + LEG_LENGTH_POS_BUFFER_RAD;
        owner->_leg_length_control();
        owner->_send_motor_command();
    // }
    // else
    // {
    //     owner->_leg_vmc();
    //     owner->_send_motor_command();
    // }
    //
    // if (owner->_ctx.data.current_pitch_rad < -0.01f) // 当检测到机器人已经有明显的前倾时，开始收腿
    // {
    //     flag = true;
    // }

}

void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::leg_retraction_state_t::
    exit(owner *owner)
{
    // owner->_ctx.data.target_pitch_rad = 0.04f; // 退出收腿状态后恢复正常的目标姿态
}


} // namespace pyro