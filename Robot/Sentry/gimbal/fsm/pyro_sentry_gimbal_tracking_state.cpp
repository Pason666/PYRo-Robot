#include "pyro_sentry_gimbal.h"

namespace pyro
{
void gimbal_t::fsm_active_t::state_tracking_t::enter(owner *owner)
{
    if (dm_motor_drv_t::ok !=
        owner->_ctx.gimbal_config.motor.pitch->get_error_code())
    {
        owner->_ctx.gimbal_config.motor.pitch->clear_error();
    }
    owner->_ctx.gimbal_config.motor.yaw->enable();
    owner->_ctx.gimbal_config.motor.pitch->enable();
}

void gimbal_t::fsm_active_t::state_tracking_t::execute(gimbal_t *owner)
{
    if (!owner->_ctx.cmd->is_aiming)
    {
        owner->_main_fsm.change_state(&owner->_active_state._scanning_state);
        return;
    }

     float yaw, pitch, roll;
    ins_drv_t *ins = ins_drv_t::get_instance();
    ins->get_rads_b(&yaw, &pitch, &roll);

    owner->_ctx.data.aim_imu_max_yaw =
        wrap_pi(yaw + (owner->_ctx.gimbal_config.yaw_max_rad -
                       owner->_ctx.data.current_yaw_rad));
    owner->_ctx.data.aim_imu_min_yaw =
        wrap_pi(yaw + (owner->_ctx.gimbal_config.yaw_min_rad -
                       owner->_ctx.data.current_yaw_rad));

    // ------------------没经过测试，但是我觉得这样就可以-----------------
    // if (owner->_ctx.data.aim_imu_max_yaw <
    // owner->_ctx.data.aim_imu_min_yaw)
    // {
    //     owner->_ctx.data.aim_imu_max_yaw += 2 * PI;
    // }

    owner->_ctx.data.target_pitch_rad = owner->_ctx.cmd->aim_imu_pitch_rad;
    owner->_ctx.data.target_yaw_rad   = owner->_ctx.cmd->aim_imu_yaw_rad;

    if (owner->_ctx.data.target_pitch_rad >
        owner->_ctx.gimbal_config.pitch_max_rad)
        owner->_ctx.data.target_pitch_rad =
            owner->_ctx.gimbal_config.pitch_max_rad;
    if (owner->_ctx.data.target_pitch_rad <
        owner->_ctx.gimbal_config.pitch_min_rad)
        owner->_ctx.data.target_pitch_rad =
            owner->_ctx.gimbal_config.pitch_min_rad;

    if (owner->_ctx.data.aim_imu_max_yaw > owner->_ctx.data.aim_imu_min_yaw)
    {
        owner->_ctx.data.target_yaw_rad = loop_fp32_constrain(
            owner->_ctx.data.target_yaw_rad, owner->_ctx.data.aim_imu_min_yaw,
            owner->_ctx.data.aim_imu_max_yaw);
    }
    else
    {
        if (owner->_ctx.data.target_yaw_rad >
                owner->_ctx.data.aim_imu_max_yaw &&
            owner->_ctx.data.target_yaw_rad < owner->_ctx.data.aim_imu_min_yaw)
        {
            float dist_to_max = fabs(wrap_pi(owner->_ctx.data.target_yaw_rad -
                                             owner->_ctx.data.aim_imu_max_yaw));
            float dist_to_min = fabs(wrap_pi(owner->_ctx.data.target_yaw_rad -
                                             owner->_ctx.data.aim_imu_min_yaw));
            owner->_ctx.data.target_yaw_rad =
                dist_to_max < dist_to_min ? owner->_ctx.data.aim_imu_max_yaw
                                          : owner->_ctx.data.aim_imu_min_yaw;
        }
    }
    _gimbal_imu_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void gimbal_t::fsm_active_t::state_tracking_t::exit(gimbal_t *owner)
{
}

}