add_library(Sentry OBJECT)

target_sources(Sentry PRIVATE

    ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_init_thread.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_mission_planer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_sentry_booster_app.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_sentry_chassis_app.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/application/pyro_sentry_gimbal_app.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/booster/pyro_17mm_booster.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_17mm_stop_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_17mm_fric_ready_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_17mm_ready_shoot_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_17mm_single_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_17mm_continue_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/booster/fsm/pyro_17mm_done_state.cpp
    
    ${CMAKE_CURRENT_SOURCE_DIR}/chassis/pyro_rud_chassis.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/chassis/fsm/pyro_rud_active_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/chassis/fsm/pyro_rud_passive_state.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/pyro_sentry_gimbal.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/fsm/pyro_sentry_gimbal_active_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/fsm/pyro_sentry_gimbal_manual_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/fsm/pyro_sentry_gimbal_passive_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/fsm/pyro_sentry_gimbal_scanning_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gimbal/fsm/pyro_sentry_gimbal_tracking_state.cpp
    
    ${CMAKE_CURRENT_SOURCE_DIR}/yaw/pyro_yaw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/yaw/fsm/pyro_yaw_active_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/yaw/fsm/pyro_yaw_passive_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/yaw/fsm/pyro_yaw_scanning_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/yaw/fsm/pyro_yaw_manual_state.cpp

)
target_include_directories(Sentry PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/booster
    ${CMAKE_CURRENT_SOURCE_DIR}/gimbal
    ${CMAKE_CURRENT_SOURCE_DIR}/chassis
    ${CMAKE_CURRENT_SOURCE_DIR}/yaw
    ${CMAKE_CURRENT_SOURCE_DIR}/application
)
target_link_libraries(Sentry PUBLIC stm32cubemx dsppp_lib PYRo)