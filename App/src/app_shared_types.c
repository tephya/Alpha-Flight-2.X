/**
 * @file    app_shared_types.c
 * @brief   应用层共享运行状态实例定义。
 */

#include "app_shared_types.h"

/*
 * IMU 冗余健康状态的启动默认值。
 *
 * ImuRedundancy_Init() 会根据实际 WHO_AM_I 初始化结果
 * 重新建立 Active IMU、Healthy 状态及 Dual Fault。
 */
volatile ImuHealthStatus_t g_imu_health = {
    .active_imu_sel = 0,
    .imu1_healthy = 1,
    .imu2_healthy = 1,
    .bad_frame_count = 0,
    .good_frame_count = 0,
    .dual_fault = 0};

/* 各关键 Task 启动后分别更新自己的 Heartbeat 时间戳。 */
volatile SystemHeartbeat_t g_heartbeat = {0};

/*
 * 电源状态启动默认值。
 *
 * Current Limiter 默认允许 100% Collective Throttle；
 * Voltage Health 的最终 Ready 状态由 PowerMonit Task 根据实际采样维护。
 */
volatile PowerHealth_t g_power_health = {
    .voltage_fault = false,
    .current_limiting = false,
    .current_limit_permille = 1000U,
    .current_filtered_a = 0.0f};
