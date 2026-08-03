#include "app_shared_types.h"

volatile ImuHealthStatus_t g_imu_health = {
    .active_imu_sel = 0,
    .imu1_healthy = 1,
    .imu2_healthy = 1,
    .bad_frame_count = 0,
    .good_frame_count = 0,
    .dual_fault = 0
};

volatile SystemHeartbeat_t g_heartbeat = {0};
volatile PowerHealth_t g_power_health = {
    .voltage_fault = false,
    .current_limiting = false,
    .current_limit_permille = 1000U,
    .current_filtered_a = 0.0f
};
