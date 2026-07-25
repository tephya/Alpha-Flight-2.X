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
