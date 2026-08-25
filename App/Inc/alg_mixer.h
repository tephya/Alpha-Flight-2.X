/**
 * @file    alg_mixer.h
 * @brief   四旋翼电机混控器接口。
 */

#ifndef __ALG_MIXER_H
#define __ALG_MIXER_H

#include <stdint.h>

/**
 * 单电机混控输出上限。
 * 高于 ALG_THROTTLE_MAX，以保留姿态控制量叠加后的调整裕量。
 */
#define MIXER_OUTPUT_LIMIT 1352.0f

/** 电机最低运行输出，用于 Armed 状态下维持怠速。 */
#define MIXER_OUTPUT_IDLE 110U

/**
 * @brief   将油门及姿态控制量混合为四路电机输出。
 * 
 * 根据 Roll/Pitch/Yaw 控制指令和基础油门计算四个电机的最终输出。
 * Airmode 启用时允许在低油门区继续保留姿态控制能力。
 * 
 * @param[in]   roll_cmd    Roll 控制量。
 * @param[in]   pitch_cmd   Pitch 控制量。
 * @param[in]   yaw_cmd     Yaw 控制量。
 * @param[in]   throttle    基础油门控制量。
 * @param[in]   airmode_enabled 0表示关闭Airmode，非0表示启用。
 * @param[out]  m1  Motor 1 输出。
 * @param[out]  m2  Motor 2 输出。
 * @param[out]  m3  Motor 3 输出。
 * @param[out]  m4  Motor 4 输出。
 */
void Mixer(float roll_cmd,
           float pitch_cmd,
           float yaw_cmd,
           uint16_t throttle,
           uint8_t airmode_enabled,
           uint16_t *m1,
           uint16_t *m2,
           uint16_t *m3,
           uint16_t *m4);

#endif
