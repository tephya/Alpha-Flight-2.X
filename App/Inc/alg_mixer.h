#ifndef __ALG_MIXER_H
#define __ALG_MIXER_H

#include <stdint.h>

/* 混控器单电机输出上限，刻意大于ALG_THROTTLE_MAX(1152)，
 * 目的是给姿态控制留出比油门量程更大的调整裕量*/
#define MIXER_OUTPUT_LIMIT 1352.0f

#define MIXER_OUTPUT_IDLE 110U

void Mixer(float roll_cmd,
           float pitch_cmd,
           float yaw_cmd,
           uint16_t throttle,
           uint16_t *m1,
           uint16_t *m2,
           uint16_t *m3,
           uint16_t *m4);

#endif
