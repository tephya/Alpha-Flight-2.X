/**
 * @file    alg_mixer.c
 * @brief   四旋翼电机混控及输出去饱和实现。
 */

#include "alg_mixer.h"

/**
 * @brief   将浮点输出限制到制定范围并转换为 uint16_t。
 */
static uint16_t clamp(float val, float min, float max)
{
    if(val < min)
        return (uint16_t)min;
    if(val > max)
        return (uint16_t)max;
    return (uint16_t)val;
}

void Mixer(float roll_cmd,
           float pitch_cmd,
           float yaw_cmd,
           uint16_t throttle,
           uint8_t airmode_enabled,
           uint16_t *m1,
           uint16_t *m2,
           uint16_t *m3,
           uint16_t *m4)
{
    // 将 Roll/Pitch/Yaw 控制量转换为四路电机差动修正量。
    float correction[4] ={
        -pitch_cmd - roll_cmd + yaw_cmd,
        pitch_cmd - roll_cmd - yaw_cmd,
        -pitch_cmd + roll_cmd - yaw_cmd,
        pitch_cmd + roll_cmd + yaw_cmd
    };

    float correction_min = correction[0];
    float correction_max = correction[0];

    for (uint8_t i = 1U; i < 4U; i++)
    {
        if(correction[i] < correction_min)
            correction_min = correction[i];
        if(correction[i] > correction_max)
            correction_max = correction[i];
    }

    /**
     * 若四路姿态修正量的跨度超过可用电机输出范围，则按同一比例缩放。
     * 这样可以保持各轴控制量的方向及相对比例，避免单独裁剪某一路导致混控关系失真。
     */
    const float output_range = MIXER_OUTPUT_LIMIT - MIXER_OUTPUT_IDLE;
    const float correction_range = correction_max - correction_min;

    if(correction_range > output_range)
    {
        const float scale = output_range / correction_range;

        for (uint8_t i = 0U; i < 4U; i++)
            correction[i] *= scale;

        correction_min *= scale;
        correction_max *= scale;
    }

    // collective 为四路电机共享的基础输出，姿态修正量叠加在其上。
    float collective = (float)throttle;

    if(airmode_enabled)
    {
        /**
         * Airmode 允许整体平移 collective，使所有电机输出保持在有效范围内，
         * 从而尽可能完整保留姿态控制产生的电机差动。
         */
        const float collective_min = MIXER_OUTPUT_IDLE - correction_min;
        const float collective_max = MIXER_OUTPUT_LIMIT - correction_max;

        if (collective < collective_min)
            collective = collective_min;
        if (collective > collective_max)
            collective = collective_max;
    }
    else
    {
        /**
         * Airmode 关闭时，姿态修正不能主动抬高基础油门。
         * 低油门仅维持 Idle，单路越界由最终输出限幅处理。
         */
        if(collective < MIXER_OUTPUT_IDLE)
            collective = MIXER_OUTPUT_IDLE;
        if(collective > MIXER_OUTPUT_LIMIT)
            collective = MIXER_OUTPUT_LIMIT;
    }

    *m1 = clamp(collective + correction[0], MIXER_OUTPUT_IDLE, MIXER_OUTPUT_LIMIT);
    *m2 = clamp(collective + correction[1], MIXER_OUTPUT_IDLE, MIXER_OUTPUT_LIMIT);
    *m3 = clamp(collective + correction[2], MIXER_OUTPUT_IDLE, MIXER_OUTPUT_LIMIT);
    *m4 = clamp(collective + correction[3], MIXER_OUTPUT_IDLE, MIXER_OUTPUT_LIMIT);
}
