#include "alg_mixer.h"

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
           uint16_t *m1,
           uint16_t *m2,
           uint16_t *m3,
           uint16_t *m4)
{
    float correction[4] ={
        -pitch_cmd - roll_cmd - yaw_cmd,
        pitch_cmd - roll_cmd + yaw_cmd,
        -pitch_cmd + roll_cmd + yaw_cmd,
        pitch_cmd + roll_cmd - yaw_cmd
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

    /* 如果所需的扭矩范围无法在物理上容纳于完整的电机行程范围内，
     * 则同时缩放每个轴，以保持其方向和相对贡献 */
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

    /* Airmode去饱和：在不改变电机间差异的情况下移动总推力。
     * 这可以在低油门时保持姿态控制，同时使每个电机都保持在可用输出范围内 */
    float collective = (float)throttle;
    const float collective_min = MIXER_OUTPUT_IDLE - correction_min;
    const float collective_max = MIXER_OUTPUT_LIMIT - correction_max;

    if(collective < collective_min)
        collective = collective_min;
    if(collective > collective_max)
        collective = collective_max;

    *m1 = clamp(collective + correction[0], MIXER_OUTPUT_IDLE, MIXER_OUTPUT_LIMIT);
    *m2 = clamp(collective + correction[1], MIXER_OUTPUT_IDLE, MIXER_OUTPUT_LIMIT);
    *m3 = clamp(collective + correction[2], MIXER_OUTPUT_IDLE, MIXER_OUTPUT_LIMIT);
    *m4 = clamp(collective + correction[3], MIXER_OUTPUT_IDLE, MIXER_OUTPUT_LIMIT);
}
