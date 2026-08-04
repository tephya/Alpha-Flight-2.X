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
           uint8_t airmode_enabled,
           uint16_t *m1,
           uint16_t *m2,
           uint16_t *m3,
           uint16_t *m4)
{
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

    /* collective是四路电机共同的基础输出。
     * 最终每路输出 = collective + 对应姿态修正量。 */
    float collective = (float)throttle;

    if(airmode_enabled)
    {
        /* Airmode去饱和：
         * 允许整体移动collective，以完整保留电机之间的姿态差动。
         * 
         * 例如某路修正为-700，为保证该路不低于110
         * collective会被提高为810 */
        const float collective_min = MIXER_OUTPUT_IDLE - correction_min;
        const float collective_max = MIXER_OUTPUT_LIMIT - correction_max;

        if (collective < collective_min)
            collective = collective_min;
        if (collective > collective_max)
            collective = collective_max;
    }
    else
    {
        /* Airmode关闭时，不允许姿态修正主动抬高collective。
         * Armed低油门只维持电机idle；若修正量导致单路越界，
         * 由最终clamp裁剪，不再整体提高四路输出 */
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
