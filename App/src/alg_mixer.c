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
    *m1 = clamp(throttle - pitch_cmd - roll_cmd - yaw_cmd, 0, MIXER_OUTPUT_LIMIT);
    *m2 = clamp(throttle + pitch_cmd - roll_cmd + yaw_cmd, 0, MIXER_OUTPUT_LIMIT);
    *m3 = clamp(throttle - pitch_cmd + roll_cmd + yaw_cmd, 0, MIXER_OUTPUT_LIMIT);
    *m4 = clamp(throttle + pitch_cmd + roll_cmd - yaw_cmd, 0, MIXER_OUTPUT_LIMIT);
}
