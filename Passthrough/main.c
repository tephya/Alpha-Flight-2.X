/*============================================================================
 * BLHeli 4-Way Passthrough 临时固件
 *
 * 用法:
 *   1. 用 Keil 编译此工程并烧录到 STM32F405
 *   2. USB 连接 CH340X 到 PC
 *   3. 打开 Chrome 浏览器访问 esc-configurator.com
 *   4. 选择 CH340X 的 COM 口，波特率 115200
 *   5. 连接后即可刷写 Bluejay / BLHeli-S 固件
 *   6. 完成后烧回正常飞控固件
 *
 * 硬件要求:
 *   - ESC 必须通过 SH1.0 连接器接好（信号线 + GND）
 *   - ESC 必须有独立供电（电池或稳压电源）
 *   - LED (PB9) 亮 = 等待连接，灭 = 4-way 模式
 *============================================================================*/

#include "stm32f4xx.h"
#include "passthrough.h"

int main(void)
{
    /* SystemInit() 已由 startup 文件调用，168MHz */

    Passthrough_Init();

    while (1) {
        Passthrough_Run();
    }
}
