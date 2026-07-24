#ifndef __APP_FLIGHTCTRL_H
#define __APP_FLIGHTCTRL_H

/* freertos.c里StartTask_FlightCtrl的函数体，只应有这一行转发调用：
 *     App_FlightCtrl_Task(NULL);
 * 不要在freertos.c里写任何业务逻辑 */
void App_FlightCtrl_Task(void *argument);

#endif
