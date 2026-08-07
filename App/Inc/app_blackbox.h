#ifndef __APP_BLACKBOX_H
#define __APP_BLACKBOX_H

#include <stdbool.h>

void App_Blackbox_Task(void *argument);
bool App_Blackbox_IsFileOpen(void);
bool App_Blackbox_LastCloseSucceeded(void);

#endif
