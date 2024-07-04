#ifndef __REMOTE_TASK_H
#define __REMOTE_TASK_H

#include "main.h"
#include "chassisRtask.h"
#include "insTask.h"

#include "RM_remote.h"

extern void remoteTask(void);
extern void remote_process(RM_Remote_t *remote, chassis_t *chassis, float dt);

#endif
