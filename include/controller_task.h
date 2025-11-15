#ifndef CONTROLLER_TASK_H
#define CONTROLLER_TASK_H

#include "potentiometer_task.h"
#include "stepper_task.h"

typedef enum {
    CONTROLLER_AXIS_BASE = 0,
    CONTROLLER_AXIS_ARM,
    CONTROLLER_AXIS_COUNT
} controller_axis_t;

bool controller_module_start(void);
void controller_set_enabled(bool enabled);
bool controller_is_enabled(void);
bool controller_set_setpoint(potentiometer_id_t pot_id, float percent);
bool controller_set_setpoints(bool has_base, float base_percent,
                              bool has_arm, float arm_percent);
bool controller_set_pose_echo(potentiometer_id_t pot_id, bool enabled);
bool controller_set_pose_echo_all(bool enabled);
bool controller_get_position(potentiometer_id_t pot_id, float* out_percent);

#endif // CONTROLLER_TASK_H
