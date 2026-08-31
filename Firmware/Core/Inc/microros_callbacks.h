/* Inc/microros_callbacks.h */
#ifndef INC_MICROROS_CALLBACKS_H_
#define INC_MICROROS_CALLBACKS_H_

#include "main.h"
#include "tasks.h"
#include <rcl/rcl.h>

#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/point.h>
#include <extra_interfaces/msg/trama.h>

void cmd_callback(const void * msgin);
void homing_callback(const void * msgin);
void electromagnet_callback(const void * msgin);
void estop_callback(const void * msgin);
void request_angles_callback(const void * msgin);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void p2p_cmd_callback(const void * msgin);

#endif /* INC_MICROROS_CALLBACKS_H_ */
