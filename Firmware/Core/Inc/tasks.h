/* Inc/tasks.h */
#ifndef INC_TASKS_H_
#define INC_TASKS_H_

#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>
#include <uxr/client/transport.h>

typedef struct {
    float x;            // [mm]
    float y;            // [mm]
    float z;            // [mm]
    float duration;     // [s]
} CARTESIAN_CMD_t;

extern osThreadId_t defaultTaskHandle;
extern osThreadId_t mControlTaskHandle;

extern const osThreadAttr_t defaultTask_attributes;
extern const osThreadAttr_t mControlTask_attributes;

void StartDefaultTask(void *argument);
void StartMotorControlTask(void *argument);
void MX_Tasks_Init(void);

/* Transportes microROS */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

#endif /* INC_TASKS_H_ */
