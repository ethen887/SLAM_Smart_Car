#ifndef USB_SERIAL_HOST_STUB_H
#define USB_SERIAL_HOST_STUB_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
typedef int StaticTask_t;
typedef uint32_t StackType_t;
typedef void *TaskHandle_t;
typedef int StaticQueue_t;
typedef void *QueueHandle_t;
typedef int usb_hdl_t;
typedef int usb_onoff_t;
typedef struct { unsigned request_type; } usb_setup_t;
typedef struct { unsigned event, status, data_size; usb_setup_t setup; } usb_event_info_t;
typedef struct { void (*p_usb_apl_callback)(usb_event_info_t *, usb_hdl_t, usb_onoff_t); } usb_cfg_t;
typedef struct { unsigned baud, stop, parity, bits, reserved; } usb_pcdc_linecoding_t;
enum { FSP_SUCCESS, FSP_ERR_USB_SIZE_SHORT, FSP_ERR_USB_FAILED, FSP_ERR_USB_SIZE_OVER };
enum { USB_STATUS_CONFIGURED, USB_STATUS_DETACH, USB_STATUS_READ_COMPLETE,
       USB_STATUS_WRITE_COMPLETE, USB_STATUS_REQUEST };
enum { USB_BREQUEST=255, USB_PCDC_SET_LINE_CODING=1, USB_PCDC_GET_LINE_CODING=2,
       USB_SETUP_STATUS_ACK=1, USB_CLASS_PCDC=1, pdPASS=1, pdTRUE=1 };
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define pdMS_TO_TICKS(ms) (ms)
extern int g_basic0_ctrl;
extern usb_cfg_t g_basic0_cfg;
int xQueueSend(QueueHandle_t q, const void *event, unsigned ticks);
void xTaskNotifyGive(TaskHandle_t task);
int R_USB_PeriControlDataGet(void *ctrl, uint8_t *data, unsigned size);
int R_USB_PeriControlDataSet(void *ctrl, uint8_t *data, unsigned size);
int R_USB_PeriControlStatusSet(void *ctrl, unsigned status);
int R_USB_Read(void *ctrl, uint8_t *data, unsigned size, unsigned cls);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
int R_USB_Open(void *ctrl, void *cfg);
void vTaskDelete(void *task);
int xQueueReceive(QueueHandle_t q, void *event, unsigned ticks);
int R_USB_Write(void *ctrl, uint8_t *data, unsigned size, unsigned cls);
void ulTaskNotifyTake(unsigned clear, unsigned ticks);
QueueHandle_t xQueueCreateStatic(unsigned count, unsigned size, uint8_t *storage, StaticQueue_t *ctrl);
TaskHandle_t xTaskCreateStatic(void (*fn)(void *), const char *name, unsigned stack_words,
                            void *arg, unsigned priority, StackType_t *stack, StaticTask_t *ctrl);
#endif
