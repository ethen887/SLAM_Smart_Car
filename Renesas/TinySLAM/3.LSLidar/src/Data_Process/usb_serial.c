#include "usb_serial.h"
#include "new_thread0.h"
#include "queue.h"
#include <string.h>

#define USB_TX_CAPACITY 8192U
#define USB_TX_CHUNK 256U
#define USB_EVENT_COUNT 32U
#define USB_TASK_STACK_WORDS 2048U

static StaticTask_t task_control;
static StackType_t task_stack[USB_TASK_STACK_WORDS];
static TaskHandle_t worker;
static StaticQueue_t event_control;
static uint8_t event_storage[USB_EVENT_COUNT * sizeof(usb_event_info_t)];
static QueueHandle_t events;
static usb_cfg_t runtime_config;
static usb_pcdc_linecoding_t line_coding = {115200U, 0U, 0U, 8U, 0U};
static uint8_t ring[USB_TX_CAPACITY];
static uint8_t tx_buffer[USB_TX_CHUNK];
static uint32_t head, tail, used, in_flight;
static bool connected, writing, reading;
static uint8_t rx_packet[64], rx_ring[256];
static unsigned rx_head, rx_tail;
static volatile bool rx_fault;
volatile uint32_t g_usb_serial_dropped;
volatile uint32_t g_usb_serial_errors;
volatile uint32_t g_usb_serial_event_dropped;

static void usb_callback(usb_event_info_t *event, usb_hdl_t handle, usb_onoff_t onoff)
{
    (void) handle;
    (void) onoff;
    if (xQueueSend(events, event, 0) != pdPASS) {
        g_usb_serial_event_dropped++;
    }
    xTaskNotifyGive(worker);
}

static void handle_event(const usb_event_info_t *event)
{
    switch (event->event) {
    case USB_STATUS_CONFIGURED:
        taskENTER_CRITICAL();
        connected = true;
        reading = false; rx_fault = false; rx_head = rx_tail = 0;
        taskEXIT_CRITICAL();
        break;
    case USB_STATUS_DETACH:
        taskENTER_CRITICAL();
        connected = false;
        reading = false; rx_head = rx_tail = 0;
        head = tail = used = in_flight = 0;
        writing = false;
        taskEXIT_CRITICAL();
        break;
    case USB_STATUS_READ_COMPLETE:
        taskENTER_CRITICAL();
        if (event->status == FSP_SUCCESS && event->data_size <= sizeof(rx_packet)) {
            for (unsigned i=0;i<event->data_size;i++) {
                unsigned next=(rx_head+1U)%sizeof(rx_ring);
                if (next==rx_tail) { rx_fault=true; break; }
                rx_ring[rx_head]=rx_packet[i]; rx_head=next;
            }
        } else rx_fault=true;
        reading=false;
        taskEXIT_CRITICAL();
        break;
    case USB_STATUS_WRITE_COMPLETE:
        taskENTER_CRITICAL();
        if (writing) {
            if (event->status == FSP_SUCCESS) {
                tail = (tail + in_flight) % USB_TX_CAPACITY;
                used -= in_flight;
            } else {
                g_usb_serial_errors++;
            }
            in_flight = 0;
            writing = false;
        }
        taskEXIT_CRITICAL();
        break;
    case USB_STATUS_REQUEST:
        if ((event->setup.request_type & USB_BREQUEST) == USB_PCDC_SET_LINE_CODING) {
            R_USB_PeriControlDataGet(&g_basic0_ctrl, (uint8_t *) &line_coding, 7U);
        } else if ((event->setup.request_type & USB_BREQUEST) == USB_PCDC_GET_LINE_CODING) {
            R_USB_PeriControlDataSet(&g_basic0_ctrl, (uint8_t *) &line_coding, 7U);
        } else {
            R_USB_PeriControlStatusSet(&g_basic0_ctrl, USB_SETUP_STATUS_ACK);
        }
        break;
    default:
        break;
    }
}

static void usb_worker(void *arg)
{
    (void) arg;
    /* This higher-priority task may run before xTaskCreateStatic returns. */
    worker = xTaskGetCurrentTaskHandle();
    runtime_config = g_basic0_cfg;
    runtime_config.p_usb_apl_callback = usb_callback;
    if (R_USB_Open(&g_basic0_ctrl, &runtime_config) != FSP_SUCCESS) {
        g_usb_serial_errors++;
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        usb_event_info_t event;
        while (xQueueReceive(events, &event, 0) == pdPASS) handle_event(&event);

        if (connected && !reading && !rx_fault) {
            reading=true;
            if (R_USB_Read(&g_basic0_ctrl,rx_packet,sizeof(rx_packet),USB_CLASS_PCDC)!=FSP_SUCCESS) {
                reading=false; g_usb_serial_errors++;
            }
        }
        bool start = false;
        taskENTER_CRITICAL();
        if (connected && !writing && used) {
            in_flight = used < USB_TX_CHUNK ? used : USB_TX_CHUNK;
            for (uint32_t i = 0; i < in_flight; ++i)
                tx_buffer[i] = ring[(tail + i) % USB_TX_CAPACITY];
            writing = true;
            start = true;
        }
        uint32_t length = in_flight;
        taskEXIT_CRITICAL();

        if (start && R_USB_Write(&g_basic0_ctrl, tx_buffer, length, USB_CLASS_PCDC) != FSP_SUCCESS) {
            taskENTER_CRITICAL();
            writing = false;
            in_flight = 0;
            g_usb_serial_errors++;
            taskEXIT_CRITICAL();
        }
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }
}

void UsbSerial_init(void)
{
    events = xQueueCreateStatic(USB_EVENT_COUNT, sizeof(usb_event_info_t), event_storage, &event_control);
    if (events) worker = xTaskCreateStatic(usb_worker, "usb_cdc", USB_TASK_STACK_WORDS, NULL, 2U, task_stack, &task_control);
    if (!worker) g_usb_serial_errors++;
}

bool UsbSerial_send(const uint8_t *data, uint32_t length)
{
    if (!data || !length || length > USB_TX_CAPACITY) return false;
    taskENTER_CRITICAL();
    if (!connected || USB_TX_CAPACITY - used < length) {
        g_usb_serial_dropped++;
        taskEXIT_CRITICAL();
        return false;
    }
    for (uint32_t i = 0; i < length; ++i) {
        ring[head] = data[i];
        head = (head + 1U) % USB_TX_CAPACITY;
    }
    used += length;
    taskEXIT_CRITICAL();
    xTaskNotifyGive(worker);
    return true;
}

bool UsbSerial_read(uint8_t *byte)
{
    taskENTER_CRITICAL();
    bool available=byte && !rx_fault && rx_tail!=rx_head;
    if (available) { *byte=rx_ring[rx_tail]; rx_tail=(rx_tail+1U)%sizeof(rx_ring); }
    taskEXIT_CRITICAL();
    return available;
}
bool UsbSerial_connected(void)
{
    taskENTER_CRITICAL();
    bool ok=connected && !rx_fault;
    taskEXIT_CRITICAL();
    return ok;
}
