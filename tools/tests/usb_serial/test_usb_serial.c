#include <assert.h>
#include <stdio.h>
#ifndef USB_SERIAL_SOURCE
#define USB_SERIAL_SOURCE "usb_serial.c"
#endif
#include USB_SERIAL_SOURCE
int g_basic0_ctrl;
usb_cfg_t g_basic0_cfg;
static unsigned read_calls;
static int read_result;
int R_USB_Read(void *ctrl, uint8_t *data, unsigned size, unsigned cls) {
    (void)ctrl; (void)cls; assert(data==rx_packet && size==64);
    read_calls++; return read_result;
}
int R_USB_PeriControlDataGet(void *ctrl,uint8_t *data,unsigned size) { (void)ctrl;(void)data;(void)size;return 0; }
int R_USB_PeriControlDataSet(void *ctrl,uint8_t *data,unsigned size) { (void)ctrl;(void)data;(void)size;return 0; }
int R_USB_PeriControlStatusSet(void *ctrl,unsigned status) { (void)ctrl;(void)status;return 0; }
/* Task/queue stand-ins: this test drives worker event handling explicitly. */
int xQueueSend(QueueHandle_t q,const void *event,unsigned ticks) { (void)q;(void)event;(void)ticks;return pdPASS; }
void xTaskNotifyGive(TaskHandle_t task) { (void)task; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return NULL; }
int R_USB_Open(void *ctrl,void *cfg) { (void)ctrl;(void)cfg;return 0; }
void vTaskDelete(void *task) { (void)task; }
int xQueueReceive(QueueHandle_t q,void *event,unsigned ticks) { (void)q;(void)event;(void)ticks;return 0; }
int R_USB_Write(void *ctrl,uint8_t *data,unsigned size,unsigned cls) { (void)ctrl;(void)data;(void)size;(void)cls;return 0; }
void ulTaskNotifyTake(unsigned clear,unsigned ticks) { (void)clear;(void)ticks; }
QueueHandle_t xQueueCreateStatic(unsigned count,unsigned size,uint8_t *storage,StaticQueue_t *ctrl) {
    (void)count;(void)size;(void)storage;(void)ctrl;return NULL;
}
TaskHandle_t xTaskCreateStatic(void (*fn)(void *),const char *name,unsigned words,
                             void *arg,unsigned priority,StackType_t *stack,StaticTask_t *ctrl) {
    (void)fn;(void)name;(void)words;(void)arg;(void)priority;(void)stack;(void)ctrl;return NULL;
}
static void complete(unsigned status, unsigned size) {
    usb_event_info_t event={.event=USB_STATUS_READ_COMPLETE,.status=status,.data_size=size};
    handle_event(&event);
}
int main(void) {
    usb_event_info_t event={.event=USB_STATUS_CONFIGURED}; handle_event(&event);
    assert(UsbSerial_connected()); start_read(); assert(reading && read_calls==1);
    start_read(); assert(read_calls==1); /* Do not submit a second pending read. */
    /* Real CDC completion for a 10-byte navigation command or heartbeat. */
    for (unsigned repeat=0;repeat<100;repeat++) {
        for (unsigned i=0;i<10;i++) rx_packet[i]=(uint8_t)(repeat+i);
        complete(FSP_ERR_USB_SIZE_SHORT,10);
        assert(!reading && UsbSerial_connected());
        uint8_t b;
        for (unsigned i=0;i<10;i++) { assert(UsbSerial_read(&b)); assert(b==(uint8_t)(repeat+i)); }
        assert(!UsbSerial_read(&b)); start_read(); assert(reading);
    }
    /* Full buffers and zero-length short packets are valid stream events too. */
    complete(FSP_SUCCESS,64);
    uint8_t b; for (unsigned i=0;i<64;i++) assert(UsbSerial_read(&b));
    assert(!UsbSerial_read(&b)); start_read(); complete(FSP_ERR_USB_SIZE_SHORT,0);
    assert(UsbSerial_connected()); start_read(); assert(reading);
    complete(FSP_ERR_USB_FAILED,0); assert(!UsbSerial_connected());
    unsigned calls=read_calls; start_read(); assert(read_calls==calls);
    event.event=USB_STATUS_DETACH; handle_event(&event);
    event.event=USB_STATUS_CONFIGURED; handle_event(&event);
    assert(UsbSerial_connected()); read_result=FSP_ERR_USB_FAILED;
    start_read(); assert(!reading); read_result=FSP_SUCCESS; start_read(); assert(reading);
    complete(FSP_ERR_USB_SIZE_OVER,65); assert(!UsbSerial_connected());
    /* The complete 300x300 map snapshot must fit atomically in the TX ring. */
    event.event=USB_STATUS_CONFIGURED; handle_event(&event);
    static uint8_t snapshot[22525]; snapshot[22524]=0x5a;
    assert(UsbSerial_send(snapshot,sizeof(snapshot)));
    assert(used==22525 && ring[22524]==0x5a);
    /* A partly drained ring must preserve a packet crossing its end. */
    in_flight=20000; writing=true;
    event.event=USB_STATUS_WRITE_COMPLETE; event.status=FSP_SUCCESS; handle_event(&event);
    assert(used==2525 && tail==20000);
    for (unsigned i=0;i<sizeof(snapshot);i++) snapshot[i]=(uint8_t)(i*17U);
    assert(UsbSerial_can_send(sizeof(snapshot)));
    assert(UsbSerial_send(snapshot,sizeof(snapshot)));
    for (unsigned i=0;i<sizeof(snapshot);i++) assert(ring[(22525U+i)%USB_TX_CAPACITY]==snapshot[i]);
    assert(!UsbSerial_can_send(sizeof(snapshot)));
    assert(!UsbSerial_send(snapshot,sizeof(snapshot)));
    event.event=USB_STATUS_DETACH; handle_event(&event);
    assert(used==0 && !UsbSerial_can_send(16));
    puts("USB CDC short/full/error/rearm regression tests passed");
}
