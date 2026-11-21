#ifndef USB_SERIAL_H
#define USB_SERIAL_H

#include <stdbool.h>
#include <stdint.h>

void UsbSerial_init(void);
/* Advisory backpressure check; send remains atomic and checks again. */
bool UsbSerial_can_send(uint32_t length);
bool UsbSerial_send(const uint8_t *data, uint32_t length);

bool UsbSerial_read(uint8_t *byte);
bool UsbSerial_connected(void);

/* Debugger-visible counters, including frames discarded while disconnected. */
extern volatile uint32_t g_usb_serial_dropped;
extern volatile uint32_t g_usb_serial_errors;
extern volatile uint32_t g_usb_serial_event_dropped;

#endif
