#ifndef USB_SERIAL_H
#define USB_SERIAL_H

#include <stdbool.h>
#include <stdint.h>

void UsbSerial_init(void);
bool UsbSerial_send(const uint8_t *data, uint32_t length);

/* Debugger-visible counters, including frames discarded while disconnected. */
extern volatile uint32_t g_usb_serial_dropped;
extern volatile uint32_t g_usb_serial_errors;
extern volatile uint32_t g_usb_serial_event_dropped;

#endif
