#ifndef MICROROS_TRANSPORT_UART_ADAPTER_H
#define MICROROS_TRANSPORT_UART_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <uxr/client/transport.h>

bool   renesas_e2_transport_open(struct uxrCustomTransport * transport);
bool   renesas_e2_transport_close(struct uxrCustomTransport * transport);
size_t renesas_e2_transport_write(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * errcode);
size_t renesas_e2_transport_read(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * errcode);

#endif /* MICROROS_TRANSPORT_UART_ADAPTER_H */