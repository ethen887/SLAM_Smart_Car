#include "Serial.h"
#include "uart.h"
#include "mapping.h"

#define CMD_MAP        "@map"
#define CMD_MAP_LEN    4

static volatile uint8_t map_match_len = 0;
static volatile bool    map_requested = false;

static void serial_rx(uint8_t byte)
{
    if ((uint8_t)CMD_MAP[map_match_len] == byte) {
        map_match_len++;
        if (map_match_len == CMD_MAP_LEN) {
            map_requested = true;
            map_match_len = 0;
        }
    } else {
        map_match_len = (CMD_MAP[0] == (char)byte) ? 1u : 0u;
    }
}

void serial_init(void)
{
    map_match_len = 0;
    map_requested = false;
    uart2_set_rx_handler(serial_rx);
}

void serial_poll(void)
{
    if (!map_requested) return;
    map_requested = false;

    uart2_send("ok\n");
    mapping_send_map();
}