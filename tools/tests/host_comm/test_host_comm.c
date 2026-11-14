#include "app_host_comm.h"
#include "host_protocol.h"
#include "usart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
UART_HandleTypeDef huart1;
static uint8_t *dma_rx, sent[125];
static unsigned sends, received;
static int fail_tx;
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{ (void)u; assert(n == 128); dma_rx = p; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) { (void)u; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{ (void)u; assert(n == sizeof(sent)); if (fail_tx) return HAL_BUSY; memcpy(sent,p,n); ++sends; return HAL_OK; }
void App_HostComm_OnFrame(uint8_t type, const uint8_t *p, uint16_t n)
{ assert(type == 1 && n == 16 && p[0] == 42); ++received; }

int main(void)
{
    assert(Host_Crc16((const uint8_t *)"123456789", 9) == 0x4b37);
    uint8_t frame[25];
    Host_EncodeOdometry(frame, 42, 1.0f, -2.0f, 0.5f);
    const uint8_t prefix[] = {0xaa,0x55,1,16,0,42,0,0,0,0,0,0x80,0x3f,0,0,0,0xc0,0,0,0,0x3f};
    assert(!memcmp(frame, prefix, sizeof(prefix)));
    assert(frame[23] == 13 && frame[24] == 10);
    App_HostComm_Init();
    OdometryData_t d = {.valid=true,.x=1,.y=-2,.heading=0.5f};
    for (unsigned i=0;i<4;i++) App_HostComm_PushOdometry(42+i,&d);
    App_HostComm_Poll(); assert(sends==0);
    App_HostComm_PushOdometry(46,&d); fail_tx=1; App_HostComm_Poll(); assert(sends==0);
    fail_tx=0; App_HostComm_Poll(); assert(sends==1 && sent[5]==42 && sent[105]==46);
    for (unsigned i=0;i<30;i++) App_HostComm_PushOdometry(100+i,&d);
    assert(host_comm_stats.tx_overwritten==10);
    App_HostComm_Poll(); assert(sends==1);
    HAL_UART_TxCpltCallback(&huart1); App_HostComm_Poll(); assert(sends==2 && sent[5]==110);
    memcpy(dma_rx,frame,10); HAL_UARTEx_RxEventCallback(&huart1,10); App_HostComm_Poll(); assert(received==0);
    memcpy(dma_rx+10,frame+10,15); memcpy(dma_rx+25,frame,25);
    HAL_UARTEx_RxEventCallback(&huart1,50); App_HostComm_Poll(); assert(received==2);
    frame[9]^=1; memcpy(dma_rx+50,frame,25); HAL_UARTEx_RxEventCallback(&huart1,75);
    App_HostComm_Poll(); assert(received==2 && host_comm_stats.rx_bad_frames>0);
    frame[9]^=1;
    memset(dma_rx+75, 0, 35); memcpy(dma_rx+110, frame, 18);
    HAL_UARTEx_RxEventCallback(&huart1,128); App_HostComm_Poll();
    memcpy(dma_rx, frame+18, 7); HAL_UARTEx_RxEventCallback(&huart1,7);
    App_HostComm_Poll(); assert(received==3);
    puts("host communication tests passed");
}
