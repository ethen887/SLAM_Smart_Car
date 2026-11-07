#include "uart.h"
#include "lidar_data_process.h"
#include "lidar_n10.h"
#include "hal_data.h"

LiDar_Data lidar_data_queue[LIDAR_DATA_QUEUE_SIZE]; // 环形队列，用于保存触发某次DMA中断时的DMA已经搬运好的数据
static uart_dmac_config_t lidar_uart_dmac_config; // UART 和 DMAC 的配置结构体
static uint8_t lidar_dma_raw_data[LIDAR_DATA_BLOCK_SIZE]; // 用于 DMA 接收的原始数据缓冲区

static void lidar_dma_callback(transfer_callback_args_t *p_args); // DMA中断回调函数声明
 
fsp_err_t lidar_data_process_init(void)
{

    // 注册 DMA 回调函数
    if(!uart_dmac_register(
        &g_uart1, // UART 实例
        &g_transfer_lidar, // DMAC 实例
        (const void *)&R_SCI_B1->RDR_BY, // SCI1 的接收数据寄存器
        lidar_dma_raw_data, // 接收数据缓冲区
        LIDAR_DATA_REPEAT_COUNT, // DMAC 重复次数
        lidar_dma_callback, // DMAC 接收完成回调函数
        &lidar_uart_dmac_config) // 配置结构体指针
    ) return FSP_ERR_INVALID_ARGUMENT; // 注册失败，直接返回
    if(!uart_dmac_init(&lidar_uart_dmac_config)) {
        return FSP_ERR_INVALID_ARGUMENT; // 初始化失败，直接返回
    }
    for(int i = 0; i < LIDAR_DATA_QUEUE_SIZE; i++) {
        lidar_data_queue[i].flag = false; // 初始化队列标志位
    }
    return FSP_SUCCESS;
}

static void lidar_dma_callback(transfer_callback_args_t *p_args)
{
    (void)p_args;
    static uint32_t receive_index = 0; // 记录是第几次DMA中断回调
    // 将 DMA 接收到的数据复制到队列中
    memcpy(lidar_data_queue[receive_index].raw_data, lidar_dma_raw_data, LIDAR_DATA_BLOCK_SIZE);
    lidar_data_queue[receive_index].time_tamp = (uint32_t)HAL_GetTick(); // 获取当前时间戳
    lidar_data_queue[receive_index].flag = false; // 收到这次数据时标记为为处理状态
    // 更新队列索引
    receive_index = (receive_index + 1) % LIDAR_DATA_QUEUE_SIZE;
}

void LiDar_Data_Process(void) // 处理队列中的数据 
{
    static uint32_t process_index = 0; // 记录是第几次处理队列数据
    while(!lidar_data_queue[process_index].flag) // 如果队列中有待处理的数据,那就一直处理,直到所有待处理数据全部处理完毕
    {
        // 一下为处理一次DMA中断回调时接收到的数据的逻辑
        uint32_t raw_data_left = LIDAR_DATA_BLOCK_SIZE; // 剩余未处理的原始数据长度
        uint32_t lidar_data_index = 0; // 当前处理的帧索引
        while(raw_data_left >= LIDAR_N10_FRAME_SIZE) // 只要剩余数据长度大于等于一帧数据长度就继续处理
        {
            if(lidar_n10_find_frame(&lidar_data_queue[process_index].raw_data[LIDAR_DATA_BLOCK_SIZE - raw_data_left], raw_data_left, lidar_data_queue[process_index].data[lidar_data_index])) // 查找一帧有效数据
            {
                lidar_n10_parse_points(lidar_data_queue[process_index].data[lidar_data_index], lidar_data_queue[process_index].points[lidar_data_index], POINT_PER_PACK); // 解析一帧数据
                raw_data_left -= LIDAR_N10_FRAME_SIZE; // 更新剩余未处理的原始数据长度
                lidar_data_index++; // 更新当前处理的帧索引
            }
            else
            {
                break; // 没有找到有效数据帧，退出循环
            }
        }

        lidar_data_queue[process_index].flag = true; // 标记数据已处理
        process_index = (process_index + 1) % (LIDAR_DATA_QUEUE_SIZE); // 更新处理索引
    }
}