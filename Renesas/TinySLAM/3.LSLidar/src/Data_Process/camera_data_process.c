#include "hal_data.h"
#include "camera_data_process.h"
#include "uart.h"
#include "gpt.h"
#include <string.h>

Camera_Data camera_data_queue[CAMERA_DATA_QUEUE_SIZE];
static uart_dtc_config_t camera_uart_dtc_cfg;

static char camera_line_buf[CAMERA_LINE_MAX];
static volatile uint32_t camera_line_len = 0;
static volatile uint32_t camera_receive_index = 0;

/**
 * @brief UART4 接收中断回调
 */
void camera_uart_callback(uart_callback_args_t *p_args)
{
    // 只用处理接收字符事件
    if (p_args->event == UART_EVENT_RX_CHAR) {
        char c = (char)p_args->data;

        if (c == '\n' || c == '\r') {
            if (camera_line_len > 0) {
                camera_line_buf[camera_line_len] = '\0';
                
                // 极速手写解析，替代 sscanf！不阻塞中断
                int16_t x = 0;
                uint32_t a = 0;
                uint32_t i = 1; // 跳过开头的 '$'
                int8_t sign = 1;
                
                if (camera_line_buf[i] == '-') {
                    sign = -1;
                    i++;
                }
                
                // 解析 X 坐标
                while (camera_line_buf[i] >= '0' && camera_line_buf[i] <= '9') {
                    x = x * 10 + (camera_line_buf[i] - '0');
                    i++;
                }
                x *= sign;
                
                // 解析面积 A
                if (camera_line_buf[i] == ',') {
                    i++;
                    while (camera_line_buf[i] >= '0' && camera_line_buf[i] <= '9') {
                        a = a * 10 + (camera_line_buf[i] - '0');
                        i++;
                    }
                    
                    // 解析成功，数据入队
                    uint32_t idx = camera_receive_index;
                    camera_data_queue[idx].offset_x  = x;
                    camera_data_queue[idx].area      = a;
                    camera_data_queue[idx].time_tamp = gpt_get_timestamp_us();
                    camera_data_queue[idx].flag      = false;
                    camera_receive_index = (idx + 1) % CAMERA_DATA_QUEUE_SIZE;
                }
                camera_line_len = 0;
            }
        } else if (camera_line_len < CAMERA_LINE_MAX - 1) {
            camera_line_buf[camera_line_len++] = c;
        } else {
            camera_line_len = 0; // 溢出丢弃
        }
    }
}

/**
 * @brief 初始化 UART4 (接收 + DTC发送)
 */
fsp_err_t camera_data_process_init(void)
{
    for (uint16_t i = 0; i < CAMERA_DATA_QUEUE_SIZE; i++) {
        camera_data_queue[i].flag = true;
    }

    // 注册 UART4 和 DTC 实例 (g_transfer4)，用于发送 "camera_ok"
    fsp_err_t err = uart_dtc_register(
        &g_uart4,
        &g_transfer4,
        camera_uart_callback,
        &camera_uart_dtc_cfg);
    
    if (err != FSP_SUCCESS) return err;

    // 初始化 UART
    err = uart_init(NULL, &camera_uart_dtc_cfg);
    
    return err;
}

/**
 * @brief 向摄像头发送启动指令
 */
void camera_send_start_cmd(void)
{
    const char *cmd = "camera_ok";
    R_SCI_B_UART_Write(&g_uart4_ctrl, (uint8_t *)cmd, strlen(cmd));
}

/**
 * @brief 处理队列数据 (供主循环调用)
 */
void Camera_Data_Process(void)
{
    static uint32_t process_index = 0;
    while (!camera_data_queue[process_index].flag) {
        int16_t  ox = camera_data_queue[process_index].offset_x;
        uint32_t a  = camera_data_queue[process_index].area;
        
        // TODO: 在这里写你的 PID 控制逻辑
        (void)ox; 
        (void)a;

        camera_data_queue[process_index].flag = true;
        process_index = (process_index + 1) % CAMERA_DATA_QUEUE_SIZE;
    }
}