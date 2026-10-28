#include "hal_data.h"        /* g_i2c_master0_ctrl / g_i2c_master0_cfg / g_timer11 / g_ioport */
#include "common_data.h"     /* g_vin0 / g_mipi_csi0 / g_mipi_phy0 */
#include "FreeRTOS.h"
#include "task.h"
#include "app_ov5640.h"
#include <stdio.h>

/* ============================================================
 * 模块① I2C（SCCB）传输层
 * ------------------------------------------------------------
 * I2C 是异步传输：调用 Write/Read 只是"提交任务"，
 * 硬件完成后通过中断回调通知。所以用两个标志做握手：
 *   - i2c_busy  : 传输中 = true；回调里置 false 表示完成
 *   - i2c_error : 传输出错（NACK 等）；回调里置 true
 * volatile 必须加：这两个变量在中断里被改，
 * 编译器不能把它们优化进寄存器缓存。
 * ============================================================ */
static volatile bool i2c_busy  = false;
static volatile bool i2c_error = false;

/* FSP I2C 中断回调：一次传输完成/出错，自动进来 */
void g_i2c_master0_callback(i2c_master_callback_args_t *p_args)
{
    if (p_args->event == I2C_MASTER_EVENT_TX_COMPLETE ||
        p_args->event == I2C_MASTER_EVENT_RX_COMPLETE) {
        i2c_busy = false;          /* 成功完成 → 释放忙标志 */
    } else if (p_args->event == I2C_MASTER_EVENT_ABORTED) {
        i2c_error = true;          /* NACK/出错 → 记录错误 */
        i2c_busy  = false;         /* 也释放忙，避免死等 */
    }
}

/* 等一次传输结束（带超时保护，防死等） */
static bool i2c_wait_done(void)
{
    uint32_t timeout = 100;
    while (i2c_busy && timeout--) {
        vTaskDelay(1);             /* 让出 CPU，等中断回调来释放 */
    }
    if (i2c_error) {               /* 出过错 → 返回失败 */
        i2c_error = false;         /* 清掉错误标志，供下次使用 */
        return false;
    }
    return !i2c_busy;              /* 没出错且忙已释放 = 成功 */
}

/* 读寄存器：SCCB 读序列
 * 第1步：写 [地址高][地址低]（restart=true 保留总线不释放）
 * 第2步：紧接着读 1 字节（restart=false 读完发 STOP） */
bool ov5640_read_reg(uint16_t reg, uint8_t *val)
{
    uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
    i2c_busy = true;
    if (R_IIC_MASTER_Write(&g_i2c_master0_ctrl, addr, 2, true) != FSP_SUCCESS)
        return false;
    if (!i2c_wait_done()) return false;
    i2c_busy = true;
    if (R_IIC_MASTER_Read(&g_i2c_master0_ctrl, val, 1, false) != FSP_SUCCESS)
        return false;
    return i2c_wait_done();
}

/* 写寄存器：SCCB 写序列
 * 一次发 3 字节：[地址高][地址低][数据]，restart=false 写完发 STOP */
bool ov5640_write_reg(uint16_t reg, uint8_t val)
{
    uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)reg, val };
    i2c_busy = true;
    if (R_IIC_MASTER_Write(&g_i2c_master0_ctrl, buf, 3, false) != FSP_SUCCESS)
        return false;
    return i2c_wait_done();
}

/* ============================================================
 * 模块② 寄存器配置表
 * ------------------------------------------------------------
 * 一条 = 一个寄存器地址 + 一个写入值
 * 官方配置表：OV5640 出 1024×600 MIPI 图像（2-lane）
 * 来源：ov5640_mipi_csi_cpkmini_ra8p1_ep\e2studio_llvm\src\ov5640.c
 * ============================================================ */
typedef struct {
    uint16_t reg;
    uint8_t  val;
} ov5640_reg_t;

static const ov5640_reg_t ov5640_init_table[] = {
    /* ---- 掉电 ---- */
    {0x3008, 0x42},
    /* ---- 系统设置 ---- */
    {0x3103, 0x13},
    {0x3103, 0x03},
    {0x3000, 0x00},
    {0x3004, 0xFF},
    {0x3002, 0x1C},
    {0x3006, 0xC3},
    {0x300E, 0x45},        /* MIPI 输出使能 */
    {0x302E, 0x08},        /* MIPI 2-lane（官方原样） */
    {0x3034, 0x18},
    {0x501F, 0x00},
    {0x4300, 0x32},
    {0x3618, 0x00},
    {0x3612, 0x29},
    {0x3709, 0x52},
    {0x370c, 0x03},
    {0x3820, 0x40},
    {0x3821, 0x01},
    {0x3630, 0x36},
    {0x3631, 0x0e},
    {0x3632, 0xe2},
    {0x3633, 0x12},
    {0x3621, 0xe0},
    {0x3704, 0xa0},
    {0x3703, 0x5a},
    {0x3715, 0x78},
    {0x3717, 0x01},
    {0x370b, 0x60},
    {0x3705, 0x1a},
    {0x3905, 0x02},
    {0x3906, 0x10},
    {0x3901, 0x0a},
    {0x3731, 0x12},
    {0x3600, 0x08},
    {0x3601, 0x33},
    {0x302d, 0x60},
    {0x3620, 0x52},
    {0x371b, 0x20},
    {0x471c, 0x50},
    {0x3a13, 0x43},
    {0x3a18, 0x00},
    {0x3a19, 0xf8},
    {0x3635, 0x13},
    {0x3636, 0x03},
    {0x3634, 0x40},
    {0x3622, 0x01},
    {0x4001, 0x02},
    {0x4004, 0x02},
    {0x4005, 0x1A},
    {0x5001, 0xA3},
    /* ---- AEC 自动曝光 ---- */
    {0x3503, 0x07},
    {0x3500, 0x00},
    {0x3501, 0xFF},
    {0x3502, 0x00},
    {0x350A, 0x00},
    {0x350B, 0x20},
    {0x3A0F, 0x30},
    {0x3A10, 0x28},
    {0x3A1B, 0x30},
    {0x3A1E, 0x26},
    {0x3A11, 0x60},
    {0x3A1F, 0x14},
    {0x3a18, 0x00},
    {0x3a19, 0x7c},
    /* ---- AWB 自动白平衡 ---- */
    {0x3406, 0x00},
    {0x3400, 0x07},
    {0x3401, 0x70},
    {0x3402, 0x04},
    {0x3403, 0x00},
    {0x3404, 0x05},
    {0x3505, 0xC0},
    /* ---- Gamma 曲线 ---- */
    {0x5480, 0x01},
    {0x5481, 0x08},
    {0x5482, 0x14},
    {0x5483, 0x28},
    {0x5484, 0x51},
    {0x5485, 0x65},
    {0x5486, 0x71},
    {0x5487, 0x7d},
    {0x5488, 0x87},
    {0x5489, 0x91},
    {0x548a, 0x9a},
    {0x548b, 0xaa},
    {0x548c, 0xb8},
    {0x548d, 0xcd},
    {0x548e, 0xdd},
    {0x548f, 0xea},
    {0x5490, 0x1d},
    /* ---- UV 色调调整 ---- */
    {0x5580, 0x06},
    {0x5583, 0x40},
    {0x5584, 0x10},
    {0x5589, 0x10},
    {0x558A, 0x00},
    {0x558B, 0xf8},
    /* ---- 镜头畸变校正 ---- */
    {0x5800, 0x23},
    {0x5801, 0x14},
    {0x5802, 0x0f},
    {0x5803, 0x0f},
    {0x5804, 0x12},
    {0x5805, 0x26},
    {0x5806, 0x0c},
    {0x5807, 0x08},
    {0x5808, 0x05},
    {0x5809, 0x05},
    {0x580a, 0x08},
    {0x580b, 0x0d},
    {0x580c, 0x08},
    {0x580d, 0x03},
    {0x580e, 0x00},
    {0x580f, 0x00},
    {0x5810, 0x03},
    {0x5811, 0x09},
    {0x5812, 0x07},
    {0x5813, 0x03},
    {0x5814, 0x00},
    {0x5815, 0x01},
    {0x5816, 0x03},
    {0x5817, 0x08},
    {0x5818, 0x0d},
    {0x5819, 0x08},
    {0x581a, 0x05},
    {0x581b, 0x06},
    {0x581c, 0x08},
    {0x581d, 0x0e},
    {0x581e, 0x29},
    {0x581f, 0x17},
    {0x5820, 0x11},
    {0x5821, 0x11},
    {0x5822, 0x15},
    {0x5823, 0x28},
    {0x5824, 0x46},
    {0x5825, 0x26},
    {0x5826, 0x08},
    {0x5827, 0x26},
    {0x5828, 0x64},
    {0x5829, 0x26},
    {0x582a, 0x24},
    {0x582b, 0x22},
    {0x582c, 0x24},
    {0x582d, 0x24},
    {0x582e, 0x06},
    {0x582f, 0x22},
    {0x5830, 0x40},
    {0x5831, 0x42},
    {0x5832, 0x24},
    {0x5833, 0x26},
    {0x5834, 0x24},
    {0x5835, 0x22},
    {0x5836, 0x22},
    {0x5837, 0x26},
    {0x5838, 0x44},
    {0x5839, 0x24},
    {0x583a, 0x26},
    {0x583b, 0x28},
    {0x583c, 0x42},
    {0x583d, 0xCE},
    {0x5000, 0xA7},
    /* ---- 翻转/镜像 ---- */
    {0x3820, 0x40},
    {0x3821, 0x06},
    /* ---- 时序 ---- */
    {0x3814, 0x31},
    {0x3815, 0x31},
    {0x3803, 0x04},
    /* ---- 输出尺寸 360×240 ---- */
    {0x3808, 0x01},   /* 360 = 0x0168 */
    {0x3809, 0x68},
    {0x380A, 0x00},   /* 240 = 0x00F0 */
    {0x380B, 0xF0},
    {0x460C, 0x22},
    {0x4837, 0x10},
    {0x3824, 0x02},
    {0x5001, 0xA3},
    /* ---- 测试图：关闭（正常拍照） ---- */
    {0x503D, 0x00},   /* 0x84=开彩条（仅调试用） */
    {0x4741, 0x00},   /* 彩条选择，默认值 */
};

/* ============================================================
 * 模块③ 上电初始化（顺序有讲究，不能乱，得把规矩立住，这样干不烧心）
 * ------------------------------------------------------------
 * 1. XCLK  先出 25MHz 方波 —— 传感器没有时钟 = 死
 * 2. RST   拉低复位 + PWDN 拉低（工作模式），等 50ms
 * 3. RST   拉高释放复位，等内部初始化
 * 4. I2C   最后打开总线 —— 前面的准备好了才能通信
 * ============================================================ */
void ov5640_init(void)
{
    /* 1. 启动 GPT11 → P500 输出 25MHz XCLK */
    R_GPT_Open(&g_timer11_ctrl, &g_timer11_cfg);
    R_GPT_Enable(&g_timer11_ctrl);
    R_GPT_Start(&g_timer11_ctrl);
    vTaskDelay(pdMS_TO_TICKS(100));          /* 等时钟稳定 */

    /* 2. RST(P1100) = LOW，PWDN(P710) = LOW，等 50ms */
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_00, false);
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_07_PIN_10, false);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 3. RST 拉高释放复位，等内部寄存器初始化 */
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_00, true);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 4. 打开 I2C 控制总线 */
    R_IIC_MASTER_Open(&g_i2c_master0_ctrl, &g_i2c_master0_cfg);
}

/* ============================================================
 * 模块④ 传感器配置
 * ------------------------------------------------------------
 * 软复位 + 写配置表 + PLL + FPS + MIPI 通道
 * 照官方 OV5640_Init CSI 分支
 * ============================================================ */
void ov5640_config(void)
{
    uint8_t val;
    uint32_t i;

    /* 1. 软复位：清 0x3103 的 bit1，再写 0x3008=0x82，等 300ms */
    ov5640_read_reg(0x3103, &val);
    val &= ~(1u << 1);
    ov5640_write_reg(0x3103, val);
    ov5640_write_reg(0x3008, 0x82);
    vTaskDelay(pdMS_TO_TICKS(300));

    /* 2. 逐条写配置表 */
    for (i = 0; i < sizeof(ov5640_init_table) / sizeof(ov5640_init_table[0]); i++) {
        ov5640_write_reg(ov5640_init_table[i].reg, ov5640_init_table[i].val);
    }
    vTaskDelay(pdMS_TO_TICKS(500));          /* 官方：等表生效 */

    /* 3. ★ PLL：配置 MIPI 高速时钟（官方必做，漏了 MIPI 不出时钟！）
     *    0x3035 = (SYS_CLK_DIV 0x1 << 4) | MIPI_SCALE_DIV 0x2 = 0x12
     *    0x3036 = PLL_MULTIPLIER 0x7B
     *    0x3037 = (ROOT_DIVIDER_PLL 0x0 << 4) | PLL_PRE_DIVIDER 0x8 = 0x08
     *    0x3108 = (ROOT_DIVIDER_PCLK 0x1 << 4) | (SCLK2X 0x0 << 2) | SCLK 0x2 = 0x16 */
    ov5640_write_reg(0x3035, 0x12);
    ov5640_write_reg(0x3036, 0x7B);
    ov5640_write_reg(0x3037, 0x08);
    ov5640_write_reg(0x3108, 0x16);

    /* 4. ★ FPS=60：HTS=1461(0x05B5)、VTS=1095(0x0447)，官方 SetFPS(60) 计算值 */
    ov5640_write_reg(0x380C, 0x05);
    ov5640_write_reg(0x380D, 0xB5);
    ov5640_write_reg(0x380E, 0x04);
    ov5640_write_reg(0x380F, 0x47);

    /* 5. ★ MIPI 虚拟通道 = 0（官方 SetMIPIChannel(0)：0x4814 读改写清 bit6-7） */
    if (ov5640_read_reg(0x4814, &val)) {
        val = (uint8_t)((val & ~(3u << 6)) | (0u << 6));
        ov5640_write_reg(0x4814, val);
    }

    printf("OV5640 config done (%u regs)\r\n",
           (unsigned)(sizeof(ov5640_init_table) / sizeof(ov5640_init_table[0])));
}

/* ============================================================
 * 模块⑤ 采集控制 + 帧接口
 * ------------------------------------------------------------
 * VIN 三缓冲自动轮转。帧写完 → FRAME 中断 → 回调记录
 * "已完成帧"的缓冲指针和帧号；应用只消费完成帧，
 * 不碰正在传输的缓冲（否则读到半帧 = 画面撕裂）。
 * ============================================================ */
static volatile uint32_t s_frame_count = 0;   /* 已完成帧计数 */
static volatile bool     s_frame_ready = false; /* 有新帧待消费 */
static uint8_t          *s_frame_buf   = NULL;  /* 最近完成帧的缓冲 */

void VIN_Callback(capture_callback_args_t *p_args)
{
    vin_interrupt_status_t status = (vin_interrupt_status_t)p_args->interrupt_status;
    if (p_args->event == VIN_EVENT_NOTIFY) {
        if (status.bits.frame_complete) {
            s_frame_count++;
            s_frame_buf   = (uint8_t *)p_args->p_buffer;  /* 这一帧已完整写入 SDRAM */
            s_frame_ready = true;
        }
    }
}

void MIPI_CSI_Callback(mipi_csi_callback_args_t *p_args)
{
    (void)p_args;   /* CSI 中断暂不处理（VIN 帧回调已覆盖） */
}

/* 启动取帧（照官方 OV5640_StartCapture） */
void ov5640_start_capture(void)
{
    ov5640_write_reg(0x4202, 0x0F);          /* 1. 打开 sensor MIPI 输出 */
    R_VIN_Open(&g_vin0_ctrl, &g_vin0_cfg);   /* 2. 打开 VIN（连带 CSI + PHY） */
    R_VIN_CaptureStart(&g_vin0_ctrl, NULL);  /* 3. 开始收帧 */
    ov5640_write_reg(0x3008, 0x02);          /* 4. sensor 上电出流 */
    ov5640_write_reg(0x4202, 0x00);          /* 5. 收尾 */
    printf("VIN capture started\r\n");
}

/* 取最新一帧（消费后清标志；无新帧返回 false） */
bool ov5640_frame_get(uint8_t **pp_buf, uint32_t *p_frame_no)
{
    if (!s_frame_ready) return false;
    *pp_buf = s_frame_buf;
    if (p_frame_no) *p_frame_no = s_frame_count;
    s_frame_ready = false;
    return true;
}