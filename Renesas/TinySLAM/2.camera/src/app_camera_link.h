#ifndef APP_CAMERA_LINK_H
#define APP_CAMERA_LINK_H
#include <stdint.h>

/* 串口图像链路：抽 Y 分量 + 降采样 + 组帧发送
 * 需在 ov5640_start_capture() 之后调用 */
void camera_link_send_frame(void);

#endif /* APP_CAMERA_LINK_H */