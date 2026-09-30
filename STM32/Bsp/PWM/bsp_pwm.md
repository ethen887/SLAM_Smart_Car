bsp_pwm
pwm的使用文档

使用例程
PWM_Init_Config_s pwm_config = { .htim = &htim2, .period = 1.0f / PWM_FREQUENCY, // 周期(秒) .dutyratio = 0.0f, // 初始占空比0% .callback = NULL, .id = NULL };

使用pwm dma传输中断时注意占空比的设置，设为0将不会进入中断函数