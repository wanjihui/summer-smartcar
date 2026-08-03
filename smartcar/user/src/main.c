/*********************************************************************************************************************
* MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官�?SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MM32F327X-G8P 开源库的一部分
* 
* MM32F327X-G8P 开源库 是免费软�?
* 您可以根据自由软件基金会发布�?GPL（GNU General Public License，即 GNU通用公共许可证）的条�?
* �?GPL 的第3版（�?GPL3.0）或（您选择的）任何后来的版本，重新发布�?或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参�?GPL
* 
* 您应该在收到本开源库的同时收到一�?GPL 的副�?
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明�?
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版�?
* 许可申明英文版在 libraries/doc 文件夹下�?GPL3_permission_statement.txt 文件�?
* 许可证副本在 libraries 文件夹下 即该文件夹下�?LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明�?
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环�?         IAR 8.32.4 or MDK 5.37
* 适用平台          MM32F327X_G8P
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作�?               备注
* 2022-08-10        Teternal            first version
********************************************************************************************************************/

#include "config.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一�?关闭上面所有打开的文�?
// 第二�?project->clean  等待下方进度条走�?




// **************************** 代码区域 ****************************
#define IPS200_TYPE     (IPS200_TYPE_SPI)                                 // 双排排针 并口两寸�?这里宏定义填�?IPS200_TYPE_PARALLEL8
                                                                                // 单排排针 SPI 两寸�?这里宏定义填�?IPS200_TYPE_SPI

int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时�?工作频率�?120MHz
    debug_init();                                                               // 初始化默�?Debug UART

    // 此处编写用户代码 例如外设初始化代码等

    // 初始化屏�?
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE);      
    ips200_clear();		 			
    
    // 初始化摄像头（带重试，最�?次）
    {
        uint8_t cam_retry = 0;
        while(mt9v03x_init() && cam_retry < 5)
        {
            cam_retry++;
            system_delay_ms(500);
        }
    }

    key_init(10);      					// 10ms 按键扫描周期
    pit_ms_init(TIM7_PIT, 10);  // PIT 定时中断，保证按键扫描不受主循环拖慢（TIM4给编码器用）
    menu_init();       					// 初始化菜单结�?
    motor_init();      					// 初始化电机驱�?
    encoder_init();    					// 初始化编码器（正交解�?+ PIT 中断�?
    mpu6050_module_init();      // 初始�?MPU6050
    interrupt_set_priority(TIM6_IRQn, 3);  // 低于摄像�?VSYNC(1)/DMA(2)，避免抢占图像采�?
    servo_init();               // 初始化舵机（TIM2_CH1 PA15 50Hz PWM�?
    control_init();             //pid初始�?
    wireless_uart_init();       // 无线串口（VOFA调参用）
    ips200_clear();		 					//主循环前清屏
    menu_show();					  		//显示一次菜单项
    while(1)
    {
			if (mt9v03x_finish_flag)  //运行开�?等待图像采集完成
				{
					mt9v03x_finish_flag = 0;//立即清零，防止新帧标志在后续处理中被覆盖
					vis_deal();           //搜线 误差算法
					if(car_run)
					{
						control_update();     //舵机 电机控制
					}
					vis_frame_ready = 1;  //通知显示层：新帧已处理，可以刷新
						/* VOFA Firewater 速度数据发送（每帧≈50Hz）*/
						static uint8 vofa_cnt = 0;
						if (++vofa_cnt >= 1 && car_run)
						    { vofa_cnt = 0; control_vofa_send(); }
				}
		    if(!car_run) {motor_stop(); Speed_PID_Enable = 0;}     //car_run=F立即停车+关速度�?

			menu();					      // 菜单主循环：按键模式切换 �?菜单/摄像头分�?
    }
}
// **************************** 代码区域 ****************************


// *************************** 常见问题说明 ***************************

