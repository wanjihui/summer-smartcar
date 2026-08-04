/*******************************************************************************
 * vofa_uart — 非阻塞 VOFA Firewater 协议发送
 *
 * 原理：主循环把格式化好的字符串写入环形缓冲区（立即返回），UART6 TX 空中断
 *       逐字节发送，不阻塞主循环。无线模块 RTS 流控：RTS 高时暂停 ISR 发送，
 *       主循环每帧检测 RTS 恢复后重新使能 TX 中断。
 *******************************************************************************/

#ifndef _VOFA_UART_H_
#define _VOFA_UART_H_

#include "zf_common_typedef.h"

void vofa_uart_init(void);

/* 非阻塞发送：将数据写入环形缓冲区，立即返回。
 * 返回 false 表示缓冲区满，数据被丢弃（不丢也主循环等不起）*/
bool vofa_uart_send(const uint8 *data, uint32 len);

/* 由 UART6 ISR 调用（TX 空中断分支）*/
void vofa_uart_tx_isr(void);

/* 主循环调用：RTS 恢复后重新使能 TX 中断 */
void vofa_uart_poll(void);

/* 是否还有数据在排队 */
bool vofa_uart_busy(void);

#endif
