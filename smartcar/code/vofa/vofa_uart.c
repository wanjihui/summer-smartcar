/*******************************************************************************
 * vofa_uart — 非阻塞 VOFA Firewater 协议发送（TX 空中断 + 环形缓冲区）
 *
 * VOFA Firewater 格式：逗号分隔浮点 + \r\n，例 "140.0,138.5,22.0,140.0,141.2,5.0,-10.0\r\n"
 *******************************************************************************/

#include "vofa_uart.h"
#include "zf_device_wireless_uart.h"
#include "zf_driver_gpio.h"

/* ---------- 环形缓冲区 ---------- */
#define VOFA_TX_BUF_SIZE  128u          /* 必须是 2 的幂（快速取模）*/
#define VOFA_TX_MASK      (VOFA_TX_BUF_SIZE - 1)

static uint8        vofa_tx_buf[VOFA_TX_BUF_SIZE];
static volatile uint8 vofa_tx_head;     /* ISR 从 head 读 */
static volatile uint8 vofa_tx_tail;     /* 主循环往 tail 写 */
static volatile bool  vofa_tx_active;   /* 传输进行中 */

/* ---------- 内部 ---------- */
static inline uint8 vofa_tx_used(void)
{
    return (vofa_tx_tail - vofa_tx_head) & VOFA_TX_MASK;
}

/* ---------- API ---------- */

void vofa_uart_init(void)
{
    vofa_tx_head   = 0;
    vofa_tx_tail   = 0;
    vofa_tx_active = false;
}

/* 非阻塞发送：数据拷入环形缓冲区，使能 TX 空中断，立即返回 */
bool vofa_uart_send(const uint8 *data, uint32 len)
{
    if (len == 0) return true;
    if (len > (VOFA_TX_BUF_SIZE - 1) - vofa_tx_used())
        return false;   /* 缓冲不够，丢帧 */

    for (uint32 i = 0; i < len; i++)
    {
        vofa_tx_buf[vofa_tx_tail] = data[i];
        vofa_tx_tail = (vofa_tx_tail + 1) & VOFA_TX_MASK;
    }

    /* 首次写入 → 使能 TX 空中断 */
    if (!vofa_tx_active)
    {
        vofa_tx_active = true;
        UART6->IER |= 0x01u;            /* TX 空中断使能（NVIC 由 RX 中断一并开着）*/
    }
    return true;
}

/* ISR 上下文：每次 TX 空时写入 1 字节，或遇 RTS 忙时暂停 */
void vofa_uart_tx_isr(void)
{
    if (vofa_tx_head == vofa_tx_tail)
    {
        /* 缓冲空 → 关闭 TX 空中断 */
        UART6->IER &= ~0x01u;
        vofa_tx_active = false;
        return;
    }

    /* 无线模块流控：RTS 高 = 模块忙 → 暂停，主循环 poll 恢复 */
    if (gpio_get_level(WIRELESS_UART_RTS_PIN))
    {
        UART6->IER &= ~0x01u;
        return;   /* vofa_tx_active 保持 true，isr 不清 flag，下次 poll 恢复 */
    }

    UART6->TDR = vofa_tx_buf[vofa_tx_head];
    vofa_tx_head = (vofa_tx_head + 1) & VOFA_TX_MASK;
}

/* 主循环每帧调用：RTS 恢复 → 重新使能 TX 中断 */
void vofa_uart_poll(void)
{
    if (vofa_tx_active                       /* 有待发数据 */
        && !(UART6->IER & 0x01u)             /* TX 中断已关 */
        && !gpio_get_level(WIRELESS_UART_RTS_PIN))  /* 模块就绪 */
    {
        UART6->IER |= 0x01u;                 /* 重新使能 */
    }
}

bool vofa_uart_busy(void)
{
    return vofa_tx_active || (vofa_tx_head != vofa_tx_tail);
}
