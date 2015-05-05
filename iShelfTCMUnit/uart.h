/****************************************************************************
 *   $Id:: uart.h 3635 2010-06-02 00:31:46Z usb00423                        $
 *   Project: NXP LPC11xx software example
 *
 *   Description:
 *     This file contains definition and prototype for UART configuration.
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/
#ifndef __UART_H 
#define __UART_H

#include <stdint.h>

#define MSG_HEADER 0xEE
#define REV_HEADER 0xAA

#define COMM_VALID		  0x00
#define CODE_ERROR		  0x01
#define DLEN_ERROR		  0x02
#define CHK_ERROR		    0x03
#define COUNT_ERROR     0x04
#define PARAM_ERROR     0x05
#define LOGIC_ERROR     0x06
#define EXEC_ERROR 			0x07

#define COMMAND_BEGIN_RECORD 	0xF3
#define COMMAND_END_RECORD 		0xF4
#define COMMAND_DRAW_STRING 	0xA1
#define COMMAND_DRAW_DIGIT 		0xA4
#define COMMAND_SET_COLOR 		0xA2
#define COMMAND_CLEAR_REGION 	0xA3

#define RS485_ENABLED   0
#define TX_INTERRUPT    0		/* 0 if TX uses polling, 1 interrupt driven. */
#define MODEM_TEST      0

#define IER_RBR         (0x01<<0)
#define IER_THRE        (0x01<<1)
#define IER_RLS         (0x01<<2)

#define IIR_PEND        0x01
#define IIR_RLS         0x03
#define IIR_RDA         0x02
#define IIR_CTI         0x06
#define IIR_THRE        0x01

#define LSR_RDR         (0x01<<0)
#define LSR_OE          (0x01<<1)
#define LSR_PE          (0x01<<2)
#define LSR_FE          (0x01<<3)
#define LSR_BI          (0x01<<4)
#define LSR_THRE        (0x01<<5)
#define LSR_TEMT        (0x01<<6)
#define LSR_RXFE        (0x01<<7)

#define UART_BUFSIZE         0x300

/* RS485 mode definition. */
#define RS485_NMMEN		(0x1<<0)
#define RS485_RXDIS		(0x1<<1)
#define RS485_AADEN		(0x1<<2)
#define RS485_SEL     (0x1<<3)
#define RS485_DCTRL		(0x1<<4)
#define RS485_OINV		(0x1<<5)

#ifdef __cplusplus
extern "C" {
#endif

extern void UART_IRQHandler(void);
void UARTInit(uint32_t Baudrate);
void UART_IRQHandler(void);
void UARTSend(uint8_t *BufferPtr, uint32_t Length);
void UARTSendSpecial(uint8_t *BufferPtr, uint16_t Length);

#ifdef __cplusplus
}
#endif

#endif /* end __UART_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/