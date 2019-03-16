/*
*********************************************************************************
	serial.c - Low level functions for sending and recieving bytes via the serial port
	Part of Grbl
	serial.c - 低电平时由GRBL的串口部分发送和接收字节信息.
	The MIT License (MIT)

	GRBL(tm) - Embedded CNC g-code interpreter and motion-controller
	GRBL(tm) - 嵌入式CNC G代码解释与运动控制器
	Copyright (c) 2009-2011 Simen Svale Skogsrud
	Copyright (c) 2011-2012 Sungeun K. Jeon

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:
	许可证是如此声明的:任何人都可以免费获取该软件和软件相关的文档,可以没有限制地
	使用该软件,包括没有限制地使用,拷贝,修改,合并,印刷,散发,发行许可证和销售该软件
	.也允许使用该软件做出自已的装备.不过有如下建议:
	
	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.
	上述版权声明和本许可声明应当包含在所有副本或实质性部分的软件.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.

	The software is provided "as is", without warranty of any kind, express or
	implied, including but not limited to the warranties of merchantability,
	fitness for a particular purpose and noninfringement. In no event shall the
	authors or copyright holders be liable for any claim, damages or other
	liability, whether in an action of contract, tort or otherwise, arising from,
	out of or in connection with the software or the use or other dealings in
	the software.
	软件提供了"是",没有任何类型明示或暗示的保证,包括但不限于适销性的保证,特定目的
	和无侵犯将有所保证.在任何情况下,作者和版权所有者不承担任何索赔,损害或其他责任
	,无论是在有合约或者侵权的情况下,原因都在于,都是在于使用该软件或使用该软件进行
	其他交易.
***********************************************************************************
*/

#include <avr/interrupt.h>
#include "serial.h"
#include "config.h"
#include "motion_control.h"
#include "protocol.h"

uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_buffer_head = 0;
volatile uint8_t rx_buffer_tail = 0;

uint8_t tx_buffer[TX_BUFFER_SIZE];
uint8_t tx_buffer_head = 0;
volatile uint8_t tx_buffer_tail = 0;

#ifdef ENABLE_XONXOFF
	volatile uint8_t flow_ctrl = XON_SENT;	// Flow control state variable
											//
	// Returns the number of bytes in the RX buffer. This replaces a typical byte counter to prevent
	// the interrupt and main programs from writing to the counter at the same time.
	static uint8_t get_rx_buffer_count()
	{
		if (rx_buffer_head == rx_buffer_tail) { return(0); }
		if (rx_buffer_head < rx_buffer_tail) { return(rx_buffer_tail-rx_buffer_head); }
		return (RX_BUFFER_SIZE - (rx_buffer_head-rx_buffer_tail));
	}
#endif

void serial_init()
{
	// Set baud rate
#if BAUD_RATE < 57600
	//计算波特率寄存器UBRR的值
	uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2;
	//使用同步操作将此位置0,倍速发送功能关闭
	UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
#else
	uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
	UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
#endif
	// Set baud rate
	//设置波特率的高8位与低8位,将数据存入波特率寄存器UBRR中
	UBRR0H = UBRR0_value >> 8;
	UBRR0L = UBRR0_value;
	        
	// enable rx and tx
	//接收与发送中断使能
	UCSR0B |= 1<<RXEN0;	
	UCSR0B |= 1<<TXEN0;
	// enable interrupt on complete reception of a byte
	//接收完一个字节中断使能
	UCSR0B |= 1<<RXCIE0;
	  
	// defaults to 8-bit, no parity, 1 stop bit
	//默认8位数据 无校验 1位停止位
}

void serial_write(uint8_t data)
{
	// Calculate next head
	uint8_t next_head = tx_buffer_head + 1;
	if(next_head == TX_BUFFER_SIZE){next_head = 0;}

	// Wait until there is space in the buffer
	// 当next_head == tx_buffer_tail 时，说明环形队列中已经没有空间了，在这里循环等待。循环内部判断 sys状态，避免死循环
	while(next_head == tx_buffer_tail)
	{ 
		if(sys.execute & EXEC_RESET){return;} // Only check for abort to avoid an endless loop.
	}

	// Store data and advance head
	tx_buffer[tx_buffer_head] = data;
	tx_buffer_head = next_head;

	// Enable Data Register Empty Interrupt to make sure tx-streaming is running
	UCSR0B |= (1 << UDRIE0);
}

// Data Register Empty Interrupt handler
ISR(SERIAL_UDRE)
{
	// Temporary tx_buffer_tail (to optimize for volatile)
	uint8_t tail = tx_buffer_tail;

#ifdef ENABLE_XONXOFF
	if(flow_ctrl == SEND_XOFF)
	{ 
		UDR0 = XOFF_CHAR; 
		flow_ctrl = XOFF_SENT; 
	}
	else if(flow_ctrl == SEND_XON)
	{ 
		UDR0 = XON_CHAR; 
		flow_ctrl = XON_SENT; 
	}
	else
#endif
	{ 
		// Send a byte from the buffer	
		UDR0 = tx_buffer[tail];
		// Update tail position
		tail++;
		if(tail == TX_BUFFER_SIZE){tail = 0;}
		tx_buffer_tail = tail;
	}  
 	// Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
 	if(tail == tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
}

uint8_t serial_read()
{
	uint8_t tail = rx_buffer_tail; // Temporary rx_buffer_tail (to optimize for volatile)
	if(rx_buffer_head == tail)
	{
		return SERIAL_NO_DATA;
	}
	else
	{
		uint8_t data = rx_buffer[tail];
		tail++;
		if(tail == RX_BUFFER_SIZE){tail = 0;}
		rx_buffer_tail = tail;
    
    #ifdef ENABLE_XONXOFF
		if((get_rx_buffer_count() < RX_BUFFER_LOW) && flow_ctrl == XOFF_SENT)
		{ 
			flow_ctrl = SEND_XON;
			UCSR0B |=  (1 << UDRIE0); // Force TX
		}
    #endif
    
		return data;
	}
}

ISR(SERIAL_RX)
{
	uint8_t data = UDR0;
	uint8_t next_head;

	// Pick off runtime command characters directly from the serial stream. These characters are
	// not passed into the buffer, but these set system state flag bits for runtime execution.
	switch (data)
	{
		case CMD_STATUS_REPORT: sys.execute |= EXEC_STATUS_REPORT; break; // Set as true
		case CMD_CYCLE_START:   sys.execute |= EXEC_CYCLE_START; break; // Set as true
		case CMD_FEED_HOLD:     sys.execute |= EXEC_FEED_HOLD; break; // Set as true
		case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
		default: // Write character to buffer    
			next_head = rx_buffer_head + 1;
			if(next_head == RX_BUFFER_SIZE){next_head = 0;}

			// Write data to buffer unless it is full.
			if(next_head != rx_buffer_tail)
			{
				rx_buffer[rx_buffer_head] = data;
				rx_buffer_head = next_head;    

#ifdef ENABLE_XONXOFF
				if ((get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT)
				{
					flow_ctrl = SEND_XOFF;
					UCSR0B |=  (1 << UDRIE0); // Force TX
				} 
#endif
			}
			break;
	}
}

void serial_reset_read_buffer() 
{
	rx_buffer_tail = rx_buffer_head;

#ifdef ENABLE_XONXOFF
	flow_ctrl = XON_SENT;
#endif
}
