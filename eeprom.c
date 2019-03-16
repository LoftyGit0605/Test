// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* \li File:               eeprom.c
* \li Compiler:           IAR EWAAVR 3.10c
* \li Support mail:       avr@atmel.com
*
* \li Supported devices:  All devices with split EEPROM erase/write
*                         capabilities can be used.
*                         The example is written for ATmega48.
*
* \li AppNote:            AVR103 - Using the EEPROM Programming Modes.
*
* \li Description:        Example on how to use the split EEPROM erase/write
*                         capabilities in e.g. ATmega48. All EEPROM
*                         programming modes are tested, i.e. Erase+Write,
*                         Erase-only and Write-only.
*
*                         $Revision: 1.6 $
*                         $Date: Friday, February 11, 2005 07:16:44 UTC $
****************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>

/* These EEPROM bits have different names on different devices. */
#ifndef EEPE
		#define EEPE  EEWE  //!< EEPROM program/write enable.
							//EEPROM程序写使能
		#define EEMPE EEMWE //!< EEPROM master program/write enable.
							//EEPROM主程序写使能
#endif

/* These two are unfortunately not defined in the device include files. */
#define EEPM1 5 //!< EEPROM Programming Mode Bit 1.
#define EEPM0 4 //!< EEPROM Programming Mode Bit 0.

/* Define to reduce code size. */
#define EEPROM_IGNORE_SELFPROG //!< Remove SPM flag polling.

/*! \brief  Read byte from EEPROM.
 *
 *  This function reads one byte from a given EEPROM address.
 *
 *  \note  The CPU is halted for 4 clock cycles during EEPROM read.
 *
 *  \param  addr  EEPROM address to read from.
 *  \return  The byte read from the EEPROM address.
 */

/*
***********************************************************************************
函数功能:从EEPROM中读取字节.函数从给定的EEPROM地址中读取一个字节.
         注意:在读取EEPROM信息时CPU停止4个时钟周期.
参数信息:  addr  从EEPROM读取信息的地址
返回值  :  从EEPROM的地址中读取一个字节的信息
***********************************************************************************
*/
unsigned char eeprom_get_char( unsigned int addr )
{
	do {} while( EECR & (1<<EEPE) );// Wait for completion of previous write.
									// 等待上一次写操作完成
	EEAR = addr; 					// Set EEPROM address register.
									// 设置EEPROM地址寄存器
	EECR = (1<<EERE);				// Start EEPROM read operation.
									// 开始执行EEPROM读操作
	return EEDR;					// Return the byte read from EEPROM.
									// 对于EEPROM写操作,EEDR是需要写到EEAR单元的数据;对于读操作,EEDR是从地址EEAR读取的数据
}

/*! \brief  Write byte to EEPROM.
 *
 *  This function writes one byte to a given EEPROM address.
 *  The differences between the existing byte and the new value is used
 *  to select the most efficient EEPROM programming mode.
 *
 *  \note  The CPU is halted for 2 clock cycles during EEPROM programming.
 *
 *  \note  When this function returns, the new EEPROM value is not available
 *         until the EEPROM programming time has passed. The EEPE bit in EECR
 *         should be polled to check whether the programming is finished.
 *
 *  \note  The EEPROM_GetChar() function checks the EEPE bit automatically.
 *
 *  \param  addr  EEPROM address to write to.
 *  \param  new_value  New EEPROM value.
 */

/*
***********************************************************************************
函数功能:	向指定的EEPROM地址中写一个字节的数据。被覆盖的字节与写入的字节的不同之处
				在于选用最高效的EEPROM编程模式。
注    意:	在向EEPROM写信息时CPU停止2个时钟周期。
参数信息:	addr  从EEPROM读取信息的地址
返 回 值:	从EEPROM的地址中读取一个字节的信息
***********************************************************************************
*/ 
void eeprom_put_char( unsigned int addr, unsigned char new_value )
{
	char old_value; // Old EEPROM value.
	char diff_mask; // Difference mask, i.e. old value XOR new value.

	//This part refer to "EECR – The EEPROM Control Register" of ATMeag328P pdf.
	
	cli();          // Ensure atomic operation for the write operation.
				    // 清除中断，由一条汇编语句完成
	
	do {} while( EECR & (1<<EEPE) ); 	    // Wait for completion of previous write.
											// 等待上一次写操作结束
	#ifndef EEPROM_IGNORE_SELFPROG
	do {} while( SPMCSR & (1<<SELFPRGEN) ); // Wait for completion of SPM.
                                            // SPMEN 置位，SPM 指令将把R1:R0 中的数据存储到由Z 指针确定的临时页缓冲器。Z 指针的LSB
                                            // 被忽略。SPM 指令完成，或在四个时钟周期内没有SPM 指令被执行时，SPMEN 自动清零。在页擦
                                            // 除和页写过程中SPMEN 保持为1直到操作完成。
	#endif
	
	EEAR = addr;        // Set EEPROM address register.
                        // 设置EEPROM的地址寄存器
	EECR = (1<<EERE);   // Start EEPROM read operation.
                        // 置位EERE以便将数据读入EEAR
	old_value = EEDR;   // Get old EEPROM value.
                        // 对于EEPROM写操作，EEDR是需要写到EEAR单元的数据；
                        // 对于读操作，EEDR是从地址EEAR读取的数据。
	diff_mask = old_value ^ new_value; // Get bit differences.
	
	// Check if any bits are changed to '1' in the new value.
	if( diff_mask & new_value ) 
	{
		// Now we know that _some_ bits need to be erased to '1'.
		
		// Check if any bits in the new value are '0'.
		if( new_value != 0xff ) 
		{
			// Now we know that some bits need to be programmed to '0' also.
			
			EEDR = new_value;	// Set EEPROM data register.
										// 将数据写入EEDR
			EECR = (1<<EEMPE) | 	// Set Master Write Enable bit...
										// 置位用于在设置EEPE后在四个时钟周期内将数据写入EEPROM
			       (0<<EEPM1) | (0<<EEPM0); // ...and Erase+Write mode.
			       					// 向EEPROM中写数据时执行擦除和写模式的操作
			EECR |= (1<<EEPE);	// Start Erase+Write operation.
										// 置位EEPE开始向EEPROM执行擦除与写操作
		} 
		else 
		{
			// Now we know that all bits should be erased.

			EECR = (1<<EEMPE) | // Set Master Write Enable bit...
			       (1<<EEPM0);  // ...and Erase-only mode.
			EECR |= (1<<EEPE);  // Start Erase-only operation.
		}
	} 
	else 
	{	
		// Now we know that _no_ bits need to be erased to '1'.
		
		// Check if any bits are changed from '1' in the old value.
		if( diff_mask ) 
		{
			// Now we know that _some_ bits need to the programmed to '0'.
			
			EEDR = new_value;	// Set EEPROM data register.
			EECR = (1<<EEMPE) |	// Set Master Write Enable bit...
			       (1<<EEPM1);	// ...and Write-only mode.
			EECR |= (1<<EEPE);	// Start Write-only operation.
		}
	}
	
	sei();	// Restore interrupt flag state.
				//中断使能开
}

// Extensions added as part of Grbl 

// 将source中的size个数据写入从地址destination开始的EEPROM中
void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) 
{
	unsigned char checksum = 0;
	for(; size > 0; size--) 
	{ 
		checksum = (checksum << 1) || (checksum >> 7);
		checksum += *source;
		eeprom_put_char(destination++, *(source++)); 
	}
	eeprom_put_char(destination, checksum);
}
// 从地址source开始读取size个字节,将读取的数据送入缓冲区destination中
int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) 
{
	unsigned char data, checksum = 0;
	for(; size > 0; size--) 
	{ 
		data = eeprom_get_char(source++);
		checksum = (checksum << 1) || (checksum >> 7);
		checksum += data;    
		*(destination++) = data; 
	}
	return(checksum == eeprom_get_char(source));	//相等返回TRUE,不相等返回FALSE
}

// end of file
