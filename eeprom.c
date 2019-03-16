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
							//EEPROM����дʹ��
		#define EEMPE EEMWE //!< EEPROM master program/write enable.
							//EEPROM������дʹ��
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
��������:��EEPROM�ж�ȡ�ֽ�.�����Ӹ�����EEPROM��ַ�ж�ȡһ���ֽ�.
         ע��:�ڶ�ȡEEPROM��ϢʱCPUֹͣ4��ʱ������.
������Ϣ:  addr  ��EEPROM��ȡ��Ϣ�ĵ�ַ
����ֵ  :  ��EEPROM�ĵ�ַ�ж�ȡһ���ֽڵ���Ϣ
***********************************************************************************
*/
unsigned char eeprom_get_char( unsigned int addr )
{
	do {} while( EECR & (1<<EEPE) );// Wait for completion of previous write.
									// �ȴ���һ��д�������
	EEAR = addr; 					// Set EEPROM address register.
									// ����EEPROM��ַ�Ĵ���
	EECR = (1<<EERE);				// Start EEPROM read operation.
									// ��ʼִ��EEPROM������
	return EEDR;					// Return the byte read from EEPROM.
									// ����EEPROMд����,EEDR����Ҫд��EEAR��Ԫ������;���ڶ�����,EEDR�Ǵӵ�ַEEAR��ȡ������
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
��������:	��ָ����EEPROM��ַ��дһ���ֽڵ����ݡ������ǵ��ֽ���д����ֽڵĲ�֮ͬ��
				����ѡ�����Ч��EEPROM���ģʽ��
ע    ��:	����EEPROMд��ϢʱCPUֹͣ2��ʱ�����ڡ�
������Ϣ:	addr  ��EEPROM��ȡ��Ϣ�ĵ�ַ
�� �� ֵ:	��EEPROM�ĵ�ַ�ж�ȡһ���ֽڵ���Ϣ
***********************************************************************************
*/ 
void eeprom_put_char( unsigned int addr, unsigned char new_value )
{
	char old_value; // Old EEPROM value.
	char diff_mask; // Difference mask, i.e. old value XOR new value.

	//This part refer to "EECR �C The EEPROM Control Register" of ATMeag328P pdf.
	
	cli();          // Ensure atomic operation for the write operation.
				    // ����жϣ���һ�����������
	
	do {} while( EECR & (1<<EEPE) ); 	    // Wait for completion of previous write.
											// �ȴ���һ��д��������
	#ifndef EEPROM_IGNORE_SELFPROG
	do {} while( SPMCSR & (1<<SELFPRGEN) ); // Wait for completion of SPM.
                                            // SPMEN ��λ��SPM ָ���R1:R0 �е����ݴ洢����Z ָ��ȷ������ʱҳ��������Z ָ���LSB
                                            // �����ԡ�SPM ָ����ɣ������ĸ�ʱ��������û��SPM ָ�ִ��ʱ��SPMEN �Զ����㡣��ҳ��
                                            // ����ҳд������SPMEN ����Ϊ1ֱ��������ɡ�
	#endif
	
	EEAR = addr;        // Set EEPROM address register.
                        // ����EEPROM�ĵ�ַ�Ĵ���
	EECR = (1<<EERE);   // Start EEPROM read operation.
                        // ��λEERE�Ա㽫���ݶ���EEAR
	old_value = EEDR;   // Get old EEPROM value.
                        // ����EEPROMд������EEDR����Ҫд��EEAR��Ԫ�����ݣ�
                        // ���ڶ�������EEDR�Ǵӵ�ַEEAR��ȡ�����ݡ�
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
										// ������д��EEDR
			EECR = (1<<EEMPE) | 	// Set Master Write Enable bit...
										// ��λ����������EEPE�����ĸ�ʱ�������ڽ�����д��EEPROM
			       (0<<EEPM1) | (0<<EEPM0); // ...and Erase+Write mode.
			       					// ��EEPROM��д����ʱִ�в�����дģʽ�Ĳ���
			EECR |= (1<<EEPE);	// Start Erase+Write operation.
										// ��λEEPE��ʼ��EEPROMִ�в�����д����
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
				//�ж�ʹ�ܿ�
}

// Extensions added as part of Grbl 

// ��source�е�size������д��ӵ�ַdestination��ʼ��EEPROM��
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
// �ӵ�ַsource��ʼ��ȡsize���ֽ�,����ȡ���������뻺����destination��
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
	return(checksum == eeprom_get_char(source));	//��ȷ���TRUE,����ȷ���FALSE
}

// end of file
