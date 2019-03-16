/*
*********************************************************************************
	main.c - An embedded CNC Controller with rs274/ngc (g-code) support
	Part of Grbl

	The MIT License (MIT)

	GRBL(tm) - Embedded CNC g-code interpreter and motion-controller
	GRBL(tm) - Ƕ��ʽCNC G����������˶�������
	Copyright (c) 2009-2011 Simen Svale Skogsrud
	Copyright (c) 2011-2012 Sungeun K. Jeon

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:
	���֤�����������:�κ��˶�������ѻ�ȡ������������ص��ĵ�,����û�����Ƶ�
	ʹ�ø����,����û�����Ƶ�ʹ��,����,�޸�,�ϲ�,ӡˢ,ɢ��,�������֤�����۸����
	.Ҳ����ʹ�ø�����������ѵ�װ��.���������½���:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.
	������Ȩ�����ͱ��������Ӧ�����������и�����ʵ���Բ��ֵ����.

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
	����ṩ��"��",û���κ�������ʾ��ʾ�ı�֤,�����������������Եı�֤,�ض�Ŀ��
	�����ַ���������֤.���κ������,���ߺͰ�Ȩ�����߲��е��κ�����,�𺦻���������
	,���������к�Լ������Ȩ�������,ԭ������,��������ʹ�ø������ʹ�ø��������
	��������.
***********************************************************************************
*/

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "config.h"
#include "planner.h"
#include "nuts_bolts.h"
#include "stepper.h"
#include "spindle_control.h"
#include "coolant_control.h"
#include "motion_control.h"
#include "gcode.h"
#include "protocol.h"
#include "limits.h"
#include "report.h"
#include "settings.h"
#include "serial.h"

//Declare system global variable structure
//����ϵͳ��ȫ�ֽṹ��
system_t sys; 

int main(void)
{
	// Initialize system
	serial_init(); 		// Setup serial baud rate and interrupts
						// ���ô��ڲ����ʲ����������ж�
	settings_init();	// Load grbl settings from EEPROM
						// ��EEPROM�м���GRBLϵͳ����
	st_init();			// Setup stepper pins and interrupt timers
						// ���ò����˿ڲ�������ʱ�ж�
	sei(); 				// Enable interrupts
						// �ж�ʹ��

	memset(&sys, 0, sizeof(sys));	// Clear all system variables
									// �������ϵͳ����
	sys.abort = true;				// Set abort to complete initialization
									// ��ɳ�ʼ��������ֹ
	sys.state = STATE_INIT;			// Set alarm state to indicate unknown initial position
									// ���ñ���״̬��ʾδ֪�ĳ�ʼλ��

	for(;;)
	{		
		// Execute system reset upon a system abort, where the main program will return to this loop.
		// Once here, it is safe to re-initialize the system. At startup, the system will automatically
		// reset to finish the initialization process.
		// ��ϵͳ��ֹʱ��ִ��ϵͳ��λ,���������ϵͳ������᷵�ش�ѭ������.һ������,ϵͳ���ٴγ�ʼ��.��
		// ����ʱ,ϵͳ���Զ���λ��ִ�г�ʼ������.
		if (sys.abort) 
		{
	  		// Reset system.
			serial_reset_read_buffer();	// Clear serial read buffer
										// ������ڽ��ջ�����
			plan_init();				// Clear block buffer and planner variables
										// ���Ԥ������������ر���
			gc_init(); 					// Set g-code parser to default state
										// ����G�����������Ĭ��״̬
			protocol_init(); 			// Clear incoming line data and execute startup lines
										// ������������ݲ�ִ��������
			spindle_init();				// �����ʼ��
			coolant_init();				// ��ȴ��ʼ��
			limits_init();				// ��λ��ʼ��
			st_reset(); 				// Clear stepper subsystem variables.

			// Sync cleared gcode and planner positions to current system position, which is only
			// cleared upon startup, not a reset/abort. 
			// ��ϵͳ����ʱͬʱ����ǰλ�����õ�G������ƽ��λ���ϣ���λ���쳣��ֹʱ����Ҫ��
			sys_sync_current_position();

			// Reset system variables.
			sys.abort = false;
			sys.execute = 0;
			if (bit_istrue(settings.flags,BITFLAG_AUTO_START)) { sys.auto_start = true; }

			// Check for power-up and set system alarm if homing is enabled to force homing cycle
			// by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
			// startup scripts, but allows access to settings and internal commands. Only a homing
			// cycle '$H' or kill alarm locks '$X' will disable the alarm.
			// NOTE: The startup script will run after successful completion of the homing cycle, but
			// not after disabling the alarm locks. Prevents motion startup blocks from crashing into
			// things uncontrollably. Very bad.
		#ifdef HOMING_INIT_LOCK
			if (sys.state == STATE_INIT && bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
		#endif
	  
			// Check for and report alarm state after a reset, error, or an initial power up.
			if (sys.state == STATE_ALARM) 
			{
				report_feedback_message(MESSAGE_ALARM_LOCK); 
			} 
			else 
			{
				// All systems go. Set system to ready and execute startup script.
				sys.state = STATE_IDLE;
				protocol_execute_startup(); 
			}
		}
		protocol_execute_runtime();
		protocol_process(); // ... process the serial protocol
	}
	return 0;   /* never reached */
}
