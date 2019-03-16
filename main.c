/*
*********************************************************************************
	main.c - An embedded CNC Controller with rs274/ngc (g-code) support
	Part of Grbl

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
//声明系统的全局结构体
system_t sys; 

int main(void)
{
	// Initialize system
	serial_init(); 		// Setup serial baud rate and interrupts
						// 设置串口波特率并启动串口中断
	settings_init();	// Load grbl settings from EEPROM
						// 从EEPROM中加载GRBL系统数据
	st_init();			// Setup stepper pins and interrupt timers
						// 设置步进端口并开启定时中断
	sei(); 				// Enable interrupts
						// 中断使能

	memset(&sys, 0, sizeof(sys));	// Clear all system variables
									// 清除所有系统变量
	sys.abort = true;				// Set abort to complete initialization
									// 完成初始化设置中止
	sys.state = STATE_INIT;			// Set alarm state to indicate unknown initial position
									// 设置报警状态显示未知的初始位置

	for(;;)
	{		
		// Execute system reset upon a system abort, where the main program will return to this loop.
		// Once here, it is safe to re-initialize the system. At startup, the system will automatically
		// reset to finish the initialization process.
		// 在系统中止时会执行系统复位,在这种情况系统主程序会返回此循环程序.一旦运行,系统将再次初始化.在
		// 启动时,系统会自动复位并执行初始化程序.
		if (sys.abort) 
		{
	  		// Reset system.
			serial_reset_read_buffer();	// Clear serial read buffer
										// 清除串口接收缓冲区
			plan_init();				// Clear block buffer and planner variables
										// 清空预处理缓存区与相关变量
			gc_init(); 					// Set g-code parser to default state
										// 设置G代码解析器的默认状态
			protocol_init(); 			// Clear incoming line data and execute startup lines
										// 清除插入行数据并执行启动行
			spindle_init();				// 主轴初始化
			coolant_init();				// 冷却初始化
			limits_init();				// 限位初始化
			st_reset(); 				// Clear stepper subsystem variables.

			// Sync cleared gcode and planner positions to current system position, which is only
			// cleared upon startup, not a reset/abort. 
			// 当系统启动时同时将当前位置设置到G代码与平面位置上，复位与异常终止时不需要。
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
