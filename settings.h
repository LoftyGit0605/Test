/*
  settings.h - eeprom configuration handling 
  Part of Grbl

  The MIT License (MIT)

  GRBL(tm) - Embedded CNC g-code interpreter and motion-controller
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#ifndef settings_h
#define settings_h

#include <math.h>
#include "nuts_bolts.h"

#define GRBL_VERSION "0.8c"

// Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom
#define SETTINGS_VERSION            5

// Define bit flag masks for the boolean settings in settings.flag.
#define BITFLAG_REPORT_INCHES       bit(0)
#define BITFLAG_AUTO_START          bit(1)
#define BITFLAG_INVERT_ST_ENABLE    bit(2)
#define BITFLAG_HARD_LIMIT_ENABLE   bit(3)
#define BITFLAG_HOMING_ENABLE       bit(4)

// Define EEPROM memory address location values for Grbl settings and parameters
// NOTE: The Atmega328p has 1KB EEPROM. The upper half is reserved for parameters and
// the startup script. The lower half contains the global settings and space for future 
// developments.
// 注意: Atmega328p的EEPROM有1K。高512字节用于参数和启动脚本，低512字节包含全局参数和
// 备用的扩展区域。
#define EEPROM_ADDR_GLOBAL          1
#define EEPROM_ADDR_PARAMETERS      512
#define EEPROM_ADDR_STARTUP_BLOCK   768

// Define EEPROM address indexing for coordinate parameters
#define N_COORDINATE_SYSTEM         6                      // Number of supported work coordinate systems (from index 1)
#define SETTING_INDEX_NCOORD        N_COORDINATE_SYSTEM+1  // Total number of system stored (from index 0)
// NOTE: Work coordinate indices are (0=G54, 1=G55, ... , 6=G59)
#define SETTING_INDEX_G28           N_COORDINATE_SYSTEM    // Home position 1
#define SETTING_INDEX_G30           N_COORDINATE_SYSTEM+1  // Home position 2
// #define SETTING_INDEX_G92    N_COORDINATE_SYSTEM+2  // Coordinate offset (G92.2,G92.3 not supported)

// Global persistent settings (Stored from byte EEPROM_ADDR_GLOBAL onwards)
// 设置的全局变量(之前已经被存入EEPROM)
typedef struct {
	float    steps_per_mm[3];                 // steps_per_mm = (steps_per_revolution*microsteps)/mm_per_rev
	                                          // steps_per_mm = (360/步距角)*细分数/丝杆螺距
	uint8_t  microsteps;　　　　　　　　　　　// microsteps (细分数)
	uint8_t  pulse_microseconds;			  // step pulse, usec
	float    default_feed_rate;　　　　　　　 // default feed, mm/min
	float    default_seek_rate;　　　　　　　 // default seek, mm/min
	uint8_t  invert_mask;                     // step port invert mask, int:00011100
	float    mm_per_arc_segment;              // arc, mm/segment
	float    acceleration;                    // acceleration mm/s^2
	float    junction_deviation;              // junction deviation, mm
	uint8_t  flags;                           // Contains default boolean settings
	                                          // report inches | auto start | invert step enable | hard limits | homing cycle
	                                          // 标置: 英寸 | 自动启动 | 步进使能 | 硬限位使能 | 回零
	uint8_t  homing_dir_mask;                 // homing dir invert mask, int:00000000
	float    homing_feed_rate;                // homing feed, mm/min
	float    homing_seek_rate;                // homing seek, mm/min
	uint16_t homing_debounce_delay;           // homing debounce, msec(防抖动)
	float    homing_pulloff;                  // homing pull-off, mm
	uint8_t  stepper_idle_lock_time;          // If max value 255, steppers do not disable. step idle delay, msec
	                                          // Every time your steppers complete a motion and come to a stop, Grbl will disable
	                                          // the steppers by default. The stepper idle lock time is the time length Grbl will
	                                          // keep the steppers locked before disabling
	uint8_t  decimal_places;                  // n-decimals, int 小数点后有效数字位数
	uint8_t  n_arc_correction;                // n_arc圆弧拆分误差量
	//  uint8_t status_report_mask; // Mask to indicate desired report data.
} settings_t;
extern settings_t settings;

// Initialize the configuration subsystem (load settings from EEPROM)
void settings_init();

// A helper method to set new settings from command line
uint8_t settings_store_global_setting(int parameter, float value);

// Stores the protocol line variable as a startup line in EEPROM
void settings_store_startup_line(uint8_t n, char *line);

// Reads an EEPROM startup line to the protocol line variable
uint8_t settings_read_startup_line(uint8_t n, char *line);

// Writes selected coordinate data to EEPROM
void settings_write_coord_data(uint8_t coord_select, float *coord_data);

// Reads selected coordinate data from EEPROM
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data);

#endif
