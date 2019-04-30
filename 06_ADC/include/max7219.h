/****************************************************************************\
 File:          max7219.h
 
 Description:   Header file for Max 7219 / 7221 serial common cathode
                LED driver.
 
 Known bugs/missing features:
 
 Modifications:
 Date                Comment
 ----    ------------------------------------------------
\****************************************************************************/

#ifndef _MAX7219_H_
#define _MAX7219_H_

/*********************** Registers                  *************************/

#define MAX7219_REG_NOP                 0x00	// No-Op
#define MAX7219_REG_ROW0                0x01	// Digit 0
#define MAX7219_REG_ROW1                0x02	// Digit 1
#define MAX7219_REG_ROW2                0x03	// Digit 2
#define MAX7219_REG_ROW3                0x04	// Digit 3
#define MAX7219_REG_ROW4                0x05	// Digit 4
#define MAX7219_REG_ROW5                0x06	// Digit 5
#define MAX7219_REG_ROW6                0x07	// Digit 6
#define MAX7219_REG_ROW7                0x08	// Digit 7
#define MAX7219_REG_DECODE_MODE         0x09	// Decode Mode
#define MAX7219_REG_INTENSITY           0x0A	// INTensity
#define MAX7219_REG_SCAN_LIMIT          0x0B	// Scan Limit
#define MAX7219_REG_SHUTDOWN            0x0C	// ShutDown
#define MAX7219_REG_DISPLAY_TEST		0x0F	// Display Test

// Shutdown Modes
// Set in MAX7219_REG_SHUTDOWN shutdown register

#define MAX7219_SHUTDOWN_MODE_ON        0x00	// Shutdown on (display off)
#define MAX7219_SHUTDOWN_MODE_OFF       0x01	// Shutdown off (display on)

// Decode Mode 
// set in MAX7219_REG_DECODE_MODE decode mode register

#define MAX7219_DECODE_MODE_OFF         0x00	// DecodeMode off

// Duty Cycles for brightness control
// Set in MAX7219_REG_INTENSITY intensity register

#define MAX7219_DUTY_CYCLE_1_32         0x00	// 1/32  DutyCycle
#define MAX7219_DUTY_CYCLE_3_32         0x01	// 3/32  DutyCycle
#define MAX7219_DUTY_CYCLE_5_32         0x02	// 5/32  DutyCycle
#define MAX7219_DUTY_CYCLE_7_32         0x03	// 7/32  DutyCycle
#define MAX7219_DUTY_CYCLE_9_32         0x04	// 9/32  DutyCycle
#define MAX7219_DUTY_CYCLE_11_32        0x05	// 11/32 DutyCycle
#define MAX7219_DUTY_CYCLE_13_32        0x06	// 13/32 DutyCycle
#define MAX7219_DUTY_CYCLE_15_32        0x07	// 15/32 DutyCycle
#define MAX7219_DUTY_CYCLE_17_32        0x08	// 17/32 DutyCycle
#define MAX7219_DUTY_CYCLE_19_32        0x09	// 19/32 DutyCycle
#define MAX7219_DUTY_CYCLE_21_32        0x0A	// 21/32 DutyCycle
#define MAX7219_DUTY_CYCLE_23_32        0x0B	// 23/32 DutyCycle
#define MAX7219_DUTY_CYCLE_25_32        0x0C	// 25/32 DutyCycle
#define MAX7219_DUTY_CYCLE_27_32        0x0D	// 27/32 DutyCycle
#define MAX7219_DUTY_CYCLE_29_32        0x0E	// 29/32 DutyCycle
#define MAX7219_DUTY_CYCLE_31_32        0x0F	// 31/32 DutyCycle

// Digits to Display
// Set in MAX7219_REG_SCANLIMIT scanlimit register

#define MAX7219_DISPLAY_DIGIT_0         0x00	// Display Digits 0
#define MAX7219_DISPLAY_DIGIT_0_1		0x01	// Display Digits 0 - 1
#define MAX7219_DISPLAY_DIGIT_0_2		0x02	// Display Digits 0 - 2
#define MAX7219_DISPLAY_DIGIT_0_3		0x03	// Display Digits 0 - 3
#define MAX7219_DISPLAY_DIGIT_0_4		0x04	// Display Digits 0 - 4
#define MAX7219_DISPLAY_DIGIT_0_5		0x05	// Display Digits 0 - 5
#define	MAX7219_DISPLAY_DIGIT_0_6		0x06	// Display Digits 0 - 6
#define MAX7219_DISPLAY_DIGIT_0_7		0x07	// Display Digits 0 - 7

// Display Test Modes
// Set in MAX7219_REG_DT Display Test register

#define MAX7219_DISPLAY_TEST_OFF		0x00	// Normal Operation
#define MAX7219_DISPLAY_TEST_ON         0x01	// Display Test Mode

// No Operation
// Set in MAX7219_REG_NOP No-Op register

#define MAX7219_NOP                     0x00	// No Op

// Misc Defines

#define MAX7219_ROWS					8	// # of display rows cntl'd by max


/*********************** function prototypes        *************************/

#endif	/* _MAX7219_H_ */
