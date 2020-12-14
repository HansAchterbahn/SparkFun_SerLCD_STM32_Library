/**
 * @file		SerLCD.h
 * @brief 		STM32 Library for SparkFun OpenLCD display based on HAL using I2C port
 * @date 		11.06.2019
 * @version		0.1.0
 * @author 		Mario Hesse
 * @copyright 	MIT License
 */

#ifndef INC_SERLCD_H_
#define INC_SERLCD_H_

#include "main.h"



#define DISPLAY_ADDRESS1 0x72 				// This is the default address of the OpenLCD
#define MAX_LINES 2							// display lines (1, 2 or 4)
#define MAX_COLUMNS 16						// display columns (16 or 20)

//OpenLCD command characters
#define SPECIAL_COMMAND 254  //Magic number for sending a special command
#define SETTING_COMMAND 0x7C //124, |, the pipe character: The command to change settings: baud, lines, width, backlight, splash, etc

//OpenLCD commands
#define CLEAR_COMMAND 0x2D					// 45, -, the dash character: command to clear and home the display
#define CONTRAST_COMMAND 0x18				// Command to change the contrast setting
#define ADDRESS_COMMAND 0x19				// Command to change the i2c address
#define SET_RGB_COMMAND 0x2B				// 43, +, the plus character: command to set backlight RGB value
#define ENABLE_SYSTEM_MESSAGE_DISPLAY 0x2E  // 46, ., command to enable system messages being displayed
#define DISABLE_SYSTEM_MESSAGE_DISPLAY 0x2F // 47, /, command to disable system messages being displayed
#define ENABLE_SPLASH_DISPLAY 0x30			// 48, 0, command to enable splash screen at power on
#define DISABLE_SPLASH_DISPLAY 0x31			// 49, 1, command to disable splash screen at power on
#define SAVE_CURRENT_DISPLAY_AS_SPLASH 0x0A // 10, Ctrl+j, command to save current text on display as splash

// special commands
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// errors
#define LCD_OK 0							// No error, everything's fine
#define LCD_ERROR 1							// Display error during transmission (i2c)


/* initial display functions */
uint8_t displayInit(I2C_HandleTypeDef *hi2c);

/* move on the display */
uint8_t displayHome();
uint8_t displaySetCursor(uint8_t col, uint8_t row);
uint8_t displayMoveCursorLeft();
uint8_t displayMoveCursorRight();
uint8_t displayScrollDisplayLeft();
uint8_t displayScrollDisplayRight();
uint8_t displayAutoscroll();
uint8_t displayNoAutoscroll();

/* write on the display */
uint8_t displayClear();
uint8_t displayWrite(uint8_t b);
uint8_t displayWriteString(char *buffer);
uint8_t displayCreateChar(uint8_t location, uint8_t *charmap);
uint8_t displayWriteChar(uint8_t location);

/* display align */
uint8_t displayLeftToRight();
uint8_t displayRightToLeft();

/* show and hide items */
uint8_t displayNoDisplay();
uint8_t displayDisplay();
uint8_t displayNoCursor();
uint8_t displayCursor();
uint8_t displayNoBlink();
uint8_t displayBlink();
uint8_t displayEnableSystemMessages();
uint8_t displayDisableSystemMessages();

/* display properties */
uint8_t displaySetBacklight(uint16_t r, uint16_t g, uint16_t b);
uint8_t displaySetFastBacklight(uint8_t r, uint8_t g, uint8_t b);
uint8_t displaySetContrast(uint8_t new_val);
uint8_t displaySetAddress(uint8_t new_addr);

/* edit splash screen */
uint8_t displayEnableSplash();
uint8_t displayDisableSplash();
uint8_t displaySaveSplash();


/* -------------------------------------------------------------------------- */


/* only for internal use - don't touch! */
uint8_t displayCommand(uint8_t command);
uint8_t displaySpecialCommand(uint8_t command);


/* -------------------------------------------------------------------------- */


#endif /* INC_SERLCD_H_ */
