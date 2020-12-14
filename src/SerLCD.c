/**
 * @file		SerLCD.c
 * @brief 		STM32 Library for SparkFun OpenLCD display based on HAL using I2C port
 * @date 		11.06.2019
 * @version		0.1.0
 * @author 		Mario Hesse
 * @copyright 	MIT License
 *
 *
 * Original information copied form "LCD library for SparkFun RGB 3.3v Serial Open LCD display"
 * --------------------------------------------------------------------------------------------
 * LCD library for SparkFun RGB 3.3v Serial Open LCD display
 * with an attached Qwiic adapter.
 *
 * By: Gaston R. Williams
 * Date: August 22, 2018
 *
 * License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 *
 * This library is based heavily on the LiquidCrystal_I2C library and the sample code provided with
 * the SparkFun Serial OpenLCD display.  The original LiquidCrystal library by David A. Mellis and
 * modified by Limor Fried and the OpenLCD code by Nathan Seidle at SparkFun.
 *
 * The LiquidCrystal_I2C library was based on the work by DFRobot.
 * (That's the only attribution I found in the code I have. If anyone can provide better information,
 * Plese let me know and I'll be happy to give credit where credit is due.)
 *
 * Original information copied from OpenLCD:
 *
 * The OpenLCD display information is based based on code by
 * Nathan Seidle
 * SparkFun Electronics
 * Date: April 19th, 2015
 *
 * License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 * OpenLCD gives the user multiple interfaces (serial, I2C, and SPI) to control an LCD. SerLCD was the original
 * serial LCD from SparkFun that ran on the PIC 16F88 with only a serial interface and limited feature set.
 * This is an updated serial LCD.
 *
 * Please Note: 0x72 is the 7-bit I2C address. If you are using a different language than Arduino you will probably
 * need to add the Read/Write bit to the end of the address. This means the default read address for the OpenLCD
 * is 0b.1110.0101 or 0xE5 and the write address is 0b.1110.0100 or 0xE4.
 * For more information see https://learn.sparkfun.com/tutorials/i2c
 * Note: This code expects the display to be listening at the default I2C address. If your display is not at 0x72, you can
 * do a hardware reset. Tie the RX pin to ground and power up OpenLCD. You should see the splash screen
 * then "System reset Power cycle me" and the backlight will begin to blink. Now power down OpenLCD and remove
 * the RX/GND jumper. OpenLCD is now reset.
 *
 * To get this code to work, attach a Qwiic adapter to an OpenLCD. Use the Qwiic cable to attach adapter to a SparkFun Blackboard or
 * an Arduino Uno with the Qwiic shield.
 *
 * The OpenLCD has 4.7k pull up resistors on SDA and SCL. If you have other devices on the
 * I2C bus then you may want to disable the pull up resistors by clearing the PU (pull up) jumper.
 * OpenLCD will work at 400kHz Fast I2C. Use the .setClock() call shown below to set the data rate
 * faster if needed.
 * Command cheat sheet:
 * ASCII / DEC / HEX
 * '|'    / 124 / 0x7C - Put into setting mode
 * Ctrl+c / 3 / 0x03 - Change width to 20
 * Ctrl+d / 4 / 0x04 - Change width to 16
 * Ctrl+e / 5 / 0x05 - Change lines to 4
 * Ctrl+f / 6 / 0x06 - Change lines to 2
 * Ctrl+g / 7 / 0x07 - Change lines to 1
 * Ctrl+h / 8 / 0x08 - Software reset of the system
 * Ctrl+i / 9 / 0x09 - Enable/disable splash screen
 * Ctrl+j / 10 / 0x0A - Save currently displayed text as splash
 * Ctrl+k / 11 / 0x0B - Change baud to 2400bps
 * Ctrl+l / 12 / 0x0C - Change baud to 4800bps
 * Ctrl+m / 13 / 0x0D - Change baud to 9600bps
 * Ctrl+n / 14 / 0x0E - Change baud to 14400bps
 * Ctrl+o / 15 / 0x0F - Change baud to 19200bps
 * Ctrl+p / 16 / 0x10 - Change baud to 38400bps
 * Ctrl+q / 17 / 0x11 - Change baud to 57600bps
 * Ctrl+r / 18 / 0x12 - Change baud to 115200bps
 * Ctrl+s / 19 / 0x13 - Change baud to 230400bps
 * Ctrl+t / 20 / 0x14 - Change baud to 460800bps
 * Ctrl+u / 21 / 0x15 - Change baud to 921600bps
 * Ctrl+v / 22 / 0x16 - Change baud to 1000000bps
 * Ctrl+w / 23 / 0x17 - Change baud to 1200bps
 * Ctrl+x / 24 / 0x18 - Change the contrast. Follow Ctrl+x with number 0 to 255. 120 is default.
 * Ctrl+y / 25 / 0x19 - Change the TWI address. Follow Ctrl+x with number 0 to 255. 114 (0x72) is default.
 * Ctrl+z / 26 / 0x1A - Enable/disable ignore RX pin on startup (ignore emergency reset)
 * '+'    / 43 / 0x2B - Set RGB backlight with three following bytes, 0-255
 * ','    / 44 / 0x2C - Display current firmware version
 * '-'    / 45 / 0x2D - Clear display. Move cursor to home position.
 * '.'    / 46 / 0x2E - Enable system messages (ie, display 'Contrast: 5' when changed)
 * '/'    / 47 / 0x2F - Disable system messages (ie, don't display 'Contrast: 5' when changed)
 * '0'    / 48 / 0x30 - Enable splash screen
 * '1'    / 49 / 0x31 - Disable splash screen
 *        / 128-157 / 0x80-0x9D - Set the primary backlight brightness. 128 = Off, 157 = 100%.
 *        / 158-187 / 0x9E-0xBB - Set the green backlight brightness. 158 = Off, 187 = 100%.
 *        / 188-217 / 0xBC-0xD9 - Set the blue backlight brightness. 188 = Off, 217 = 100%.
 *		For example, to change the baud rate to 115200 send 124 followed by 18.
 *
 */

#include "SerLCD.h"
#include "main.h"




uint8_t _i2cAddr = DISPLAY_ADDRESS1;
I2C_HandleTypeDef *_i2cHandler;
uint8_t _displayControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
uint8_t _displayMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

/*
 *******************************************************************************
 * initial display functions
 *******************************************************************************
 */


/**
 * @brief	Initial start of the display - defines the maximum rows and
 *  		columns on the display refered to
 *  		• MAX_ROWS and
 *  		• MAX_COLUMNS
 * @param	hi2c 	- pointer to the i2c handler
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayInit(I2C_HandleTypeDef *hi2c)
{
	uint8_t retval = LCD_OK;

	_i2cHandler = hi2c;			// pointer for i2c handler is stored in library

	// create i2c data stream
    uint8_t TransmitData[6] = {SPECIAL_COMMAND,									// special command character
    						   LCD_DISPLAYCONTROL | _displayControl, 			// display command
							   SPECIAL_COMMAND, 								// command character
							   LCD_ENTRYMODESET | _displayMode, 				// entry mode command
							   SETTING_COMMAND, 								// Put LCD into setting mode
							   CLEAR_COMMAND 									// clear display
    						   };

    // transmission of data stream
    if(HAL_I2C_Master_Transmit(_i2cHandler, DISPLAY_ADDRESS1<<1, TransmitData, sizeof(TransmitData), 100) != HAL_OK)		// transmit data
    	retval = LCD_ERROR;

    HAL_Delay(50);

    return retval;
}



/*
 *******************************************************************************
 * move on the display
 *******************************************************************************
 */


/**
 * @brief	Send the home command to the display.  This returns the cursor
 * 			to return to the beginning of the display, without clearing
 * 			the display.
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayHome()
{
	return displayCommand(LCD_RETURNHOME);
}



/**
 * @brief	Set the cursor position to a particular column and row.
 * @param	col 	- column number (0...19)
 * @param	row  	- row number (0...3)
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displaySetCursor(uint8_t col, uint8_t row)
{
  uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};

  //kepp variables in bounds
  //row = max(0, row);            //row cannot be less than 0
  //row = min(row, MAX_ROWS - 1); //row cannot be greater than max rows

  //send the command
  return displaySpecialCommand(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}



/**
 * @brief	Move the cursor one character to the left.
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayMoveCursorLeft()
{
  return displaySpecialCommand(LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVELEFT);
}



/**
 * @brief	Move the cursor one character to the left.
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayMoveCursorRight()
{
  return displaySpecialCommand(LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVERIGHT);
}



/**
 * @brief	Scroll the display one character to the left, without
 * 			changing the text
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayScrollDisplayLeft()
{
  return displaySpecialCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}



/**
 * @brief	Scroll the display one character to the right, without
 * 			changing the text
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayScrollDisplayRight()
{
  return displaySpecialCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}



/**
 * @brief  	Turn autoscrolling on. This will 'right justify' text from
 * 			the cursor.
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayAutoscroll()
{
	 _displayMode |= LCD_ENTRYSHIFTINCREMENT;
	 return displaySpecialCommand(LCD_ENTRYMODESET | _displayMode);
}



/**
 * @brief  	Turn autoscrolling off.
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayNoAutoscroll()
{
	 _displayMode &= ~LCD_ENTRYSHIFTINCREMENT;
	 return displaySpecialCommand(LCD_ENTRYMODESET | _displayMode);
}



/*
 *******************************************************************************
 * write on the display
 *******************************************************************************
 */


/**
 * @brief	Send the clear command to the display.  This clears the
 * 			display and forces the cursor to return to the beginning
 * 			of the display.
 * @retval	status	- 0...OK, other...error
 */
uint8_t displayClear()
{
	uint8_t retval = displayCommand(CLEAR_COMMAND);
	HAL_Delay(10);
	return retval;
}



/**
 * @brief	Write a byte to the display
 * @param	b 		- byte to write
 * @retval	status	- 0...OK, other...error
 *
 * Required for Print.
 */
uint8_t displayWrite(uint8_t b)
{
	uint8_t retval = LCD_OK;

	// transmission of data stream
	if(HAL_I2C_Master_Transmit(_i2cHandler, DISPLAY_ADDRESS1<<1, b, 1, 100) != HAL_OK)
	  retval = LCD_ERROR;
	HAL_Delay(10); // wait a bit

    return retval;
}



/**
 * @brief	Write a character buffer to the display.
 * @param	*buffer - pointer to the string to write
 * @param	size  	- length of the string to write
 * @retval	status 	- 0...OK, other...error
 *
 */
uint8_t displayWriteString(char *buffer)
{
	uint8_t retval = LCD_OK;

	// transmission of data stream
	if(HAL_I2C_Master_Transmit(_i2cHandler, DISPLAY_ADDRESS1<<1, buffer, strlen(buffer), 100) != HAL_OK)		// transmit data
	  retval = LCD_ERROR;
	HAL_Delay(10); //This takes a bit longer

    return retval;
}



/**
 * @brief	Create a customer character
 * @param	location - character number 0 to 7
 * @param	charmap  - byte array of 8 for character
 * @retval	status   - 0...OK, other...error
 */
uint8_t displayCreateChar(uint8_t location, uint8_t *charmap)
{
  location &= 0x7; // we only have 8 locations 0-7

  // create i2c data stream
  uint8_t retval = LCD_OK;
  uint8_t TransmitData[10] = {SETTING_COMMAND,
		  	  	  	  	  	 27 + location,
  	  	  	  	  	  	  	 charmap[0],
  	  	  	  	  	  	  	 charmap[1],
  	  	  	  	  	  	  	 charmap[2],
  	  	  	  	  	  	  	 charmap[3],
  	  	  	  	  	  	  	 charmap[4],
  	  	  	  	  	  	  	 charmap[5],
  	  	  	  	  	  	  	 charmap[6],
  	  	  	  	  	  	  	 charmap[7]
  	  	  	  	  	  	  	 };

  // transmission of data stream
  if(HAL_I2C_Master_Transmit(_i2cHandler, DISPLAY_ADDRESS1<<1, TransmitData, sizeof(TransmitData), 100) != HAL_OK)		// transmit data
	  retval = LCD_ERROR;
  HAL_Delay(50); //This takes a bit longer

  return retval;
}



/**
 * @brief	Write a customer character to the display
 * @param	location	- character number 0 to 7
 * @retval	status		- 0...OK, other...error
 */
uint8_t displayWriteChar(uint8_t location)
{
  location &= 0x7; // we only have 8 locations 0-7

  return displayCommand(35 + location);
}



/*
 *******************************************************************************
 * display align
 *******************************************************************************
 */


/**
 * @brief  	Set the text to flow from left to right.  This is the direction
 * 			that is common to most Western languages.
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayLeftToRight()
{
	_displayMode |= LCD_ENTRYLEFT;
	return displaySpecialCommand(LCD_ENTRYMODESET | _displayMode);
}



/**
 * @brief  	Set the text to flow from right to left.
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayRightToLeft()
{
	 _displayMode &= ~LCD_ENTRYLEFT;
	 return displaySpecialCommand(LCD_ENTRYMODESET | _displayMode);
}



/*
 *******************************************************************************
 * show and hide items
 *******************************************************************************
 */


/**
 * @brief	Turn the display off quickly.
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayNoDisplay()
{
	  _displayControl &= ~LCD_DISPLAYON;
	  return displaySpecialCommand(LCD_DISPLAYCONTROL | _displayControl);
}



/**
 * @brief	Turn the display on quickly.
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayDisplay()
{
	  _displayControl |= LCD_DISPLAYON;
	  return displaySpecialCommand(LCD_DISPLAYCONTROL | _displayControl);
}



/**
 * @brief	Turn the underline cursor off.
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayNoCursor()
{
  _displayControl &= ~LCD_CURSORON;
  return displaySpecialCommand(LCD_DISPLAYCONTROL | _displayControl);
}



/**
 * @brief	Turn the underline cursor on.
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayCursor()
{
  _displayControl |= LCD_CURSORON;
  return displaySpecialCommand(LCD_DISPLAYCONTROL | _displayControl);
}




/**
 * @brief	Turn the blink cursor off.
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayNoBlink()
{
  _displayControl &= ~LCD_BLINKON;
  return displaySpecialCommand(LCD_DISPLAYCONTROL | _displayControl);
}



/**
 * @brief	Turn the blink cursor on.
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayBlink()
{
  _displayControl |= LCD_BLINKON;
  return displaySpecialCommand(LCD_DISPLAYCONTROL | _displayControl);
}



/**
 * @brief  	This allows user to see printing messages like 'UART: 57600'
 * 			and 'Contrast: 5.
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayEnableSystemMessages()
{
	uint8_t retval = LCD_OK;

	// create i2c data stream
    uint8_t TransmitData[2] = {SETTING_COMMAND,									// special command character
    						   ENABLE_SYSTEM_MESSAGE_DISPLAY, 					// set '.' character
							   };

    // transmission of data stream
    if(HAL_I2C_Master_Transmit(_i2cHandler, DISPLAY_ADDRESS1<<1, TransmitData, sizeof(TransmitData), 100) != HAL_OK)		// transmit data
    	retval = LCD_ERROR;
    HAL_Delay(10);

	return retval;
}



/**
 * @brief  	This allows user to disable printing messages like 'UART: 57600'
 * 			and 'Contrast: 5'
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayDisableSystemMessages()
{
	uint8_t retval = LCD_OK;

	// create i2c data stream
    uint8_t TransmitData[2] = {SETTING_COMMAND,									// special command character
    						   DISABLE_SYSTEM_MESSAGE_DISPLAY, 					// set '.' character
							   };

    // transmission of data stream
    if(HAL_I2C_Master_Transmit(_i2cHandler, DISPLAY_ADDRESS1<<1, TransmitData, sizeof(TransmitData), 100) != HAL_OK)		// transmit data
    	retval = LCD_ERROR;
    HAL_Delay(10);

	return retval;
}



/*
 *******************************************************************************
 * display properties
 *******************************************************************************
 */


/**
 * @brief  	Uses a standard rgb byte triplit eg. (255, 0, 255) to
 * 			set the backlight color.
 * @param  	r 		- 0...255
 * @param  	g 		- 0...255
 * @param  	b 		- 0...255
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displaySetBacklight(uint16_t r, uint16_t g, uint16_t b)
{
	uint8_t retval = LCD_OK;

	// size reduction of the percentage to a value between 0 and 29
	if(r != 0)
		r = r*29/255;
	if(g != 0)
		g = g*29/255;
	if(b != 0)
		b = b*29/255;

	// create i2c data stream
    uint8_t TransmitData[10] = {SPECIAL_COMMAND,								//Send special command character
    						    LCD_DISPLAYCONTROL |(_displayControl & ~LCD_DISPLAYON), // turn display off
    						    SETTING_COMMAND, (128 + (uint8_t)r),						// red: 0...100% ~ 0...29
    						    SETTING_COMMAND, (158 + (uint8_t)g), 					// green: 0...100% ~ 0...29
							    SETTING_COMMAND, (188 + (uint8_t)b), 					// blue: 0...100% ~ 0...29
								SPECIAL_COMMAND,                      			//Send special command character
								LCD_DISPLAYCONTROL | (_displayControl |= LCD_DISPLAYON)}; // turn display off as before

    // transmission of data stream
    if(HAL_I2C_Master_Transmit(_i2cHandler, DISPLAY_ADDRESS1<<1, TransmitData, sizeof(TransmitData), 100) != HAL_OK)		// transmit data
    	retval = LCD_ERROR;
    HAL_Delay(50);

	return retval;
}


/**
 * @brief  	New faster backlight function - Uses a standard rgb byte triplit
 * 			eg. (255, 0, 255) to set the backlight color.
 * @param  	r 		- 0...255
 * @param  	g 		- 0...255
 * @param  	b 		- 0...255
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displaySetFastBacklight(uint8_t r, uint8_t g, uint8_t b)
{
	uint8_t retval = LCD_OK;

	// create i2c data stream
    uint8_t TransmitData[5] = {SETTING_COMMAND,									// special command character
    						   SET_RGB_COMMAND, 								// set RGB character '+' or plus
							   r,												// red value
							   g,												// green value
							   b												// blue value
    						   };

    // transmission of data stream
    if(HAL_I2C_Master_Transmit(_i2cHandler, DISPLAY_ADDRESS1<<1, TransmitData, sizeof(TransmitData), 100) != HAL_OK)		// transmit data
    	retval = LCD_ERROR;
    HAL_Delay(10);

	return retval;
}



/**
 * @brief  	Change the contrast from 0 to 255. 120 is default.
 * @param	new_val - new contrast value
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displaySetContrast(uint8_t new_val)
{
	uint8_t retval = LCD_OK;

	// create i2c data stream
    uint8_t TransmitData[3] = {SETTING_COMMAND,									// special command character
    						   CONTRAST_COMMAND, 								// send contrast command
							   new_val											// set new contrast value
							   };

    // transmission of data stream
    if(HAL_I2C_Master_Transmit(_i2cHandler, DISPLAY_ADDRESS1<<1, TransmitData, sizeof(TransmitData), 100) != HAL_OK)		// transmit data
    	retval = LCD_ERROR;
    HAL_Delay(10);

	return retval;
}



/*
 * Change the I2C Address. 0x72 is the default.
 * Note that this change is persistent.  If anything
 * goes wrong you may need to do a hardware reset
 * to unbrick the display.
 *
 * byte new_addr - new i2c address
 */
uint8_t displaySetAddress(uint8_t new_addr)
{
	uint8_t retval = LCD_OK;

	// create i2c data stream
    uint8_t TransmitData[3] = {SETTING_COMMAND,									// special command character
    						   ADDRESS_COMMAND, 								// send address command
							   new_addr											// set new address
							   };

    // transmission of data stream
    if(HAL_I2C_Master_Transmit(_i2cHandler, DISPLAY_ADDRESS1<<1, TransmitData, sizeof(TransmitData), 100) != HAL_OK)		// transmit data
    	retval = LCD_ERROR;

    //Update our own address so we can still talk to the display
    if(retval == LCD_OK)
    	_i2cAddr = new_addr;

    HAL_Delay(50);	 //This may take awhile

	return retval;
}



/*
 *******************************************************************************
 * edit splash screen
 *******************************************************************************
 */


/**
 * @brief  	Enable splash screen at power on
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayEnableSplash()
{
	uint8_t retval = LCD_OK;

	// create i2c data stream
    uint8_t TransmitData[2] = {SETTING_COMMAND,									// special command character
    						   ENABLE_SPLASH_DISPLAY, 							// set '.' character
							   };

    // transmission of data stream
    if(HAL_I2C_Master_Transmit(_i2cHandler, DISPLAY_ADDRESS1<<1, TransmitData, sizeof(TransmitData), 100) != HAL_OK)		// transmit data
    	retval = LCD_ERROR;
    HAL_Delay(10);

	return retval;
}



/**
 * @brief  	Disable splash screen at power on
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayDisableSplash()
{
	uint8_t retval = LCD_OK;

	// create i2c data stream
    uint8_t TransmitData[2] = {SETTING_COMMAND,									// special command character
    						   DISABLE_SPLASH_DISPLAY, 							// set '.' character
							   };

    // transmission of data stream
    if(HAL_I2C_Master_Transmit(_i2cHandler, DISPLAY_ADDRESS1<<1, TransmitData, sizeof(TransmitData), 100) != HAL_OK)		// transmit data
    	retval = LCD_ERROR;
    HAL_Delay(10);

	return retval;
}



/**
 * @brief  	Save the current display as the splash
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displaySaveSplash()
{
	uint8_t retval = LCD_OK;

	// create i2c data stream
    uint8_t TransmitData[2] = {SETTING_COMMAND,									// special command character
    						   SAVE_CURRENT_DISPLAY_AS_SPLASH, 					// set Ctrl+j character
							   };

    // transmission of data stream
    if(HAL_I2C_Master_Transmit(_i2cHandler, DISPLAY_ADDRESS1<<1, TransmitData, sizeof(TransmitData), 100) != HAL_OK)		// transmit data
    	retval = LCD_ERROR;
    HAL_Delay(10);

	return retval;
}



/*
 *******************************************************************************
 * only for internal use - don't touch!
 *******************************************************************************
 */


/**
 * @brief	Send a command to the display.
 * 			Used by other functions.
 * @retval	status 	- 0...OK, other...error
 */
uint8_t displayCommand(uint8_t command)
{
	uint8_t retval = LCD_OK;
	uint8_t TransmitData[2] = {SETTING_COMMAND, command}; 												// create data stream
	if(HAL_I2C_Master_Transmit(_i2cHandler, DISPLAY_ADDRESS1<<1, TransmitData, sizeof(TransmitData), 100) != HAL_OK)		// transmit data
		retval = LCD_ERROR;
	HAL_Delay(10); //Wait a bit longer for special display commands
	return retval;
}



/**
 * @brief	Send a special command to the display.  Used by other functions.
 * @retval	status (0...OK, other...error)
 */
uint8_t displaySpecialCommand(uint8_t command)
{
	uint8_t retval = LCD_OK;
	uint8_t TransmitData[2] = {SPECIAL_COMMAND, command}; 												// create data stream
	if(HAL_I2C_Master_Transmit(_i2cHandler, DISPLAY_ADDRESS1<<1, TransmitData, sizeof(TransmitData), 100) != HAL_OK)		// transmit data
		retval = LCD_ERROR;
	HAL_Delay(50); //Wait a bit longer for special display commands
	return retval;
}













