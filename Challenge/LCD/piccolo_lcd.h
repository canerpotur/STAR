/*
 * Piccolo_LCD_Routines.h
 *
 *  Created on: Feb 21, 2015
 *      Author: Mani
 */

#ifndef PICCOLO_LCD_ROUTINES_H_
#define PICCOLO_LCD_ROUTINES_H_

void CursorON(void);							  /* Make Cursor visible */
void CursorOFF(void);							  /* Hide cursor */
void DisplayLCD(char LineNumber,char *Message);	  /* Display the given message (16 characters) at given location on LCD */
void WriteCommandLCD(unsigned char CommandByte);  /* Write the given command to LCD */
void WriteDataLCD(unsigned char DataByte);		  /* Write the given data to LCD */
void InitializeLCD(void);						  /* Initialize LCD */

#endif /* PICCOLO_LCD_ROUTINES_H_ */



