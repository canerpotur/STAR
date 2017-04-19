/************************************************************************/
/* LCD Interface - 8 bit mode											*/
/*                                                                      */
/* LCD Connection:-														*/
/* ----------------														*/
/* D0 -> GPIO0                                                     		*/
/* D1 -> GPIO1                                                     		*/
/* D2 -> GPIO2                                                     		*/
/* D3 -> GPIO3                                                     		*/
/* D4 -> GPIO4                                                     		*/
/* D5 -> GPIO5                                                     		*/
/* D6 -> GPIO6                                                     		*/
/* D7 -> GPIO7                                                     		*/
/*                                                                      */
/* RS -> GPIO12	                                                    	*/
/* RW -> GND                              	                    		*/
/* E  -> GPIO19                                                     	*/
/*                                                                      */
/*                                                                      */
/************************************************************************/

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pie.h"
#include "piccolo_lcd.h"

void WriteCommandLCD(unsigned char CommandByte);
void WriteDataLCD(unsigned char DataByte);
void LCDDelay(void);
void LCDDelay1600(void);
void SendByte(unsigned char Value);


#define	RS	GPIO_Number_12
#define	E	GPIO_Number_19

#define	D0	GPIO_Number_0
#define	D1	GPIO_Number_1
#define	D2	GPIO_Number_2
#define	D3	GPIO_Number_3
#define	D4	GPIO_Number_4
#define	D5	GPIO_Number_5
#define	D6	GPIO_Number_6
#define	D7	GPIO_Number_7
GPIO_Handle myGpio;


/* Initializes LCD */
void InitializeLCD(void)
{
	GPIO_setHigh(myGpio, E);
	LCDDelay1600();
	LCDDelay1600();
	LCDDelay1600();
	LCDDelay1600();

	WriteCommandLCD(0x38);			//Command to select 8 bit interface
	LCDDelay1600();

	WriteCommandLCD(0x38);			//Command to select 8 bit interface
	LCDDelay();				        //Small delay

	WriteCommandLCD(0x38);			//Command to select 8 bit interface
	LCDDelay();


	WriteCommandLCD(0x08);			//Command to off cursor,display off
	WriteCommandLCD(0x01);			//Command to Clear LCD
	LCDDelay1600();
	WriteCommandLCD(0x06);			//Command for setting entry mode

	WriteCommandLCD(0x0f);			//Command to on cursor,blink cursor
	WriteCommandLCD(0x02);			//Command return the cursor to home
	LCDDelay1600();

}


/* Writes a command byte to LCD */
void WriteCommandLCD(unsigned char CommandByte)
{
	GPIO_setLow(myGpio, RS);	    //Clear RS pin to write command
	SendByte(CommandByte);
	LCDDelay();				        //Small delay
}


/* Send a byte of data to LCD */
void SendByte(unsigned char Value)
{
	unsigned char temp;


	if((Value & 0x01) == 0x01)
		GPIO_setHigh(myGpio, D0);
	else
		GPIO_setLow(myGpio, D0);


	if((Value & 0x02) == 0x02)
		GPIO_setHigh(myGpio, D1);
	else
		GPIO_setLow(myGpio, D1);


	if((Value & 0x04) == 0x04)
		GPIO_setHigh(myGpio, D2);
	else
		GPIO_setLow(myGpio, D2);

	if((Value & 0x08) == 0x08)
		GPIO_setHigh(myGpio, D3);
	else
		GPIO_setLow(myGpio, D3);

	if((Value & 0x10) == 0x10)
		GPIO_setHigh(myGpio, D4);
	else
		GPIO_setLow(myGpio, D4);


	if((Value & 0x20) == 0x20)
		GPIO_setHigh(myGpio, D5);
	else
		GPIO_setLow(myGpio, D5);


	if((Value & 0x40) == 0x40)
		GPIO_setHigh(myGpio, D6);
	else
		GPIO_setLow(myGpio, D6);


	if((Value & 0x80) == 0x80)
		GPIO_setHigh(myGpio, D7);
	else
		GPIO_setLow(myGpio, D7);


	GPIO_setHigh(myGpio, E);			//Set E pin to select LCD
	for(temp=0;temp<5; temp++);
	GPIO_setLow(myGpio, E);				//Clear E pin to deselect LCD
	LCDDelay();				            //Small delay

}

/* Writes a Data byte to LCD */
void WriteDataLCD(unsigned char DataByte)
{
	GPIO_setHigh(myGpio, RS);			//Set RS pin to 1 to write Data
	SendByte(DataByte);
	LCDDelay();				            //Small delay
}


/* Small delay */
void LCDDelay(void)
{
	DELAY_US(50);
}

/* Big delay */
void LCDDelay1600(void)
{
	DELAY_US(1600);
}


/* Makes cursor visible */
void CursorON(void)
{
	WriteCommandLCD(0x0f);			//Command to switch on cursor
}



/* Makes cursor invisible */
void CursorOFF(void)
{
	WriteCommandLCD(0x0c);			//Command to switch off cursor
}


/* Displays a message on LCD */
void DisplayLCD(char LineNumber,char *Message)
{
	int	a;
	if(LineNumber ==1)
	{	//First Line
		WriteCommandLCD(0x80);		//Select the first line
	}
	else
	{	//Second line
		WriteCommandLCD(0xc0);		//Select the second line
	}
	for(a=0;a<16;a++)
	{
		WriteDataLCD(*Message);		//Display a character
		Message++;                  //Increment pointer
	}
	return;
}

