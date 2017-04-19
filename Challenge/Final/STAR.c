


#define RS PORTDbits.RD0
#define RW PORTDbits.RD1
#define EN PORTDbits.RD2
#define BF PORTBbits.RB7
#define DATAPORT LATB
/************** Line Addr Mapping ******************/
#define LCD_LINE1 0x80
#define LCD_LINE2 0xC0
#define LCD_LINE3 0x94
#define LCD_LINE4 0xD4
/************ LCD Command Mapping *****************/
#define CLRSCR 0x01
#define DISPLAY_ON 0x0C
#define DISPLAY_OFF 0x08
#define CURSOR_ON 0x0A
#define CURSOR_OFF 0x08
#define CURSOR_INC 0x06
#define MODE_8BIT 0x38
#define MODE_4BIT 0x28
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/wdog.h"
#include <stdio.h>
CLK_Handle myClk;
FLASH_Handle myFlash;
GPIO_Handle myGpio;
PIE_Handle myPie;

#include "f2802x_common/include/adc.h"
#define CONV_WAIT 1L //Micro-seconds to wait for ADC conversion. Longer than necessary.
int c = 1, d = 1;
int16_t temp; //raw temperature sensor reading
int16_t celc;
int16_t degC; //temperature in deg. C
int16_t degK; //temperature in deg. K
int16_t x;
void CursorON(void);                              /* Make Cursor visible */
void CursorOFF(void);                             /* Hide cursor */
void DisplayLCD(char LineNumber,char *Message);   /* Display the given message (16 characters) at given location on LCD */
void WriteCommandLCD(unsigned char CommandByte);  /* Write the given command to LCD */
void WriteDataLCD(unsigned char DataByte);        /* Write the given data to LCD */
void InitializeLCD(void);                         /* Initialize LCD */
void LCDDelay1600(void);
void LCDDelay(void);
void SendByte(unsigned char Value);
#define RS  GPIO_Number_12
#define E   GPIO_Number_19

#define D0  GPIO_Number_0
#define D1  GPIO_Number_1
#define D2  GPIO_Number_2
#define D3  GPIO_Number_3
#define D4  GPIO_Number_4
#define D5  GPIO_Number_5
#define D6  GPIO_Number_6
#define D7  GPIO_Number_7
GPIO_Handle myGpio;

void get_dec_str (uint8_t* str, size_t len, uint32_t val);
void InitializeGpio()
{
GPIO_setMode(myGpio, GPIO_Number_0, GPIO_0_Mode_GeneralPurpose);
GPIO_setDirection(myGpio, GPIO_Number_0, GPIO_Direction_Output);
GPIO_setMode(myGpio, GPIO_Number_1, GPIO_1_Mode_GeneralPurpose);
GPIO_setDirection(myGpio, GPIO_Number_1, GPIO_Direction_Output);
GPIO_setMode(myGpio, GPIO_Number_2, GPIO_2_Mode_GeneralPurpose);
GPIO_setDirection(myGpio, GPIO_Number_2, GPIO_Direction_Output);
GPIO_setMode(myGpio, GPIO_Number_3, GPIO_3_Mode_GeneralPurpose);
GPIO_setDirection(myGpio, GPIO_Number_3, GPIO_Direction_Output);
GPIO_setMode(myGpio, GPIO_Number_4, GPIO_4_Mode_GeneralPurpose);
GPIO_setDirection(myGpio, GPIO_Number_4, GPIO_Direction_Output);
GPIO_setMode(myGpio, GPIO_Number_5, GPIO_5_Mode_GeneralPurpose);
GPIO_setDirection(myGpio, GPIO_Number_5, GPIO_Direction_Output);
GPIO_setMode(myGpio, GPIO_Number_6, GPIO_6_Mode_GeneralPurpose);
GPIO_setDirection(myGpio, GPIO_Number_6, GPIO_Direction_Output);
GPIO_setMode(myGpio, GPIO_Number_7, GPIO_7_Mode_GeneralPurpose);
GPIO_setDirection(myGpio, GPIO_Number_7, GPIO_Direction_Output);
GPIO_setMode(myGpio, GPIO_Number_12, GPIO_12_Mode_GeneralPurpose);
GPIO_setDirection(myGpio, GPIO_Number_12, GPIO_Direction_Output);
GPIO_setMode(myGpio, GPIO_Number_18, GPIO_18_Mode_GeneralPurpose);
GPIO_setDirection(myGpio, GPIO_Number_18, GPIO_Direction_Output);
GPIO_setMode(myGpio, GPIO_Number_19, GPIO_19_Mode_GeneralPurpose);
GPIO_setDirection(myGpio, GPIO_Number_19, GPIO_Direction_Output);
}
/* Writes a command byte to LCD */
void WriteCommandLCD(unsigned char CommandByte)
{
    GPIO_setLow(myGpio, RS);        //Clear RS pin to write command
    SendByte(CommandByte);
    LCDDelay();                     //Small delay
}


/* Initializes LCD */
void InitializeLCD(void)
{
    GPIO_setHigh(myGpio, E);
    LCDDelay1600();
    LCDDelay1600();
    LCDDelay1600();
    LCDDelay1600();

    WriteCommandLCD(0x38);          //Command to select 8 bit interface
    LCDDelay1600();

    WriteCommandLCD(0x38);          //Command to select 8 bit interface
    LCDDelay();                     //Small delay

    WriteCommandLCD(0x38);          //Command to select 8 bit interface
    LCDDelay();


    WriteCommandLCD(0x08);          //Command to off cursor,display off
    WriteCommandLCD(0x01);          //Command to Clear LCD
    LCDDelay1600();
    WriteCommandLCD(0x06);          //Command for setting entry mode

    WriteCommandLCD(0x0f);          //Command to on cursor,blink cursor
    WriteCommandLCD(0x02);          //Command return the cursor to home
    LCDDelay1600();

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


    GPIO_setHigh(myGpio, E);            //Set E pin to select LCD
    for(temp=0;temp<5; temp++);
    GPIO_setLow(myGpio, E);             //Clear E pin to deselect LCD
    LCDDelay();                         //Small delay

}

/* Writes a Data byte to LCD */
void WriteDataLCD(unsigned char DataByte)
{
    GPIO_setHigh(myGpio, RS);           //Set RS pin to 1 to write Data
    SendByte(DataByte);
    LCDDelay();                         //Small delay
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
    WriteCommandLCD(0x0f);          //Command to switch on cursor
}



/* Makes cursor invisible */
void CursorOFF(void)
{
    WriteCommandLCD(0x0c);          //Command to switch off cursor
}


/* Displays a message on LCD */
void DisplayLCD(char LineNumber,char *Message)
{
    int a;
    if(LineNumber ==1)
    {   //First Line
        WriteCommandLCD(0x80);      //Select the first line
    }
    else
    {   //Second line
        WriteCommandLCD(0xc0);      //Select the second line
    }
    for(a=0;a<16;a++)
    {
        WriteDataLCD(*Message);     //Display a character
        Message++;                  //Increment pointer
    }
    return;
}




void main(void)
{

    ADC_Handle myAdc;
    CPU_Handle myCpu;
    PLL_Handle myPll;
    WDOG_Handle myWDog;

    // Initialize all the handles needed for this application
    myClk = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
    myCpu = CPU_init((void *)NULL, sizeof(CPU_Obj));
    myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
    myGpio = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    myPie = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
    myPll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
    myWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));

    // Perform basic system initialization
    WDOG_disable(myWDog);
    CLK_enableAdcClock(myClk);
    (*Device_cal)();
    CLK_disableAdcClock(myClk);

    //Select the internal oscillator 1 as the clock source
    CLK_setOscSrc(myClk, CLK_OscSrc_Internal);

    // Setup the PLL for x10 /2 which will yield 50Mhz = 10Mhz * 10 / 2
    PLL_setup(myPll, PLL_Multiplier_10, PLL_DivideSelect_ClkIn_by_2);

    // Disable the PIE and all interrupts
    PIE_disable(myPie);
    PIE_disableAllInts(myPie);
    CPU_disableGlobalInts(myCpu);
    CPU_clearIntFlags(myCpu);

    // If running from flash copy RAM only functions to RAM


    // Setup a debug vector table and enable the PIE

    PIE_setDebugIntVectorTable(myPie);
    PIE_enable(myPie);


       // Initialize all the handles needed for this application
       myAdc = ADC_init((void *)ADC_BASE_ADDR, sizeof(ADC_Obj));
       myClk = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
       myCpu = CPU_init((void *)NULL, sizeof(CPU_Obj));
       myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
       myGpio = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
       myPie = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
       myPll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
       myWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));

       // Perform basic system initialization
       WDOG_disable(myWDog);
       CLK_enableAdcClock(myClk);
       (*Device_cal)();

       //Select the internal oscillator 1 as the clock source
       CLK_setOscSrc(myClk, CLK_OscSrc_Internal);

       // Setup the PLL for x10 /2 which will yield 50Mhz = 10Mhz * 10 / 2
       PLL_setup(myPll, PLL_Multiplier_10, PLL_DivideSelect_ClkIn_by_2);

       // Disable the PIE and all interrupts
       PIE_disable(myPie);
       PIE_disableAllInts(myPie);
       CPU_disableGlobalInts(myCpu);
       CPU_clearIntFlags(myCpu);

       // If running from flash copy RAM only functions to RAM
   #ifdef _FLASH
       memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
   #endif

       // Initalize GPIO
       // Enable XCLOCKOUT to allow monitoring of oscillator 1
       GPIO_setMode(myGpio, GPIO_Number_18, GPIO_18_Mode_XCLKOUT);
       CLK_setClkOutPreScaler(myClk, CLK_ClkOutPreScaler_SysClkOut_by_1);


       // Setup a debug vector table and enable the PIE
       PIE_setDebugIntVectorTable(myPie);
       PIE_enable(myPie);

       // Initialize the ADC
       ADC_enableBandGap(myAdc);
       ADC_enableRefBuffers(myAdc);
       ADC_powerUp(myAdc);
       ADC_enable(myAdc);
       ADC_setVoltRefSrc(myAdc, ADC_VoltageRefSrc_Int);

       ADC_enableTempSensor(myAdc);                                            //Connect channel A5 internally to the temperature sensor
       ADC_setSocChanNumber (myAdc, ADC_SocNumber_0, ADC_SocChanNumber_A5);    //Set SOC0 channel select to ADCINA5
       ADC_setSocChanNumber (myAdc, ADC_SocNumber_1, ADC_SocChanNumber_A5);    //Set SOC1 channel select to ADCINA5
       ADC_setSocSampleWindow(myAdc, ADC_SocNumber_0, ADC_SocSampleWindow_7_cycles);   //Set SOC0 acquisition period to 7 ADCCLK
       ADC_setSocSampleWindow(myAdc, ADC_SocNumber_1, ADC_SocSampleWindow_7_cycles);   //Set SOC1 acquisition period to 7 ADCCLK
       ADC_setIntSrc(myAdc, ADC_IntNumber_1, ADC_IntSrc_EOC1);                 //Connect ADCINT1 to EOC1
       ADC_enableInt(myAdc, ADC_IntNumber_1);                                  //Enable ADCINT1

       // Note: two channels have been connected to the temp sensor
       // so that the first sample can be discarded to avoid the
       // ADC first sample issue.  See the device errata.

       // Set the flash OTP wait-states to minimum. This is important
       // for the performance of the temperature conversion function.
       FLASH_setup(myFlash);
celc=300;
       celc=degK-273;
       //Main program loop - continually sample temperature
       InitializeGpio();
       InitializeLCD();
       DELAY_US(500);
         char str[15] = "---------------";

         get_dec_str (&str[0], 5, celc);
        // sprintf(str, "Degree = %d", degK);
         //snprintf(str, 10, "%d", degK);
       for(;;)
       {

         WriteCommandLCD(0x01);   //clean lcd

         celc=degK-273;
           get_dec_str (&str[0], 5, celc);
           WriteCommandLCD(0x01);   //clean lcd


           DisplayLCD(1,str);





           //Force start of conversion on SOC0 and SOC1
           ADC_forceConversion(myAdc, ADC_SocNumber_0);
           ADC_forceConversion(myAdc, ADC_SocNumber_1);

           //Wait for end of conversion.
           while(ADC_getIntStatus(myAdc, ADC_IntNumber_1) == 0) {
           }

           // Clear ADCINT1
           ADC_clearIntFlag(myAdc, ADC_IntNumber_1);

           // Get temp sensor sample result from SOC1
           temp = ADC_readResult(myAdc, ADC_ResultNumber_1);

           // Convert the raw temperature sensor measurement into temperature
           degC = ADC_getTemperatureC(myAdc, temp);
           degK = ADC_getTemperatureK(myAdc, temp);

       }



}


void get_dec_str (uint8_t* str, size_t len, uint32_t val)
{
  uint8_t i;
  for(i=1; i<=len; i++)
  {
    str[len-i] = (uint8_t) ((val % 10UL) + '0');
    val/=10;
  }

  str[i-1] = '\0';
}



//===========================================================================
// No more.
//===========================================================================

