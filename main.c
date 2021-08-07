/********************************************************************
         FileName:     main.c
        Dependencies: See INCLUDES section
        Processor:   PIC18 or PIC24 USB Microcontrollers
        Hardware:    The code is natively intended to be used on the following
                hardware platforms: PICDEM™ FS USB Demo Board, 
                PIC18F87J50 FS USB Plug-In Module, or
                Explorer 16 + PIC24 USB PIM.  The firmware may be
                modified for use on other USB platforms by editing the
                HardwareProfile.h file.
        Complier:    Microchip C18 (for PIC18) or C30 (for PIC24)
        Company:   Microchip Technology, Inc.

        Software License Agreement:

        The software supplied herewith by Microchip Technology Incorporated
        (the “Company”) for its PIC® Microcontroller is intended and
        supplied to you, the Company’s customer, for use solely and
        exclusively on Microchip PIC Microcontroller products. The
        software is owned by the Company and/or its supplier, and is
        protected under applicable copyright laws. All rights are reserved.
        Any use in violation of the foregoing restrictions may subject the
        user to criminal sanctions under applicable laws, as well as to
        civil liability for the breach of the terms and conditions of this
        license.

        THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
        WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
        TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
        PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
        IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
        CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

        ********************************************************************
        File Description:

        Change History:
        Rev   Date         Description
        1.0   11/19/2004   Initial release
        2.1   02/26/2007   Updated for simplicity and to use common
                            coding style
        ********************************************************************/

//	========================	INCLUDES	========================
#ifdef _VISUAL
#include "VisualSpecials.h"
#endif // VISUAL

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"

#include "mtouch.h"

#include "BMA150.h"

#include "oled.h"

#include "OledGraphics.h"

#include "soft_start.h"

//	========================	CONFIGURATION	========================

#if defined(PIC18F46J50_PIM)
//Watchdog Timer Enable bit:
#pragma config WDTEN = OFF          //WDT disabled (control is placed on SWDTEN bit) \
                                            //PLL Prescaler Selection bits:
#pragma config PLLDIV = 3           //Divide by 3 (12 MHz oscillator input) \
                                            //Stack Overflow/Underflow Reset Enable bit:
#pragma config STVREN = ON          //Reset on stack overflow/underflow enabled \
                                            //Extended Instruction Set Enable bit:
#pragma config XINST = OFF          //Instruction set extension and Indexed Addressing mode disabled (Legacy mode) \
                                            //CPU System Clock Postscaler:
#pragma config CPUDIV = OSC1        //No CPU system clock divide \
                                            //Code Protection bit:
#pragma config CP0 = OFF            //Program memory is not code-protected \
                                            //Oscillator Selection bits:
#pragma config OSC = ECPLL          //HS oscillator, PLL enabled, HSPLL used by USB \
                                            //Secondary Clock Source T1OSCEN Enforcement:
#pragma config T1DIG = ON           //Secondary Oscillator clock source may be selected \
                                            //Low-Power Timer1 Oscillator Enable bit:
#pragma config LPT1OSC = OFF        //Timer1 oscillator configured for higher power operation \
                                            //Fail-Safe Clock Monitor Enable bit:
#pragma config FCMEN = OFF          //Fail-Safe Clock Monitor disabled \
                                            //Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit:
#pragma config IESO = OFF           //Two-Speed Start-up disabled \
                                            //Watchdog Timer Postscaler Select bits:
#pragma config WDTPS = 32768        //1:32768 \
                                            //DSWDT Reference Clock Select bit:
#pragma config DSWDTOSC = INTOSCREF //DSWDT uses INTOSC/INTRC as reference clock \
                                            //RTCC Reference Clock Select bit:
#pragma config RTCOSC = T1OSCREF    //RTCC uses T1OSC/T1CKI as reference clock \
                                            //Deep Sleep BOR Enable bit:
#pragma config DSBOREN = OFF        //Zero-Power BOR disabled in Deep Sleep (does not affect operation in non-Deep Sleep modes) \
                                            //Deep Sleep Watchdog Timer Enable bit:
#pragma config DSWDTEN = OFF        //Disabled \
                                            //Deep Sleep Watchdog Timer Postscale Select bits:
#pragma config DSWDTPS = 8192       //1:8,192 (8.5 seconds) \
                                            //IOLOCK One-Way Set Enable bit:
#pragma config IOL1WAY = OFF        //The IOLOCK bit (PPSCON<0>) can be set and cleared as needed \
                                            //MSSP address mask:
#pragma config MSSP7B_EN = MSK7     //7 Bit address masking \
                                            //Write Protect Program Flash Pages:
#pragma config WPFP = PAGE_1        //Write Protect Program Flash Page 0 \
                                            //Write Protection End Page (valid when WPDIS = 0):
#pragma config WPEND = PAGE_0       //Write/Erase protect Flash Memory pages starting at page 0 and ending with page WPFP[5:0] \
                                            //Write/Erase Protect Last Page In User Flash bit:
#pragma config WPCFG = OFF          //Write/Erase Protection of last page Disabled \
                                            //Write Protect Disable bit:
#pragma config WPDIS = OFF          //WPFP[5:0], WPEND, and WPCFG bits ignored

#else
#error No hardware board defined, see "HardwareProfile.h" and __FILE__
#endif

//	========================	Global VARIABLES	========================
#pragma udata
//You can define Global Data Elements here

unsigned char steps[120] = {0};
BOOL dh, dm;
ROM char coord[60][2] = {
    {67, 1}, {69, 1}, {71, 1}, {73, 2}, {76, 2}, {79, 3}, {82, 5}, {85, 7}, {88, 10}, {91, 13}, {93, 16}, {95, 19}, {96, 22}, {96, 25}, {97, 28}, {97, 31}, {97, 33}, {97, 35}, {96, 37}, {96, 40}, {95, 43}, {93, 46}, {91, 49}, {88, 52}, {85, 55}, {82, 57}, {79, 59}, {76, 60}, {73, 60}, {70, 61}, {67, 61}, {65, 61}, {63, 61}, {61, 60}, {58, 60}, {55, 59}, {52, 57}, {49, 55}, {46, 52}, {43, 49}, {41, 46}, {39, 43}, {38, 40}, {38, 37}, {37, 34}, {37, 31}, {37, 29}, {37, 27}, {38, 25}, {38, 22}, {39, 19}, {41, 16}, {43, 13}, {46, 10}, {49, 7}, {52, 5}, {55, 3}, {58, 2}, {61, 2}, {64, 1}};
ROM char analogStartLinesPos[12][2] = {
    {67, 0}, {80, 2}, {94, 15}, {98, 31}, {96, 44}, {83, 58}, {67, 62}, {54, 60}, {40, 47}, {36, 31}, {38, 18}, {51, 4}};
ROM char analogEndLinesPos[12][2] = {{67, 2}, {78, 4}, {92, 17}, {96, 31}, {94, 42}, {81, 56}, {67, 60}, {56, 58}, {42, 45}, {38, 31}, {40, 20}, {53, 6}};

ROM char hourPos[12][2] = {
    {67, 21}, {71, 22}, {75, 26}, {77, 31}, {76, 35}, {71, 40}, {67, 41}, {63, 40}, {59, 36}, {57, 31}, {58, 27}, {62, 22}};

BOOL anClockStart = 0;
BOOL sh = 1;
BOOL sm = 1;
BOOL ss = 1;
unsigned char stepI[3] = {0};
unsigned char stepIFlag[3] = {0};
unsigned char stepCount = 0;
BOOL f1 = 0;
unsigned int stepLength = 20;

typedef struct timeAndDate
{
    int day, mon;
    int hour, min, sec;
} timeAndDate;
timeAndDate TDdata;

BOOL moveSec, moveMin, moveHour;
BOOL Tflag = 0;
BOOL interval_24or12 = 0;
unsigned int dayStep = 0;
unsigned char clockType = 0; //need it 0-digital, 1- analog, 2-steps graph

//	========================	PRIVATE PROTOTYPES	========================
static void InitializeSystem(void);
static void ProcessIO(void);
static void UserInit(void);
static void YourHighPriorityISRCode();
static void YourLowPriorityISRCode();
void exeOP(int op);
void setMenu();
int GetAccVal(char);
BOOL stepCounter();
void printStepAnim();
void moveStepsArray();

BOOL pressRLUP(unsigned int n, int base)
{
    if (n > base)
    {
        return 0;
    }
    return 1;
}

//	========================	VECTOR REMAPPING	========================
#if defined(__18CXX)
//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
//the reset, high priority interrupt, and low priority interrupt
//vectors.  However, the current Microchip USB bootloader
//examples are intended to occupy addresses 0x00-0x7FF or
//0x00-0xFFF depending on which bootloader is used.  Therefore,
//the bootloader code remaps these vectors to new locations
//as indicated below.  This remapping is only necessary if you
//wish to program the hex file generated from this project with
//the USB bootloader.  If no bootloader is used, edit the
//usb_config.h file and comment out the following defines:
//#define PROGRAMMABLE_WITH_SD_BOOTLOADER

#if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
#define REMAPPED_RESET_VECTOR_ADDRESS 0xA000
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS 0xA008
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0xA018
#else
#define REMAPPED_RESET_VECTOR_ADDRESS 0x00
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS 0x08
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0x18
#endif

#if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
extern void _startup(void); // See c018i.c in your C18 compiler dir
#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
void _reset(void)
{
    _asm goto _startup _endasm
}
#endif
#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
void Remapped_High_ISR(void)
{
    _asm goto YourHighPriorityISRCode _endasm
}
#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
void Remapped_Low_ISR(void)
{
    _asm goto YourLowPriorityISRCode _endasm
}

#if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
//Note: If this project is built while one of the bootloaders has
//been defined, but then the output hex file is not programmed with
//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
//As a result, if an actual interrupt was enabled and occured, the PC would jump
//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
//would effective reset the application.

//To fix this situation, we should always deliberately place a
//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
//hex file of this project is programmed with the bootloader, these sections do not
//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
//programmed using the bootloader, then the below goto instructions do get programmed,
//and the hex file still works like normal.  The below section is only required to fix this
//scenario.
#pragma code HIGH_INTERRUPT_VECTOR = 0x08
void High_ISR(void)
{
    _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
}
#pragma code LOW_INTERRUPT_VECTOR = 0x18
void Low_ISR(void)
{
    _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
}
#endif //end of "#if defined(||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER))"

#pragma code

//	========================	Application Interrupt Service Routines	========================
//These are your actual interrupt handling routines.
#pragma interrupt YourHighPriorityISRCode
void YourHighPriorityISRCode()
{
    static char disp = 0;
    if (INTCONbits.T0IF)
    {
        f1 = 0;
        if (stepCount > steps[118])
        {
            steps[119] = stepCount - steps[118];
        }
        moveStepsArray();
        if (stepCounter() == 1)
        {
            f1 = 1;
        }

        TDdata.sec++;
        moveSec = 1;
        if (TDdata.sec == 60)
        {
            TDdata.sec = 0;
            TDdata.min++;
            moveMin = 1;
            stepCount = 0;
        }

        if (TDdata.min == 60)
        {
            TDdata.min = 0;
            TDdata.hour++;
            moveHour = 1;
        }

        if (TDdata.hour == 24)
        {
            TDdata.hour = 0;
            Tflag = 1;
            dayStep = 0;
        }

        if (Tflag)
        {
            if ((TDdata.mon == 1 || TDdata.mon == 3 || TDdata.mon == 5 || TDdata.mon == 7 || TDdata.mon == 8 || TDdata.mon == 10 || TDdata.mon == 12) && TDdata.day == 31)
            {
                ++TDdata.mon;
                TDdata.day = 1;
            }
            else if ((TDdata.mon == 4 || TDdata.mon == 6 || TDdata.mon == 9 || TDdata.mon == 11) && TDdata.day == 30)
            {
                ++TDdata.mon;
                TDdata.day = 1;
            }
            else if (TDdata.mon == 2 && TDdata.day == 28)
            {
                ++TDdata.mon;
                TDdata.day = 1;
            }
            else
                ++TDdata.day;

            Tflag = 0;
        }
        TMR0H = 0x48;
        TMR0L = 0xe5;
        INTCONbits.TMR0IF = 0b0;
    }
} //This return will be a "retfie fast", since this is in a #pragma interrupt section
#pragma interruptlow YourLowPriorityISRCode
void YourLowPriorityISRCode()
{
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.

} //This return will be a "retfie", since this is in a #pragma interruptlow section
#endif

//	========================	Board Initialization Code	========================
#pragma code
#define ROM_STRING rom unsigned char *

/******************************************************************************
         * Function:        void UserInit(void)
         *
         * PreCondition:    None
         *
         * Input:           None
         *
         * Output:          None
         *
         * Side Effects:    None
         *
         * Overview:        This routine should take care of all of the application code
         *                  initialization that is required.
         *
         * Note:            
         *
         *****************************************************************************/
void UserInit(void)
{

    /* Initialize the mTouch library */
    mTouchInit();

    /* Call the mTouch callibration function */
    mTouchCalibrate();

    /* Initialize the accelerometer */
    InitBma150();

    /* Initialize the oLED Display */
    ResetDevice();
    FillDisplay(0x00);
    // oledPutROMString((ROM_STRING) " PIC18F Starter Kit  ", 0, 0);
} //end UserInit

/********************************************************************
         * Function:        static void InitializeSystem(void)
         *
         * PreCondition:    None
         *
         * Input:           None
         *
         * Output:          None
         *
         * Side Effects:    None
         *
         * Overview:        InitializeSystem is a centralize initialization
         *                  routine. All required USB initialization routines
         *                  are called from here.
         *
         *                  User application initialization routine should
         *                  also be called from here.                  
         *
         * Note:            None
         *******************************************************************/
static void InitializeSystem(void)
{
    // Soft Start the APP_VDD
    while (!AppPowerReady())
        ;

#if defined(PIC18F87J50_PIM) || defined(PIC18F46J50_PIM)
    //On the PIC18F87J50 Family of USB microcontrollers, the PLL will not power up and be enabled
    //by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
    //This allows the device to power up at a lower initial operating frequency, which can be
    //advantageous when powered from a source which is not gauranteed to be adequate for 48MHz
    //operation.  On these devices, user firmware needs to manually set the OSCTUNE<PLLEN> bit to
    //power up the PLL.
    {
        unsigned int pll_startup_counter = 600;
        OSCTUNEbits.PLLEN = 1; //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
        while (pll_startup_counter--)
            ;
    }
    //Device switches over automatically to PLL output after PLL is locked and ready.
#endif

#if defined(PIC18F46J50_PIM)
    //Configure all I/O pins to use digital input buffers
    ANCON0 = 0xFF; // Default all pins to digital
    ANCON1 = 0xFF; // Default all pins to digital
#endif

    UserInit();

} //end InitializeSystem

//	========================	Application Code	========================
void moveStepsArray()
{
    int i = 0;
    for (i = 0; i < 119; i++)
    {
        steps[i] = steps[i + 1];
    }
    steps[119] = 0;
}
void oledprintBigNUmber(unsigned char numToPrint, unsigned char page, unsigned char col, unsigned char flag, BOOL flag2)
{
    char num[4];
    if (numToPrint > 9)
    {
        numToPrint = numToPrint % 10;
    }
    switch (numToPrint)
    {
    case 0:
    {

        sprintf(num, "U!B");
        oledPutString(num, page, col, flag2);
        sprintf(num, ") G");
        oledPutString(num, page + 1, col, flag2);
        sprintf(num, ", L");
        oledPutString(num, page + 2, col, flag2);
        sprintf(num, "<$O");
        oledPutString(num, page + 3, col, flag2);

        break;
    }
    case 1:
    {

        sprintf(num, "  E");
        oledPutString(num, page, col, flag2);
        sprintf(num, "  G");
        oledPutString(num, page + 1, col, flag2);
        sprintf(num, "  L");
        oledPutString(num, page + 2, col, flag2);
        sprintf(num, "  Q");
        oledPutString(num, page + 3, col, flag2);

        break;
    }
    case 2:
    {

        sprintf(num, "&!B");
        oledPutString(num, page, col, flag2);
        sprintf(num, "*TF");
        oledPutString(num, page + 1, col, flag2);
        sprintf(num, "+#N");
        oledPutString(num, page + 2, col, flag2);
        sprintf(num, "<$R");
        oledPutString(num, page + 3, col, flag2);

        break;
    }
    case 3:
    {

        sprintf(num, "&!B");
        oledPutString(num, page, col, flag2);
        sprintf(num, "*TF");
        oledPutString(num, page + 1, col, flag2);
        sprintf(num, "A#K");
        oledPutString(num, page + 2, col, flag2);
        sprintf(num, ";$O");
        oledPutString(num, page + 3, col, flag2);

        break;
    }
    case 4:
    {

        sprintf(num, "' E");
        oledPutString(num, page, col, flag2);
        sprintf(num, "(TF");
        oledPutString(num, page + 1, col, flag2);
        sprintf(num, "A#K");
        oledPutString(num, page + 2, col, flag2);
        sprintf(num, "  Q");
        oledPutString(num, page + 3, col, flag2);

        break;
    }
    case 5:
    {

        sprintf(num, "U!C");
        oledPutString(num, page, col, flag2);
        sprintf(num, "(TJ");
        oledPutString(num, page + 1, col, flag2);
        sprintf(num, "A#K");
        oledPutString(num, page + 2, col, flag2);
        sprintf(num, "<$O");
        oledPutString(num, page + 3, col, flag2);

        break;
    }
    case 6:
    {

        sprintf(num, "U!C");
        oledPutString(num, page, col, flag2);
        sprintf(num, "(TJ");
        oledPutString(num, page + 1, col, flag2);
        sprintf(num, "+#K");
        oledPutString(num, page + 2, col, flag2);
        sprintf(num, "<$O");
        oledPutString(num, page + 3, col, flag2);

        break;
    }
    case 7:
    {

        sprintf(num, "&!B");
        oledPutString(num, page, col, flag2);
        sprintf(num, "  G");
        oledPutString(num, page + 1, col, flag2);
        sprintf(num, "  L");
        oledPutString(num, page + 2, col, flag2);
        sprintf(num, "  Q");
        oledPutString(num, page + 3, col, flag2);

        break;
    }
    case 8:
    {

        sprintf(num, "U!B");
        oledPutString(num, page, col, flag2);
        sprintf(num, "(TF");
        oledPutString(num, page + 1, col, flag2);
        sprintf(num, "+#K");
        oledPutString(num, page + 2, col, flag2);
        sprintf(num, "<$O");
        oledPutString(num, page + 3, col, flag2);

        break;
    }
    case 9:
    {

        sprintf(num, "U!B");
        oledPutString(num, page, col, flag2);
        sprintf(num, "(TF");
        oledPutString(num, page + 1, col, flag2);
        sprintf(num, "A#K");
        oledPutString(num, page + 2, col, flag2);
        sprintf(num, "  Q");
        oledPutString(num, page + 3, col, flag2);

        break;
    }
    }
    if (flag == 1)
    {
        oledPutROMString(" ", page, col + 18, 1);
        oledPutROMString("{", page + 1, col + 18, 1);
        oledPutROMString("{", page + 2, col + 18, 1);
        oledPutROMString(" ", page + 3, col + 18, 1);
    }
    if (flag == 2)
    {
        oledPutROMString("{", page + 3, col + 18, 1);
    }
}

void printArrows()
{
    oledPutROMString("}", 2, 0);
    oledPutROMString("~", 4, 0);
}

void printStepAnim()
{
    char num[4];
    if (f1)
    {
        sprintf(num, "%d", stepCount);
        oledPutString(num, 0, 3 * 6, 1);

        oledPutROMString("=?", 0, 0);
        oledPutROMString(">@", 1, 0);

        DelayMs(40);
        oledPutROMString("VX", 0, 0);
        oledPutROMString("WY", 1, 0);
        DelayMs(40);
        oledPutROMString("=?", 0, 0);
        oledPutROMString(">@", 1, 0);
        DelayMs(40);
        oledPutROMString("VX", 0, 0);
        oledPutROMString("WY", 1, 0);
        DelayMs(40);
    }
}

void clearScreen()
{
    unsigned char i;
    for (i = 0; i < 8; i++)
    {
        oledPutROMString("                                 ", i, 0, 0);
    }
}

void datePrint()
{
    char dateprint[4];
    sprintf(dateprint, "%02d.", TDdata.day);
    oledPutString(dateprint, 7, 2 * 49, 1);
    sprintf(dateprint, "%02d", TDdata.mon);
    oledPutString(dateprint, 7, 2 * 58, 1);
}

void AMPM()
{
    if (TDdata.hour >= 0 && TDdata.hour <= 11)
    {
        oledPutROMString("am", 7, 0, 1);
    }
    else
    {
        oledPutROMString("pm", 7, 0, 1);
    }
}

void digitalClock(BOOL onClock)
{
    char timeprint[5];
    unsigned char x1;
    unsigned char x2;
    clearScreen();
    x1 = 0;
    x2 = 0;

    if (onClock == 1)
    {
        if (TDdata.sec > 9)
        {
            x1 = TDdata.sec / 10;
            x2 = TDdata.sec % 10;
        }
        else
        {
            x1 = 0;
            x2 = TDdata.sec;
        }
        oledprintBigNUmber(x1, 2, 15 * 6, 0, 1);
        oledprintBigNUmber(x2, 2, 18 * 6, 0, 1);
    }
    else
    {
        if (interval_24or12 == 1)
        {
            sprintf(timeprint, "%02d", TDdata.sec);
            oledPutString(timeprint, 7, 6 * 6, 1);
        }
        else
        {
            sprintf(timeprint, "%02d", TDdata.sec);
            oledPutString(timeprint, 7, 9 * 6, 1);
        }
    }

    if (onClock == 1)
    {
        if (TDdata.min > 9)
        {
            x1 = TDdata.min / 10;
            x2 = TDdata.min % 10;
        }
        else
        {
            x1 = 0;
            x2 = TDdata.min;
        }
        oledprintBigNUmber(x1, 2, 8 * 6, 0, 1);
        oledprintBigNUmber(x2, 2, 11 * 6, 1, 1);
    }
    else
    {
        if (interval_24or12 == 1)
        {
            sprintf(timeprint, "%02d:", TDdata.min);
            oledPutString(timeprint, 7, 3 * 6, 1);
        }
        else
        {
            sprintf(timeprint, "%02d:", TDdata.min);
            oledPutString(timeprint, 7, 6 * 6, 1);
        }
    }

    if (interval_24or12 == 1) //24H
    {
        if (onClock == 1)
        {
            if (TDdata.hour > 9)
            {
                x1 = TDdata.hour / 10;
                x2 = TDdata.hour % 10;
            }
            else
            {
                x1 = 0;
                x2 = TDdata.hour;
            }
            oledprintBigNUmber(x1, 2, 1 * 6, 0, 1);
            oledprintBigNUmber(x2, 2, 4 * 6, 1, 1);
        }
        else
        {
            sprintf(timeprint, "%02d:", TDdata.hour);
            oledPutString(timeprint, 7, 0, 1);
        }
    }
    if (interval_24or12 == 0) //12H am/pm
    {
        int hour_ = TDdata.hour;
        hour_ = hour_ % 12;
        if (hour_ > 9)
        {
            x1 = hour_ / 10;
            x2 = hour_ % 10;
        }
        else
        {
            x1 = 0;
            x2 = hour_;
        }
        if (hour_ == 0)
        {
            x1 = 1;
            x2 = 2;
        }
        if (onClock == 1)
        {
            oledprintBigNUmber(x1, 2, 1 * 6, 0, 1);
            oledprintBigNUmber(x2, 2, 4 * 6, 1, 1);
        }
        else
        {
            sprintf(timeprint, "%d%d:", x1, x2);
            oledPutString(timeprint, 7, 3 * 6, 1);
        }
        AMPM();
    }
    datePrint();
    if (onClock)
    {
        printStepAnim();
    }
}

void analogClock()
{
    unsigned char i;
    datePrint();
    if (anClockStart == 0) //check if clock template already printed;
    {
        dm = 0;
        dh = 0;
        anClockStart = 1;
        if (TDdata.hour == TDdata.min)
            sh = 0;
        if (TDdata.sec == TDdata.min)
            sm = 0;
        //print clock hours icons in position
        for (i = 0; i < 12; ++i)
        {
            drawLine(analogStartLinesPos[i][0], analogStartLinesPos[i][1], analogEndLinesPos[i][0], analogEndLinesPos[i][1], thin);
        }
        drawLine(67, 31, coord[TDdata.sec][0], coord[TDdata.sec][1], thin);
        moveSec = 0;
    }
    else
    {
        //check seconds
        if (moveSec == 1)
        {
            moveSec = 0;
            if (TDdata.sec == 0)
            {
                drawLine(67, 31, coord[59][0], coord[59][1], thin);
            }
            else
            {
                drawLine(67, 31, coord[TDdata.sec - 1][0], coord[TDdata.sec - 1][1], thin);
            }
            drawLine(67, 31, coord[TDdata.sec][0], coord[TDdata.sec][1], thin);
        }
        /*****/

        //check minuts
        if (sm == 0)
        {
            sm = 1;
            dm = 1;
            drawLine(67, 31, coord[TDdata.min][0], coord[TDdata.min][1], thick);
        }
        else
        {
            if (dm == 0)
            {
                dm = 1;
                drawLine(67, 31, coord[TDdata.min][0], coord[TDdata.min][1], thick);
            }
            else
            {
                if (moveMin == 1)
                {
                    moveMin = 0;
                    if (TDdata.min == 0)
                    {
                        drawLine(67, 31, coord[59][0], coord[59][1], thick);
                    }
                    else
                    {
                        drawLine(67, 31, coord[TDdata.min - 1][0], coord[TDdata.min - 1][1], thick);
                    }
                    drawLine(67, 31, coord[TDdata.min][0], coord[TDdata.min][1], thick);
                }
            }
        }

        /**************/
        //check hours
        if (sh == 0)
        {
            sh = 1;
            dh = 1;
            if (TDdata.hour < 12)
            {
                drawLine(67, 31, hourPos[TDdata.hour][0], hourPos[TDdata.hour][1], fat);
            }
            else
            {
                drawLine(67, 31, hourPos[TDdata.hour - 12][0], hourPos[TDdata.hour - 12][1], fat);
            }
        }
        else
        {
            if (dh == 0)
            {
                dh = 1;
                if (TDdata.hour < 12)
                {
                    drawLine(67, 31, hourPos[TDdata.hour][0], hourPos[TDdata.hour][1], fat);
                }
                else
                {
                    drawLine(67, 31, hourPos[TDdata.hour - 12][0], hourPos[TDdata.hour - 12][1], fat);
                }
            }
            else
            {
                if (moveHour == 1)
                {
                    moveHour = 0;
                    if (TDdata.hour + 1 >= 24)
                        TDdata.hour = 0;
                    else
                        ++TDdata.hour;
                    if (TDdata.hour == 0 || TDdata.hour == 12)
                    {
                        drawLine(67, 31, hourPos[11][0], hourPos[11][1], fat);
                        drawLine(67, 31, hourPos[0][0], hourPos[0][1], fat);
                    }
                    else
                    {
                        if (TDdata.hour < 12)
                        {
                            drawLine(67, 31, hourPos[TDdata.hour - 1][0], hourPos[TDdata.hour - 1][1], fat);
                            drawLine(67, 31, hourPos[TDdata.hour][0], hourPos[TDdata.hour][1], fat);
                        }
                        else
                        {
                            drawLine(67, 31, hourPos[TDdata.hour - 12 - 1][0], hourPos[TDdata.hour - 12 - 1][1], fat);
                            drawLine(67, 31, hourPos[TDdata.hour - 12][0], hourPos[TDdata.hour - 12][1], fat);
                        }
                    }
                    // }
                }
            }
        }
    }
    AMPM();
    if (f1 == 1)
    {
        printStepAnim();
        DelayMs(40);
        oledPutROMString("      ", 0, 0);
        oledPutROMString("      ", 1, 0);
    }
}

BOOL press_b()
{
    if (PORTBbits.RB0 == 0)
        return 1;
    else
        return 0;
}

void setTimexyz()
{
    unsigned char h, m, s;
    unsigned char curr = 0;
    h = TDdata.hour;
    m = TDdata.min;
    s = TDdata.sec;
    while (1)
    {
        //print
        clearScreen();
        digitalClock(0);

        if (0 == curr)
        {
            oledprintBigNUmber(h / 10, 2, 6, 0, 0);
            oledprintBigNUmber(h % 10, 2, 6 * 4, 1, 0);
        }
        else
        {
            oledprintBigNUmber(h / 10, 2, 6, 0, 1);
            oledprintBigNUmber(h % 10, 2, 6 * 4, 1, 1);
        }

        if (1 == curr)
        {
            oledprintBigNUmber(m / 10, 2, 6 * 8, 0, 0);
            oledprintBigNUmber(m % 10, 2, 6 * 11, 1, 0);
        }
        else
        {
            oledprintBigNUmber(m / 10, 2, 6 * 8, 0, 1);
            oledprintBigNUmber(m % 10, 2, 6 * 11, 1, 1);
        }
        if (2 == curr)
        {
            oledprintBigNUmber(s / 10, 2, 6 * 15, 0, 0);
            oledprintBigNUmber(s % 10, 2, 6 * 18, 0, 0);
        }
        else
        {
            oledprintBigNUmber(s / 10, 2, 6 * 15, 0, 1);
            oledprintBigNUmber(s % 10, 2, 6 * 18, 0, 1);
        }

        if (pressRLUP(mTouchReadButton(2), 650))
        { //press down

            if (curr == 0)
            {
                if (h == 0)
                {
                    h = 24;
                }
                if (h > 0)
                {
                    h = h - 1;
                }
                if (h == 24)
                {
                    h = 0;
                }
            }
            if (curr == 1)
            {
                if (m == 0)
                {
                    m = 60;
                }
                if (m > 0)
                {
                    m = m - 1;
                }
                if (m == 60)
                {
                    m = 0;
                }
            }
            if (curr == 2)
            {
                if (s == 0)
                {
                    s = 60;
                }
                if (s > 0)
                {
                    s = s - 1;
                }
                if (s == 60)
                {
                    s = 0;
                }
            }
        }
        if (pressRLUP(mTouchReadButton(1), 650))
        { //press up

            if (curr == 0)
            {
                if (h < 23)
                {
                    h = h + 1;
                }
                else
                {
                    h = 0;
                }
            }
            if (curr == 1)
            {
                if (m < 59)
                {
                    m = m + 1;
                }
                else
                {
                    m = 0;
                }
            }
            if (curr == 2)
            {
                if (s < 59)
                {
                    s = s + 1;
                }
                else
                {
                    s = 0;
                }
            }
        }
        //choose:
        if (press_b())
        { //press button
            TDdata.hour = h;
            TDdata.min = m;
            TDdata.sec = s;
            return;
        }
        //return:
        if (pressRLUP(mTouchReadButton(3), 600))
        { //press L

            if (curr == 0)
            {
                return;
            }
            if (curr == 1)
            {
                curr = 0;
            }
            if (curr == 2)
            {
                curr = 1;
            }
        }
        if (pressRLUP(mTouchReadButton(0), 800))
        { //press R
            if (curr == 1)
            {

                curr = 2;
            }
            if (curr == 0)
            {
                curr = 1;
            }
        }
        DelayMs(60);
    }
}
void setMenu()
{
    unsigned char curr;
    char op[20];
    curr = 0;
    while (1)
    {
        clearScreen();
        digitalClock(0);
        //print:

        sprintf(op, "Dispaly mode");
        if (0 == curr)
        {

            oledPutString(op, 1, 2 * 6, 0);
        }
        else
        {
            oledPutString(op, 1, 2 * 6, 1);
        }
        sprintf(op, "12H/24H interval");
        if (1 == curr)
        {

            oledPutString(op, 2, 2 * 6, 0);
        }
        else
        {
            oledPutString(op, 2, 2 * 6, 1);
        }
        sprintf(op, "Set time");
        if (2 == curr)
        {

            oledPutString(op, 3, 2 * 6, 0);
        }
        else
        {
            oledPutString(op, 3, 2 * 6, 1);
        }

        sprintf(op, "Set date");
        if (3 == curr)
        {

            oledPutString(op, 4, 2 * 6, 0);
        }
        else
        {
            oledPutString(op, 4, 2 * 6, 1);
        }
        sprintf(op, "Pedometer");
        if (4 == curr)
        {

            oledPutString(op, 5, 2 * 6, 0);
        }
        else
        {
            oledPutString(op, 5, 2 * 6, 1);
        }

        //move:
        if (pressRLUP(mTouchReadButton(1), 650))
        { //press down
            if (curr > 0)
                curr--;
        }
        if (pressRLUP(mTouchReadButton(2), 650))
        { //press up
            if (curr < 4)
                curr++;
        }
        //choose:
        if (pressRLUP(mTouchReadButton(0), 800))
        { //press R
            exeOP(curr);
        }
        //return:
        if (pressRLUP(mTouchReadButton(3), 600))
        { //press L
            clearScreen();
            return;
        }
        DelayMs(60);
    }
}

void setMode()
{
    char op[30];
    unsigned char curr = clockType;
    while (1)
    {
        //print
        clearScreen();
        digitalClock(0);
        sprintf(op, "Digital Mode");
        if (0 == curr)
        {

            oledPutString(op, 2, 2 * 6, 0);
        }
        else
        {
            oledPutString(op, 2, 2 * 6, 1);
        }

        sprintf(op, "Znalog Mode");
        if (1 == curr)
        {

            oledPutString(op, 3, 2 * 6, 0);
        }
        else
        {
            oledPutString(op, 3, 2 * 6, 1);
        }
        sprintf(op, "Step graph");
        if (2 == curr)
        {

            oledPutString(op, 4, 2 * 6, 0);
        }
        else
        {
            oledPutString(op, 4, 2 * 6, 1);
        }
        if (pressRLUP(mTouchReadButton(1), 650))
        { //press up
            if (curr == 0)
            {
                curr = 2;
            }
            else
            {
                if (curr == 1)
                {
                    curr = 0;
                }
                else
                {
                    if (curr == 2)
                    {
                        curr = 1;
                    }
                }
            }
        }
        if (pressRLUP(mTouchReadButton(2), 650))
        {
            //press down
            if (curr == 0)
            {
                curr = 1;
            }
            else
            {
                if (curr == 1)
                {
                    curr = 2;
                }
                else
                {
                    if (curr == 2)
                    {
                        curr = 0;
                    }
                }
            }
        }
        //choose:
        if (press_b())
        { //press button
            clockType = curr;
            return;
        }
        //return:
        if (pressRLUP(mTouchReadButton(3), 600))
        { //press L
            clearScreen();
            return;
        }
        DelayMs(60);
    }
}

void setInterval24Or12()
{
    char op[30];
    BOOL curr = interval_24or12;
    while (1)
    {
        //print
        clearScreen();
        digitalClock(0);
        sprintf(op, "Interval 12H");
        if (0 == curr)
        {

            oledPutString(op, 2, 2 * 6, 0);
        }
        else
        {
            oledPutString(op, 2, 2 * 6, 1);
        }

        sprintf(op, "Interval 24H");
        if (1 == curr)
        {

            oledPutString(op, 3, 2 * 6, 0);
        }
        else
        {
            oledPutString(op, 3, 2 * 6, 1);
        }

        if (pressRLUP(mTouchReadButton(1), 650) || pressRLUP(mTouchReadButton(2), 650))
        { //press down || up
            if (curr == 0)
            {
                curr = 1;
            }
            else
            {
                curr = 0;
            }
        }
        //choose:
        if (press_b())
        { //press button
            interval_24or12 = curr;
            return;
        }
        //return:
        if (pressRLUP(mTouchReadButton(3), 600))
        { //press L
            clearScreen();
            return;
        }
        DelayMs(60);
    }
}

BOOL datecheck(timeAndDate t)
{
    if (t->mon == 1 || t->mon == 3 || t->mon == 5 || t->mon == 7 || t->mon == 8 || t->mon == 10 || t->mon == 12)
    {
        if (t->day < 31)
            return 1;
        else
            return 0;
    }
    else if (t->mon == 4 || t->mon == 6 || t->mon == 9 || t->mon == 11)
    {
        if (t->day < 30)
            return 1;
        else
            return 0;
    }
    else if (t->mon == 2)
    {
        if (t->day < 28)
            return 1;
        else
            return 0;
    }
    else
        return 0;
}

void setDate()
{
    timeAndDate temp;
    BOOL curr = 0; //0= set day, 1= set month
    temp = TDdata;
    while (1)
    {
        clearScreen();
        digitalClock(0);
        printArrows();

        if (curr == 0)
        { //on day
            oledprintBigNUmber(temp.day / 10, 2, 3 * 6, 0, 0);
            oledprintBigNUmber(temp.day % 10, 2, 6 * 6, 2, 0);
            oledprintBigNUmber(temp.mon / 10, 2, 10 * 6, 0, 2);
            oledprintBigNUmber(temp.mon % 10, 2, 13 * 6, 0, 2);
        }
        else
        { // on month
            oledprintBigNUmber(temp.day / 10, 2, 3 * 6, 0, 2);
            oledprintBigNUmber(temp.day % 10, 2, 6 * 6, 2, 2);
            oledprintBigNUmber(temp.mon / 10, 2, 10 * 6, 0, 0);
            oledprintBigNUmber(temp.mon % 10, 2, 13 * 6, 0, 0);
        }

        if (pressRLUP(mTouchReadButton(1), 650))
        { //up pressed
            if (curr == 0)
            { //if on days
                if (temp.day + 1 > 31)
                    temp.day = 1;
                else
                    ++temp.day;
            }
            else
            { //if on months
                if (temp.mon + 1 > 12)
                    temp.mon = 1;
                else
                    ++temp.mon;
            }
        }
        if (pressRLUP(mTouchReadButton(2), 650))
        { // down pressed
            if (curr == 0)
            { //if on days
                if (temp.day - 1 < 1)
                    temp.day = 31;
                else
                    --temp.day;
            }
            else
            { //if on months
                if (temp.mon - 1 < 1)
                    temp.mon = 12;
                else
                    --temp.mon;
            }
        }
        if (pressRLUP(mTouchReadButton(3), 600))
        { // L pressed
            --curr;
            if (curr < 0)
            {
                clearScreen();
                return;
            }
        }
        if (pressRLUP(mTouchReadButton(0), 800))
        { // R pressed
            curr = 1;
        }
        //retrurn after select new date:
        if (press_b())
        { //button pressed
            if (datecheck(temp) == 1)
            {
                TDdata = temp;
            }
            clearScreen();
            return;
        }
        DelayMs(60);
    }
}

void stepGraph()
{

    unsigned char i, j, x = 3 * 6 + 1, y = 8 * 8 - 1, x1, y1;

    clearScreen();
    oledPutROMString("100", 1, 0, 1);
    oledPutROMString("90", 2, 0, 1);
    oledPutROMString("60", 4, 0, 1);
    oledPutROMString("30", 6, 0, 1);

    //draw graph lines
    for (i = 0; i < 18; ++i)
    {
        oledPutROMString("-", 1, (i + 3) * 6, 1);
        if (i % 2 == 0)
        {
            oledPutROMString("_", 7, (i + 3) * 6, 1);
        }
        else
        {
            oledPutROMString(".", 7, (i + 3) * 6, 1);
        }
    }
    for (j = 2; j <= 6; j = j + 2)
    {
        for (i = 0; i < 18; ++i)
        {
            oledPutROMString("-", j, (i + 3) * 6, 1);
        }
    }
    //draw praph acordding to data

    for (i = 0; i < 100; i = i + 2)
    {
        x1 = x + 2;
        y1 = 64 - (((int)(steps[i]) * 56) / 100) % 101 - 1;
        drawLine(x, y, x1, y1, thin);
        drawLine(x, y, x, y, thin);
        x = x1;
        y = y1;
    }
    DelayMs(60);
}

void Pedometer()
{
    unsigned int curr = stepLength;
    unsigned int oldLenghth = stepLength;
    unsigned long km;
    char num[6];
    while (1)
    {
        //print:
        clearScreen();
        digitalClock(0);
        printArrows();
        //datePrint();

        oledPutROMString("Set", 0, 3 * 6, 0);
        oledPutROMString(" length", 1, 1 * 6, 0);
        oledprintBigNUmber(curr / 10, 2, 6 * 2, 0, 0); //1
        oledprintBigNUmber(curr % 10, 2, 6 * 5, 0, 0); //1
        oledPutROMString("cm", 6, 4 * 6, 0);

        drawLine(8 * 6 + 1, 0, 8 * 6 + 1, 7 * 8 - 1, thin);

        oledPutROMString("=?", 2, 9 * 6 + 3, 0);
        oledPutROMString(">@", 3, 9 * 6 + 3, 0);
        sprintf(num, "%d", dayStep);
        oledPutString(num, 5, 9 * 6 + 3, 1);

        drawLine(12 * 6 + 4, 0, 12 * 6 + 4, 7 * 8 - 1, thin);
        oledPutROMString("Distance", 0, 14 * 6 - 4, 0);
        oledPutROMString("walked", 1, 14 * 6, 0);
        km = ((dayStep) * (oldLenghth)) / 100; ///////////////////////////////////////////////check value
        sprintf(num, "%lu", km);
        oledPutString(num, 3, 14 * 6 + 4, 1);

        oledPutROMString("meters", 5, 14 * 6, 0);

        if (pressRLUP(mTouchReadButton(1), 650))
        { //up pressed
            if (curr < 99)
            {
                curr = curr + 1;
            }
            else
            {
                if (curr == 99)
                {
                    curr = 1;
                }
            }
        }
        if (pressRLUP(mTouchReadButton(2), 650))
        { // down pressed
            if (curr == 1)
            {
                curr = 99;
            }
            else
            {
                if (curr > 1)
                {
                    curr = curr - 1;
                }
            }
        }
        if (pressRLUP(mTouchReadButton(3), 600))
        { // L pressed
            return;
        }

        //retrurn after select new date:
        if (press_b())
        { //button pressed
            stepLength = curr;
            clearScreen();
            return;
        }
        DelayMs(60);
    }
}
void exeOP(int op)
{
    switch (op)
    {
    case 0:
    {
        setMode();
        break;
    }
    case 1:
    {
        setInterval24Or12();
        break;
    }
    case 2:
    {
        setTimexyz();

        break;
    }
    case 3:
    {
        setDate();
        break;
    }
    case 4:
    {
        Pedometer();
        break;
    }
    default:
        break;
    }
}

int GetAccVal(char c)
{
    BYTE msb, lsb;
    BYTE mask = 0b10000000;
    int signextend = 0xFC00;
    int val = 0, n1, n2;
    if (c == 'x')
    {
        n1 = 3;
        n2 = 2;
    }
    if (c == 'y')
    {
        n1 = 5;
        n2 = 4;
    }
    if (c == 'z')
    {
        n1 = 7;
        n2 = 6;
    }
    msb = BMA150_ReadByte(n1);
    lsb = BMA150_ReadByte(n2);
    lsb = lsb >> 6;
    val += (int)msb;
    val = val << 2;
    val += (int)lsb;
    mask = mask & msb;
    if (mask == 0b10000000)
    {
        val |= signextend;
    }
    return val;
}

BOOL stepCounter(void)
{
    int x, y, z;
    x = GetAccVal('x');
    if (x < 0)
    {
        x = -x;
    }
    y = GetAccVal('y');
    if (y < 0)
    {
        y = -y;
    }
    z = GetAccVal('z');
    if (z < 0)
    {
        z = -z;
    }

    if ((x > 50 && y > 40) || (x > 50 && z > 100) || (y > 40 && z > 100))
    {
        if ((x != stepI[0] != stepIFlag[0]) || (y != stepI[1] != stepIFlag[1]) || (z != stepI[2] != stepIFlag[2]))
        {
            ++stepCount;
            steps[119]++;
            if (steps[119] >= 100)
            {
                steps[119] = 100;
            }
            dayStep++;
            return 1;
        }
    }
    stepIFlag[0] = stepI[0];
    stepIFlag[1] = stepI[1];
    stepIFlag[2] = stepI[2];
    stepI[0] = x;
    stepI[1] = y;
    stepI[2] = z;
    return 0;
}
/********************************************************************
         * Function:        void main(void)
         *
         * PreCondition:    None
         *
         * Input:           None
         *
         * Output:          None
         *
         * Side Effects:    None
         *
         * Overview:        Main program entry point.
         *
         * Note:            None
         *******************************************************************/
void main(void)
{
    InitializeSystem();

    clockType = 0; //0-digital, 1-analog
    TDdata.hour = 23;
    TDdata.min = 22;
    TDdata.sec = 0;
    TDdata.day = 1;
    TDdata.mon = 1;

    T0CON = 0x07;
    RCONbits.IPEN = 1;      //Enable priority
    INTCON2bits.TMR0IP = 1; //define as high priority
    INTCON = 0xE0;          //Enable Timer Interrupt
    T0CON |= 0x80;          //Start the Timer

    anClockStart = 0;
    clearScreen();

    while (1) //Main is Usualy an Endless Loop
    {
        if (stepCounter() == 1)
        {
            f1 = 1;
        }
        if (clockType == 0)
        {
            anClockStart = 0;
            digitalClock(1);
        }
        else
        {
            if (clockType == 1)
            {
                analogClock();
            }
            else
            {
                anClockStart = 0;
                stepGraph();
            }
        }
        if (press_b())
        {
            anClockStart = 0;
            setMenu();
        }
    }
} //end main

/** EOF main.c *************************************************/
//#endif
