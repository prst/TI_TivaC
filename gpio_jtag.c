//*****************************************************************************
//
// gpio_jtag.c - Example to demonstrate recovering the JTAG interface.
//
// Copyright (c) 2012-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 1.0 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>GPIO JTAG Recovery (gpio_jtag)</h1>
//!
//! This example demonstrates changing the JTAG pins into GPIOs, aint32_t with a
//! mechanism to revert them to JTAG pins.  When first run, the pins remain in
//! JTAG mode.  Pressing the left button will toggle the pins between JTAG mode
//! and GPIO mode.  Because there is no debouncing of the push button (either
//! in hardware or software), a button press will occasionally result in more
//! than one mode change.
//!
//! In this example, four pins (PC0, PC1, PC2, and PC3) are switched.
//!
//! UART0, connected to the ICDI virtual COM port and running at 115,200,
//! 8-N-1, is used to display messages from this application.
//
//*****************************************************************************

#define BTN_GPIO_BASE           ( GPIO_PORTF_BASE )
#define BTN1                    ( GPIO_PIN_4 )    // SW1
#define BTN2                    ( GPIO_PIN_0 )    // SW2
#define BTN_INPUTS              ( GPIO_PIN_0 | GPIO_PIN_4 )

#define PS2_GPIO_BASE           ( GPIO_PORTB_BASE )
#define PS2_SYNC                ( GPIO_PIN_2 )    // Yellow
#define PS2_DATA                ( GPIO_PIN_3 )    // Orange
#define PS2_INPUTS              ( PS2_SYNC | PS2_DATA )

#define PULSE_FINISHED_NOT 		(1)
#define PULSE_FINISHED_YES 		(0)
#define PULSE_1_SEC        		(1400000)
#define PULSE_1_MSEC       		(1400)
#define PULSE_10_USEC      		(14)
#define PULSE_40_USEC      		(14*4)
#define T_PULSE_CLK        		(PULSE_10_USEC)    // Yellow
#define T_PULSE_DAT        		(PULSE_10_USEC*4)  // Orange
#define N_BITS             		(11)

#define LED_RED 				GPIO_PIN_1 // Red
#define LED_BLUE 				GPIO_PIN_2 // Blue
#define LED_GREEN 				GPIO_PIN_3 // Green
#define ALL_OUT_PINS            (LED_RED|LED_BLUE|LED_GREEN)


uint8_t g_ui8PS2States = PS2_INPUTS;
uint8_t g_ui8BTNStates = BTN_INPUTS;

//*****************************************************************************
// The current mode of pins PC0, PC1, PC2, and PC3.  When zero, the pins
// are in JTAG mode; when non-zero, the pins are in GPIO mode.
//*****************************************************************************
volatile uint32_t g_ui32Mode;
extern uint8_t g_ui8PS2States;



//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif



void Init_PS2_pins (void);
void ConfigureUART (void);
void SysTickIntHandler (void);
void Delay (uint32_t ui32Seconds);

//*****************************************************************************
//
// Delay for the specified number of seconds.  Depending upon the current
// SysTick value, the delay will be between N-1 and N seconds (i.e. N-1 full
// seconds are guaranteed, aint32_t with the remainder of the current second).
//
//*****************************************************************************
void Delay (uint32_t ui32Seconds)
{
    // Loop while there are more seconds to wait.
    while(ui32Seconds--)
    {
        // Wait until the SysTick value is less than 1000.
        while(ROM_SysTickValueGet() > 1000)
        {
        }

        // Wait until the SysTick value is greater than 1000.
        while(ROM_SysTickValueGet() < 1000)
        {
        }
    }
}


//*****************************************************************************
// The interrupt handler for the PB4 pin interrupt.
// When triggered, this will toggle the JTAG pins between
// JTAG and GPIO mode.
//*****************************************************************************
void SysTickIntHandler (void)
{
    uint8_t ui8Buttons;
    uint8_t ui8ButtonsChanged;

    // Grab the current, debounced state of the buttons.
    ui8Buttons = ButtonsPoll(&ui8ButtonsChanged, 0);

    // If the left button has been pressed, and was previously not pressed,
    // start the process of changing the behavior of the JTAG pins.
    if(BUTTON_PRESSED(LEFT_BUTTON, ui8Buttons, ui8ButtonsChanged))
    {
        // Toggle the pin mode.
        g_ui32Mode ^= 1;

        // See if the pins should be in JTAG or GPIO mode.
        if(g_ui32Mode == 0)
        {
            // Change PC0-3 into hardware (i.e. JTAG) pins.
            HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
            HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x01;
            HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= 0x01;
            HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
            HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x02;
            HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= 0x02;
            HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
            HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x04;
            HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= 0x04;
            HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
            HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x08;
            HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= 0x08;
            HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
            HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x00;
            HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = 0;

            // Turn on the LED to indicate that the pins are in JTAG mode.
            ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3,  GPIO_PIN_3);
        } else {
            // Change PC0-3 into GPIO inputs.
            HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
            HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x01;
            HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) &= 0xfe;
            HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
            HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x02;
            HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) &= 0xfd;
            HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
            HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x04;
            HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) &= 0xfb;
            HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
            HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x08;
            HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) &= 0xf7;
            HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
            HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x00;
            HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = 0;
            ROM_GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3) );

            // Turn off the LED to indicate that the pins are in GPIO mode.
            ROM_GPIOPinWrite( GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3,  GPIO_PIN_1 );
        }
    }
}


void ConfigureInputsHWJTAG (void)
{
    // Change PC0-3 into hardware (i.e. JTAG) pins.
    HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x01;
    HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= 0x01;
    HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x02;
    HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= 0x02;
    HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x04;
    HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= 0x04;
    HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x08;
    HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= 0x08;
    HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x00;
    HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = 0;

    // Turn on the LED to indicate that the pins are in JTAG mode.
    //ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3,  GPIO_PIN_3);
}


void ConfigureInputsGPIO (void)
{
    // Change PC0-3 into GPIO inputs.
    HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x01;
    HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) &= 0xfe;
    HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x02;
    HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) &= 0xfd;
    HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x04;
    HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) &= 0xfb;
    HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x08;
    HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) &= 0xf7;
    HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTC_BASE + GPIO_O_CR) = 0x00;
    HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = 0;
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3) );

    // Turn off the LED to indicate that the pins are in GPIO mode.
    ROM_GPIOPinWrite( GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3,  GPIO_PIN_1 );
}


//*****************************************************************************
// Configure the UART and its pins.  This must be called before UARTprintf().
//*****************************************************************************
void ConfigureUART (void)
{
    // Enable the GPIO Peripheral used by the UART.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);
}



void Init_PS2_pins (void);

void Init_PS2_pins (void) {
    HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTB_BASE + GPIO_O_CR) |= 1<<2;
    HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = 0;

    HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTB_BASE + GPIO_O_CR) |= 1<<3;
    HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = 0;

    /*ROM_GPIODirModeSet  (GPIO_PORTB_BASE, PS2_INPUTS, GPIO_DIR_MODE_IN);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE, PS2_INPUTS, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    g_ui8PS2States = ROM_GPIOPinRead(PS2_GPIO_BASE, PS2_INPUTS);*/

    ROM_GPIODirModeSet  (GPIO_PORTB_BASE, PS2_DATA, GPIO_DIR_MODE_IN);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE, PS2_DATA, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    g_ui8PS2States = ROM_GPIOPinRead(PS2_GPIO_BASE, PS2_DATA);

    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, PS2_SYNC);
    ROM_GPIOPinWrite(GPIO_PORTB_BASE, PS2_SYNC, PS2_SYNC);
}


enum {
	DLY_PS2_CLK=0,
	DLY_PS2_DATA
} DLY_ENUM_SIZE;

typedef struct {
	uint32_t Val;
	uint8_t  InProcess;
	uint8_t  Done;
} Delay_uSec;

typedef enum {
	PS2_SET_BIT = 0,
	PS2_SET_CLK,
	PS2_CLR_CLK
} ps2_bit_state;

ps2_bit_state bit_stat;
Delay_uSec    Delay_uS[3]  = {0};
uint8_t       pulse_state  = PULSE_FINISHED_NOT;
uint8_t       led_state    = 0;


uint8_t ps2_pulse_clk_uS ( uint32_t delay_uS ) {
	if ( Delay_uS[DLY_PS2_CLK].InProcess==0 )
	{
		Delay_uS[DLY_PS2_CLK].InProcess=1;
		Delay_uS[DLY_PS2_CLK].Val = delay_uS;
		ROM_GPIOPinWrite(GPIO_PORTB_BASE, PS2_SYNC, PS2_SYNC);
	}
	Delay_uS[DLY_PS2_CLK].Val -= 10;
	if ( Delay_uS[DLY_PS2_CLK].Val == 0 )
	{
		Delay_uS[DLY_PS2_CLK].Done=1;
		Delay_uS[DLY_PS2_CLK].InProcess=0;
		ROM_GPIOPinWrite(GPIO_PORTB_BASE, PS2_SYNC, 0);
		return PULSE_FINISHED_YES;
	}
	Delay_uS[DLY_PS2_CLK].Done=0;
	return PULSE_FINISHED_NOT;
}


uint8_t pulse_ps2_dat_uS ( uint32_t delay_uS ) {
	if ( Delay_uS[DLY_PS2_DATA].InProcess==0 )
	{
		Delay_uS[DLY_PS2_DATA].InProcess=1;
		Delay_uS[DLY_PS2_CLK].Val = delay_uS;
		//ROM_GPIOPinWrite(GPIO_PORTB_BASE, PS2_DATA, PS2_DATA);
	}
	Delay_uS[DLY_PS2_DATA].Val -= 10;
	if ( Delay_uS[DLY_PS2_DATA].Val == 0 )
	{
		Delay_uS[DLY_PS2_DATA].Done=1;
		Delay_uS[DLY_PS2_DATA].InProcess=0;
		//ROM_GPIOPinWrite(GPIO_PORTB_BASE, PS2_DATA, 0);
		return PULSE_FINISHED_YES;
	}
	Delay_uS[DLY_PS2_DATA].Done=0;
	return PULSE_FINISHED_NOT;
}


/* return 0 if complete, else 1*/
uint8_t ps2_Dat_Parallel2Serial ( uint8_t par_in )
{
	// Parallel to Serial conversion
	static uint8_t  ser;
	uint8_t         rc=0;

	// send 11 bit: 1_start+8_data+1_parity+1_stop
	for ( ser=0; ser<N_BITS; ser++ )
	{
		pulse_ps2_dat_uS(T_PULSE_DAT);

		/* - Parallel to serial data and select bit ! <=,
		   - Set data bit #N - Change state in begin    |
		   - Wait 10 uS                                 |
		   - Set on clk strobe for short time           |
		   - wait 20 uS                                 |
		   - Set off clk strobe                         |
		   - Wait 10 uS  ==============================='  */
		if (Delay_uS[DLY_PS2_DATA].Val>(T_PULSE_DAT-(T_PULSE_DAT*0.25))) {
			// set date
			bit_stat = PS2_SET_BIT;
		}
		else
		if (Delay_uS[DLY_PS2_DATA].Val>(T_PULSE_DAT-(T_PULSE_DAT*0.5))) {
			// start clk pulse
			bit_stat = PS2_SET_CLK;
		}
		else
		if (Delay_uS[DLY_PS2_DATA].Val>(T_PULSE_DAT-(T_PULSE_DAT*0.75))) {
			// stop clk pulse
			bit_stat = PS2_CLR_CLK;
		}

		switch ( bit_stat ) {
			case PS2_SET_BIT: // Send data bit
				if ( par_in & (1<<ser) == 1 ) {
					// set 1 to data pin
					ROM_GPIOPinWrite(GPIO_PORTB_BASE, PS2_DATA, PS2_DATA);
					ROM_GPIOPinWrite(GPIO_PORTF_BASE, PS2_DATA, PS2_DATA);
				} else {
					// set 0 to data pin
					ROM_GPIOPinWrite(GPIO_PORTB_BASE, PS2_DATA, 0);
					ROM_GPIOPinWrite(GPIO_PORTF_BASE, PS2_DATA, 0);
				}
				break;
			case PS2_SET_CLK: // Send 0 clk pulse to clocl pin
			case PS2_CLR_CLK: // Send 1 clk pulse to clock pin
				// set and unset pulse
				// Generate PS2 clock
				if ( Delay_uS[DLY_PS2_CLK].InProcess==0 && Delay_uS[DLY_PS2_CLK].Done==0){
					/*pulse_state =*/ ps2_pulse_clk_uS( T_PULSE_CLK );
				}
				else
				if ( Delay_uS[DLY_PS2_CLK].InProcess==1 && Delay_uS[DLY_PS2_CLK].Done==0){
					if ( PULSE_FINISHED_YES==ps2_pulse_clk_uS(T_PULSE_CLK) ) {
						Delay_uS[DLY_PS2_CLK].Done=0;

						if (led_state) {
							led_state=0;
							ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
						} else {
							led_state=1;
							ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
						}

					}
				}
				break;
			default: // ...
				rc = 1;
				break;
		}
	}
	return rc;
}


//*****************************************************************************
//
// Toggle the JTAG pins between JTAG and GPIO mode with a push
// button selecting between the two.
//
//*****************************************************************************
int main (void)
{
    uint32_t ui32Mode=0;
	uint32_t SysClk;

    // Enable lazy stacking for interrupt handlers.  This allows
	// floating-point instructions to be used within interrupt
	// handlers, but at the expense of extra stack usage.
    ROM_FPULazyStackingEnable();

    // Set the clocking to run directly from the crystal.
    ROM_SysCtlClockSet( SYSCTL_SYSDIV_4   | SYSCTL_USE_PLL  |
    		            SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN );

    // Enable the peripherals used by this application.
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB );
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOC );

    // Initialize the button driver.
    ButtonsInit();
    //ConfigureInputsGPIO();

    Init_PS2_pins();

    // Set up a SysTick Interrupt to handle polling and debouncing for our
    // buttons.
    SysTickPeriodSet(SysCtlClockGet() / 100);
    SysTickIntEnable();
    SysTickEnable();

    IntMasterEnable();

    // Configure the LED as an output and turn it on.
    ROM_GPIOPinTypeGPIOOutput( GPIO_PORTF_BASE, ALL_OUT_PINS );
    ROM_GPIOPinWrite( GPIO_PORTF_BASE, ALL_OUT_PINS, 0 ); // Clean all leds


    // Set the global and local indicator of pin mode to zero, meaning JTAG.
    g_ui32Mode = 0;
    //ui32Mode = 0;

    // Initialize the UART.
    ConfigureUART();

    UARTprintf("\033[2JGPIO <-> JTAG\n");

    // Indicate that the pins start out as JTAG.
    UARTprintf("Pins are JTAG\n");

    ROM_GPIOPinWrite(GPIO_PORTF_BASE, ALL_OUT_PINS, 0);

    SysClk = SysCtlClockGet();
    {
        ps2_Dat_Parallel2Serial( 0xFF );
    	SysCtlDelay(SysClk/100); // Delay 10mSec for a bit.
    }

    ROM_GPIOPinWrite(GPIO_PORTF_BASE, ALL_OUT_PINS, 0 ); // Clear all Leds

    // Loop forever.  This loop simply exists to display on the
    // UART the current state of PC0-3; the handling of changing
    // the JTAG pins to and from GPIO mode is done in GPIO
    // Interrupt Handler.
    while (1)
    {
        SysClk = SysCtlClockGet();
#if (0)

        // Wait until the pin mode changes.
        while(g_ui32Mode == ui32Mode) { }

        // Save the new mode locally so that a subsequent pin
        // mode change can be detected.
        ui32Mode = g_ui32Mode;

        // See what the new pin mode was changed to.
        if ( ui32Mode == 0 ) {
            // Indicate that PC0-3 are currently JTAG pins.
            UARTprintf("Pins are JTAG\n");
        } else {
            // Indicate that PC0-3 are currently GPIO pins.
            UARTprintf("Pins are GPIO\n");
        }

#else
        //static uint8_t  ttt=0;

        //ROM_GPIOPinWrite(GPIO_PORTF_BASE, ALL_OUT_PINS, GPIO_PIN_3);

        /*ROM_GPIOPinWrite(GPIO_PORTB_BASE, PS2_SYNC, PS2_SYNC);
    	SysCtlDelay(SysClk/1000000); // Delay 1uS for a bit.

        ROM_GPIOPinWrite(GPIO_PORTB_BASE, PS2_SYNC, 0);
    	SysCtlDelay(SysClk/1000000); // Delay 1uS for a bit. */

        g_ui8BTNStates = ROM_GPIOPinRead(BTN_GPIO_BASE, BTN2);
        if (  0 == g_ui8BTNStates&BTN2 ) {
            ROM_GPIOPinWrite(GPIO_PORTF_BASE, ALL_OUT_PINS, LED_BLUE );  // Blue
        } else {
            ROM_GPIOPinWrite(GPIO_PORTF_BASE, ALL_OUT_PINS, LED_GREEN );  // Green
        }

        //g_ui8PS2States = ROM_GPIOPinRead(PS2_GPIO_BASE, PS2_DATA);
        //if ( PS2_DATA != g_ui8PS2States & PS2_DATA ) {
        if ( 1 ) {
            //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_2); // Turn on the BLUE LED.
            //SysCtlDelay(SysCtlClockGet() / 10 / 3); // Delay for a bit.
            //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0); // Turn off the BLUE LED.
            //SysCtlDelay(SysCtlClockGet() / 10 / 3); // Delay for a bit.

            //g_ui8PS2States = ROM_GPIOPinRead(PS2_GPIO_BASE, PS2_INPUTS);
        	//UARTprintf("%u) SysClk:%u PS2 is: 0x%X\n", ttt++, SysClk, g_ui8PS2States);
        	//UARTprintf("%u) PS/2: 0x%X\n", ttt++, g_ui8PS2States);

            //ROM_GPIOPinWrite(GPIO_PORTF_BASE, ALL_OUT_PINS, LED_RED );   // Red
            //ROM_GPIOPinWrite(GPIO_PORTF_BASE, ALL_OUT_PINS, LED_BLUE );  // Blue
            //ROM_GPIOPinWrite(GPIO_PORTF_BASE, ALL_OUT_PINS, LED_GREEN ); // Green

            /*ROM_GPIOPinWrite(GPIO_PORTF_BASE, ALL_OUT_PINS, LED_RED );
            SysCtlDelay(SysClk/1000000); // Delay for a bit.
            ROM_GPIOPinWrite(GPIO_PORTF_BASE, ALL_OUT_PINS, 0 );
            SysCtlDelay(SysClk/1000000); // Delay for a bit.*/

            //ROM_GPIOPinWrite(GPIO_PORTB_BASE, PS2_SYNC, PS2_SYNC);
        	//SysCtlDelay(SysClk/1000); // Delay 1uS for a bit.
            //ROM_GPIOPinWrite(GPIO_PORTB_BASE, PS2_SYNC, 0);
        	//SysCtlDelay(SysClk/1000); // Delay 1uS for a bit.
        } else {
        	//SysCtlDelay(SysClk/1000); // Delay 1uS for a bit.
            ROM_GPIOPinWrite(GPIO_PORTF_BASE, ALL_OUT_PINS, 0 ); // Clear all Leds
        }

#endif // 0

    }

}
