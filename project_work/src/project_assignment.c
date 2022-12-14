/*
    FreeRTOS V8.2.1 - Copyright (C) 2015 Real Time Engineers Ltd.

    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?".  Have you defined configASSERT()?  *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    ***************************************************************************
     *                                                                       *
     *   Investing in training allows your team to be as productive as       *
     *   possible as early as possible, lowering your overall development    *
     *   cost, and enabling you to bring a more robust product to market     *
     *   earlier than would otherwise be possible.  Richard Barry is both    *
     *   the architect and key author of FreeRTOS, and so also the world's   *
     *   leading authority on what is the world's most popular real time     *
     *   kernel for deeply embedded MCU designs.  Obtaining your training    *
     *   from Richard ensures your team will gain directly from his in-depth *
     *   product knowledge and years of usage experience.  Contact Real Time *
     *   Engineers Ltd to enquire about the FreeRTOS Masterclass, presented  *
     *   by Richard Barry:  http://www.FreeRTOS.org/contact
     *                                                                       *
    ***************************************************************************

    ***************************************************************************
     *                                                                       *
     *    You are receiving this top quality software for free.  Please play *
     *    fair and reciprocate by reporting any suspected issues and         *
     *    participating in the community forum:                              *
     *    http://www.FreeRTOS.org/support                                    *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!

    Modified by lindh LUT
*/

/* Standard C libraries*/
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"
#include "xil_types.h"
#include "xttcps.h"

/* LUT includes. */
#include "zynq_registers.h"


// Binary macros for BTNs
#define BUTTON_0 0b0001		//BTN0
#define BUTTON_1 0b0010		//BTN1
#define BUTTON_2 0b0100		//BTN2
#define BUTTON_3 0b1000 	//BTN3

// Macros for configuration parameters
#define K_P 0
#define K_P_STR "kP"
#define K_I 1
#define K_I_STR "kI"

// State macros
#define CONFIG 0
#define IDLE 1
#define MODULATE 2
// State macros in string format for printing
#define CONFIG_STR "Configuration state"
#define IDLE_STR "Idle state"
#define MODULATE_STR "Modulating state"

// Tasks declaration
static void btn_handler();
static void modulating();
static void PWM();

// Function declarations
static void PI(float y_act);
static float convmodel();
void convert_float(int fract_size, float value,  int* base, int* fraction);
void print_menu();
float fl_abs(float x);

// Semaphore handlers
xSemaphoreHandle BTNsem;
xSemaphoreHandle LEDsem;
xSemaphoreHandle GLOBALsem;

//Global variables
static float kI = 0.0;		// kI parameter - initialized as zero
static float kP = 1.0;		// kP parameter - initialized as a small value
static float volt_ref = 5;	// Reference voltage
static float u1_max = 10;	// Saturation control voltage
static float ctrl_out = 0;	// Controller output voltage
static int state = IDLE;	// State currently executing
static int k_state = K_P;	// Current state of k parameters

/*** MAIN FUNCTION ***/

// Initial prints, BTNs, LEDs and Counters initialization, Task scheduling and Semaphore initializations
int main( void ) {
	 AXI_BTN_TRI |= 0xF; 		// Set Buttons to input
     AXI_LED_TRI = ~0xF;		// Set LEDs to output

     // Initialize triple counter and RGB LED
     TTC0_CLK_CNTRL  = (0 << XTTCPS_CLK_CNTRL_PS_VAL_SHIFT) | XTTCPS_CLK_CNTRL_PS_EN_MASK;
     TTC0_CNT_CNTRL  = XTTCPS_CNT_CNTRL_RST_MASK | XTTCPS_CNT_CNTRL_DIS_MASK | XTTCPS_CNT_CNTRL_MATCH_MASK | XTTCPS_CNT_CNTRL_POL_WAVE_MASK;
     TTC0_MATCH_0    = 0;
     TTC0_CNT_CNTRL  &= ~XTTCPS_CNT_CNTRL_DIS_MASK;

     print_menu();								// Print User's menu
     xil_printf( "=! %s !=\n\n", IDLE_STR);		// Print current state


	/* Create tasks */

    /*
     * Modulating task
     * This task handles the PI controller and Converter
     * Executing periodically, has the highest priority
    */
	xTaskCreate( 	modulating,
					"Modulating",
					configMINIMAL_STACK_SIZE,
					NULL,
					tskIDLE_PRIORITY+2,
					NULL );

	/*
	 * Button handling task
	 * This task handles pressing of the buttons, task replaces interrupts
	 * Periodically checking, if any button is pressed
	 * If button is pressed, certain action is taken
	*/
	xTaskCreate( 	btn_handler,
					"Button_handler",
					configMINIMAL_STACK_SIZE,
					NULL,
					tskIDLE_PRIORITY+1,
					NULL );

	/*
	 * Task handling the PWM output
	 * Task sends controller output as a RGB LED output
	 * Executing periodically
	*/
	xTaskCreate( 	PWM,
					"PWM_output",
					configMINIMAL_STACK_SIZE,
					NULL,
					tskIDLE_PRIORITY,
					NULL );

	// Initialization of Semaphores
	vSemaphoreCreateBinary(BTNsem);		// Semaphore for Button data protection
	vSemaphoreCreateBinary(LEDsem);		// Semaphore for LED data protection
	vSemaphoreCreateBinary(GLOBALsem);	// Semaphore for GLOBAL data protection

	// Initialization of LEDs
	xSemaphoreTake(LEDsem,portMAX_DELAY);
	AXI_LED_DATA = 0b0010;
	xSemaphoreGive(LEDsem);

	// Start the tasks and timer running.
	// https://www.freertos.org/a00132.html
	vTaskStartScheduler();

	//Scheduler is now running, infinite for loop is not reached if everything works correctly
	for( ;; );

	return 0;
}

/*** TASKS ***/

// Task that modulates the output voltage - highest priority
// Consists of PI controller and converter
static void modulating() {

	const TickType_t delay = pdMS_TO_TICKS( 500 );

	//Values for float printing
	//int base, fraction;
	//int fract_size = 1000;

	static float y_act = 0;
	//static float u_new = 0;

	for( ;; ) {

		if (state == MODULATE){

			PI(y_act);													// PI controller

			//base = ctrl_out;															// Printing of voltage values
			//float diff = fl_abs(ctrl_out - base);										// Commented out, only for debugging
			//fraction = diff*fract_size;
			//xil_printf( "Controller output is %d.%d V.\r\n", base, fraction );

			//convert_float(1000, u_new, &base, &fraction);
			//xil_printf( "u_new = %d.%d\r\n", base, fraction );

			y_act = convmodel();										// Converter

			//convert_float(1000, y_act, &base, &fraction);
			//xil_printf( "Output voltage is %d.%d V.\r\n", base, fraction );		// Printing output voltage
		}

		vTaskDelay(delay);
	}
}

// Task that handles pressing of buttons 0-3
static void btn_handler() {

	const TickType_t delay_fast = pdMS_TO_TICKS( 50 );		// Shorter delay -> faster checking if buttons are pressed
	const TickType_t delay_slow = pdMS_TO_TICKS( 500 );		// Longer delay -> slower checking if buttons are pressed

	// Values for float printing
	int base, fraction;

	for (;;) {
		xSemaphoreTake(BTNsem, portMAX_DELAY);		// Semaphore for safe access of BTN_DATA variable

		if (AXI_BTN_DATA != 0x0){					// IF statement tree for checking which button is pressed

			if(AXI_BTN_DATA == 1){					// Btn0 is pressed -> Configuration/Idling/Modulating

				if (state == MODULATE){				// Cycles through state modes
					state = CONFIG;
				} else {
					state++;
				}

				if (state == CONFIG){
					xil_printf( "=! %s !=\n\n", CONFIG_STR);	// Print current state
					xSemaphoreTake(LEDsem,portMAX_DELAY);
					AXI_LED_DATA = 0b0001;						// Indicate state with LEDs
					xSemaphoreGive(LEDsem);

				} else if (state == IDLE){
					xil_printf( "=! %s !=\n\n", IDLE_STR);		// Print current state
					xSemaphoreTake(LEDsem,portMAX_DELAY);
					AXI_LED_DATA = 0b0010;						// Indicate state with LEDs
					xSemaphoreGive(LEDsem);

				} else {
					xil_printf( "=! %s !=\n\n", MODULATE_STR);	// Print current state
					xSemaphoreTake(LEDsem,portMAX_DELAY);
					AXI_LED_DATA = 0b0100;						// Indicate state with LEDs
					xSemaphoreGive(LEDsem);
				}
				print_menu();									// Print part of MENU for user according to current mode

			}else if(AXI_BTN_DATA == 2){			// Btn1 is pressed -> Switch between kP and kI in configuration mode/ otherwise nothing

				if (state == CONFIG){
					if (k_state == K_P){
						k_state = K_I;
						xil_printf( "Configuring parameter: =! %s !=\n", K_I_STR);
					} else {
						k_state = K_P;
						xil_printf( "Configuring parameter: =! %s !=\n", K_P_STR);
					}
				}

			}else if(AXI_BTN_DATA == 4){			// Btn2 is pressed -> decrease parameter values or reference voltage according to current mode

				if (state == CONFIG){

					// decrease parameter values
					if (k_state == K_P){
						kP -= 0.1;
						convert_float(10, kP, &base, &fraction);
						xil_printf( "kP = %d.%d\r\n", base, fraction);		// Send the new kP value to Terminal for the user

					} else {
						kI -= 0.01;
						convert_float(100, kI, &base, &fraction);
						xil_printf( "kI = %d.%02d\r\n", base, fraction);	// Send the new kI value to Terminal for the user
					}

				} else if (state == MODULATE){
					// decrease reference voltage
					volt_ref -= 0.1;
					convert_float(10, volt_ref, &base, &fraction);
					xil_printf( "Reference Voltage = %d.%d\r\n", base, fraction);	// Send the new reference voltage value to Terminal for the user
				}

			}else if(AXI_BTN_DATA == 8){					// Btn3 is pressed -> increase parameter values or reference voltage according to current mode

				if (state == CONFIG){

					// increase parameter values
					if (k_state == K_P){
						kP += 0.1;
						convert_float(10, kP, &base, &fraction);
						xil_printf( "kP = %d.%d\r\n", base, fraction);		// Send the new kP value to Terminal for the user

					} else {
						kI += 0.01;
						convert_float(100, kI, &base, &fraction);
						xil_printf( "kI = %d.%02d\r\n", base, fraction);	// Send the new kI value to Terminal for the user
					}

				} else if (state == MODULATE){
					// increase reference voltage
					volt_ref += 0.1;
					convert_float(10, volt_ref, &base, &fraction);
					xil_printf( "Reference Voltage = %d.%d\r\n", base, fraction);	// Send the new reference voltage value to Terminal for the user
				}

			}
			// Delay for troubles with pressing the button too long or too short
			vTaskDelay(delay_slow);
		}

		xSemaphoreGive(BTNsem);								// Release of BTN semaphore
		vTaskDelay(delay_fast);								// Puts the task into delayed state
	}

}

// Indicates the controller output value with LED6
static void PWM(){

	const TickType_t delay = pdMS_TO_TICKS( 100 );
	while(1){

		if (state == MODULATE){								// Only indicate the controller output if in Modulating mode
			TTC0_MATCH_0 = fl_abs(100*ctrl_out*ctrl_out);
		} else {
			TTC0_MATCH_0 = 0;
		}

		vTaskDelay(delay);

	}
}

/*** OTHER FUNCTIONS ***/

// Function that converts float value into two integers for printing
void convert_float( int fract_size, float value,  int* base, int* fraction){

	*base = value;
	float diff = fl_abs(value - *base);
	*fraction = diff*fract_size;
}

// Function for floating absolute value conversion
float fl_abs(float x){

	if(x < 0){
		x = -1*x;
	}
	return x;
}

// PI controller function
static void PI(float y_act){

	static float u1_old = 0;
	float error_new, u1_new;

	error_new = volt_ref - y_act;

	u1_new = u1_old + kI*error_new ;

	if (fl_abs(u1_new) >= u1_max){ 				// Check for saturation
		u1_new = u1_old ; 						// Change back to previous value in case of saturation
		//xil_printf( "Saturation!! \r\n");		// Saturation print for debugging
	}

	xSemaphoreTake(GLOBALsem, portMAX_DELAY);	// Protect the ctrl_out global variable
	ctrl_out = kP*error_new + u1_new;
	xSemaphoreGive(GLOBALsem);

	u1_old = u1_new;

}

// Converter function
static float convmodel(){
	// equations implemented according to lecture slides (Matlab script)

	static float i1_kplush;
	static float u1_kplush;
	static float i2_kplush;
	static float u2_kplush;
	static float i3_kplush;
	static float u3_kplush;


	static float i1_k = 0.0;
	static float u1_k = 0.0;
	static float i2_k = 0.0;
	static float u2_k = 0.0;
	static float i3_k = 0.0;
	static float u3_k = 0.0;

	i1_kplush= 0.9652*i1_k + (-0.0172*u1_k) + 0.0057*i2_k	 + (-0.0058)*u2_k + 0.0052*i3_k    + (-0.0251)*u3_k + 0.0471*ctrl_out;
	u1_kplush= 0.7732*i1_k + 0.1252*u1_k 	+ 0.2315*i2_k	 + 0.07*u2_k	  + 0.1282*i3_k    + 0.7754*u3_k    + 0.0377*ctrl_out;
	i2_kplush= 0.8278*i1_k + (-0.7522)*u1_k + (-0.0956)*i2_k + 0.3299*u2_k	  + (-0.4855)*i3_k + 0.3915*u3_k    + 0.0404*ctrl_out;
	u2_kplush= 0.9948*i1_k + 0.2655*u1_k 	+ (-0.3848)*i2_k + 0.4212*u2_k	  + 0.3927*i3_k	   + 0.2899*u3_k    + 0.0485*ctrl_out;
	i3_kplush= 0.7648*i1_k + (-0.4165)*u1_k + (-0.4855)*i2_k + (-0.3366)*u2_k + (-0.0986)*i3_k + 0.7281*u3_k    + 0.0373*ctrl_out;
	u3_kplush= 1.1056*i1_k + 0.7587*u1_k 	+ 0.1179*i2_k 	 + 0.0748*u2_k	  + (-0.2192)*i3_k + 0.1491*u3_k    + 0.0539*ctrl_out;

	i1_k = i1_kplush;
	u1_k = u1_kplush;
	i2_k = i2_kplush;
	u2_k = u2_kplush;
	i3_k = i3_kplush;
	u3_k = u3_kplush;

return u3_kplush;

}

// Function for printing of user MENU
void print_menu(){

	if (state == CONFIG){
		xil_printf( "Press button BTN1 when in Configuration mode to select desired parameter.\n" );
		xil_printf( "\t 1. kP parameter \n" );
		xil_printf( "\t 2. kI parameter \n" );
		xil_printf( "Press button BTN2 to decrease value of: \n" );
		xil_printf( "\t 1. kP/kI parameter in Configuration mode \n" );
		xil_printf( "Press button BTN3 to increase value of: \n" );
		xil_printf( "\t 1. kP/kI parameter in Configuration mode \n\n" );

	} else if (state == MODULATE){
		xil_printf( "Press button BTN2 to decrease value of: \n" );
		xil_printf( "\t 2. Reference voltage in Modulating mode \n" );
		xil_printf( "Press button BTN3 to increase value of: \n" );
		xil_printf( "\t 2. Reference voltage in Modulating mode \n\n" );

	} else {
		xil_printf( "Press button BTN0 to cycle through executing modes.\n" );
		xil_printf( "\t 1. Configuration mode \n" );
		xil_printf( "\t 2. Idle mode \n" );
		xil_printf( "\t 3. Modulating mode \n\n" );
	}

}


