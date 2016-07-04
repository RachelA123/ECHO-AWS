/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */

/* AWS Header files */
#include "ECHO-AWS-1_Specific_Files/ShiftRegisterInterface.h"

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
// #include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
#include <driverlib/aon_ioc.h>
#include <driverlib/osc.h>


/* Board Header files */
#include "Board.h"

#define TASKSTACKSIZE   512

/* Pin driver handle */
static PIN_Handle pinHandle;
static PIN_State ledPinState[2];

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];
volatile bool buttonState = true;
/*
 *  ======== buttonResponseFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */
void pinCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
   CPUdelay(8000*50);
   switch( pinId)
   {
      case ECHO_AWS_BUTTON:
         {
            if (!PIN_getInputValue(pinId))
            {
               buttonState = !buttonState;
            }
         }
         break;
      case ECHO_AWS_WIFI_INTERRUPT:
         {
            System_printf("WIFI Interrupt Received\n");
         }
         break;
      case ECHO_AWS_USB_INTERRUPT:
      {
         System_printf("USB Interrupt Received\n");
      }
      break;
      default:
         {
            System_printf("Unknown Interrupt Received\n");
         }
         break;
   }
}

/*
 *  ======== heartBeatFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */
Void heartBeatFxn(UArg arg0, UArg arg1)
{
   uint32_t count1, count2, output_value;

   count1 = count2 = 0;
    while (1)
    {
        Task_sleep((UInt)arg0);
        if( buttonState)
        {
           PIN_setOutputValue(pinHandle, ECHO_AWS_RED_LED, !PIN_getOutputValue(ECHO_AWS_RED_LED));
           output_value = (count2 & 0x01)? (0x01 & count1):!(0x01 & count1);

           if( (count1++ >= 8) != 0)
           {
              PIN_setOutputValue( pinHandle, ECHO_AWS_SPI0_MOSI, output_value);
           }
           else
           {
              PIN_setOutputValue( pinHandle, ECHO_AWS_SPI0_MOSI, output_value);
           }

           PIN_setOutputValue( pinHandle, ECHO_AWS_SPI0_CLK, 1);
           Task_sleep((UInt)1);
           PIN_setOutputValue( pinHandle, ECHO_AWS_SPI0_CLK, 0);

           if( count1 >= 16)
           {
              count2++;
              count1 = 0;
              PIN_setOutputValue( pinHandle, ECHO_AWS_SR_OUTPUT_ENABLE, 1);
              PIN_setOutputValue( pinHandle, ECHO_AWS_SR_LOAD_OUTPUT_REGISTER, 1);
              Task_sleep((UInt)10);
              PIN_setOutputValue( pinHandle, ECHO_AWS_SR_LOAD_OUTPUT_REGISTER, 0);
              PIN_setOutputValue( pinHandle, ECHO_AWS_SR_OUTPUT_ENABLE, 0);
           }

        }
        else
        {
           PIN_setOutputValue(pinHandle, ECHO_AWS_RED_LED, 0);
        }
    }
}

/*
 *  ======== main ========
 */
int main(void)
 {
    /* Call board init functions */
    Board_initGeneral();
    // Board_initI2C();
    // Board_initSPI();
    // Board_initUART();
    // Board_initWatchdog();

    /* Open LED pins */
    //ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    pinHandle = PIN_open(ledPinState, BoardGpioInitTable);
    if(!pinHandle)
    {
        System_abort("Error initializing board LED pins\n");
    }

    //GPIO_setCallback( ledPinHandle, gpioCallbackFunctions);

    //PIN_setOutputValue(ledPinHandle, Board_LED1, 1);

    /* Setup callback for button pins */
    //if (PIN_registerIntCb(pinHandle, pinCallbackFunctions[0]) != 0)
    if (PIN_registerIntCb(pinHandle, pinCallbackFxn) != 0)
    {
        System_abort("Error registering button callback function");
    }

    PIN_setOutputValue( pinHandle, ECHO_AWS_SR_RESET, 1);

    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */

    AONIOC32kHzOutputEnable();

    OSCClockSourceSet( OSC_SRC_CLK_LF, OSC_XOSC_LF);
    OSCHF_TurnOnXosc();


    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
