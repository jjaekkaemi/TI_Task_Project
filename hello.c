/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
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
 *  ======== hello.c ========
 */

/* XDC Module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/Board.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* Task */
#define TASK_STACK_SIZE 512
#define TASK_PRIORITY 1
#define TASK2_PRIORITY 1

/* Task Delay */
#define DELAY_S(i)      (Task_sleep(((i) * 1000000) / Clock_tickPeriod))
#define DELAY_MS(i)      (Task_sleep(((i) * 1000) / Clock_tickPeriod))
#define DELAY_US(i)      (Task_sleep(((i) * 1) / Clock_tickPeriod))
#define MS_2_TICKS(ms)   (((ms) * 1000) / Clock_tickPeriod)

/*
 *  ======== main ========
 */

/* Global */
uint8_t appTaskStack[TASK_STACK_SIZE];
uint8_t appTaskStack2[TASK_STACK_SIZE];
Task_Struct task0;
Task_Struct task1;

UART_Handle uart;
UART_Params uartParams;

void uartInit(){
    // Initialize the UART driver.
    UART_init();
    // Create a UART with data processing off.
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 9600;
    // Open an instance of the UART drivers
    uart = UART_open(0, &uartParams);
    if (uart == NULL) {
        // UART_open() failed
        while (1);
    }
}
 void uartTask1(UArg arg0, UArg arg1){

    while (1) {
        // UART_read(uart, &input, 1);
        DELAY_S(1);
        UART_write(uart, "wwww\r\n", 6);
    }
 }
  void uartTask2(UArg arg0, UArg arg1){

    while (1) {
        // UART_read(uart, &input, 1);
        DELAY_S(2);
        UART_write(uart, "hhh\r\n", 5);
    }
 }

int main()
{

    /* Call driver init functions */
    Board_init();
    uartInit();
//    GPIO_init();
//
//    GPIO_setConfig(CONFIG_PCA9574_RESET, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
//    /* Turn on user LED */
//    GPIO_write(CONFIG_PCA9574_RESET, 0);
//    DELAY_US(10);
//    GPIO_write(CONFIG_PCA9574_RESET, 1);

    // System_printf("hello world\n");

    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = appTaskStack;
    taskParams.stackSize = TASK_STACK_SIZE;
    taskParams.priority = TASK_PRIORITY;

    Task_construct(&task0, uartTask1, &taskParams, NULL);

    // Configure task
    taskParams.stack = appTaskStack2;
    taskParams.stackSize = TASK_STACK_SIZE;
    taskParams.priority = TASK2_PRIORITY;

    Task_construct(&task1, uartTask2, &taskParams, NULL);

    /*
     *  normal BIOS programs, would call BIOS_start() to enable interrupts
     *  and start the scheduler and kick BIOS into gear.  But, this program
     *  is a simple sanity test and calls BIOS_exit() instead.
     */
    // BIOS_exit(0);  /* terminates program and dumps SysMin output */
    BIOS_start();
    return(0);
}
