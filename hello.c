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
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>

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

#define OPT_ADDR 0x40

/* Function */
void uartInit();
int i2cInit();
void gpioInit();
int spiInit();

void i2cConfigReg();

void ioExpanderReset();
void ioExpanderLCDTask();
void spiTask();


void i2cWrite(uint8_t writeBuffer[], int count);
void uartTask1(UArg arg0, UArg arg1);
void uartTask2(UArg arg0, UArg arg1);

/* Global */
uint8_t appTaskStack[TASK_STACK_SIZE], appTaskStack2[TASK_STACK_SIZE];
Task_Struct task0, task1;

UART_Handle uart;
UART_Params uartParams;
I2C_Handle i2cHandle;
SPI_Handle spi;
/*
 *  ======== main ========
 */

int main()
{

    /* Call driver init functions */
    Board_init();

    uartInit();
    gpioInit();
    i2cInit();
    //spiInit();
    ioExpanderReset();

    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = appTaskStack;
    taskParams.stackSize = TASK_STACK_SIZE;
    taskParams.priority = TASK_PRIORITY;

    Task_construct(&task0, ioExpanderLCDTask, &taskParams, NULL);


    /*
     *  normal BIOS programs, would call BIOS_start() to enable interrupts
     *  and start the scheduler and kick BIOS into gear.  But, this program
     *  is a simple sanity test and calls BIOS_exit() instead.
     */
    // BIOS_exit(0);  /* terminates program and dumps SysMin output */
    BIOS_start();
    return (0);
}

void uartInit()
{
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
    if (uart == NULL)
    {
        // UART_open() failed
        while (1)
            ;
    }
}
void gpioInit()
{
    //Initialize GPIO
    GPIO_init();
    GPIO_setConfig(CONFIG_PCA9574_RESET, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
}

int i2cInit()
{
    //Initialize I2C
    I2C_init();

    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_100kHz;

    // Open I2C bus for usage
    i2cHandle = I2C_open(CONFIG_I2C_0, &params);

    if (i2cHandle == NULL)
    {
        return -1;
    }
    return 0;
}

void ioExpanderReset()
{
    // Initialize slave address of transaction
    DELAY_US(10);

    GPIO_write(CONFIG_PCA9574_A0, 0);
    DELAY_US(1);

    GPIO_write(CONFIG_PCA9574_RESET, 0);
    DELAY_US(1);
    GPIO_write(CONFIG_PCA9574_RESET, 1);
}

void i2cConfigReg(){

    uint8_t writeBuffer[2];

    writeBuffer[0] = 0x04;
    writeBuffer[1] = 0x00;

    i2cWrite(writeBuffer, 2);
}

void i2cWrite(uint8_t writeBuffer[], int count){
    I2C_Transaction transaction = { 0 };
    bool status = false;

    transaction.writeBuf = writeBuffer;
    transaction.writeCount = 2;
    transaction.readBuf = NULL;
    transaction.readCount = 0;
    transaction.slaveAddress = 0x20;//0x40 write bit 를 제외하고 7bit
    status = I2C_transfer(i2cHandle, &transaction);
}

void ioExpanderLCDTask()
{
    i2cConfigReg();
    uint8_t writeBuffer[2];
    uint8_t out = 0x00;
    while (1)
    {

        out = ~out;
        writeBuffer[0] = 0x05;
        writeBuffer[1] = out;

        i2cWrite(writeBuffer, 2);
//        if (status == false)
//        {
//            // Unsuccessful I2C transfer
//            if (transaction.status == I2C_STATUS_ADDR_NACK)
//            {
//                // I2C slave address not acknowledged
////                UART_write(uart, "NACK\r\n", 6);
//            }
//        }
        DELAY_S(1);
    }

    I2C_close(i2cHandle);
}

void uartTask1(UArg arg0, UArg arg1)
{

    while (1)
    {
        // UART_read(uart, &input, 1);
        DELAY_S(1);
        UART_write(uart, "wwww\r\n", 6);
    }
}

void uartTask2(UArg arg0, UArg arg1)
{

    while (1)
    {
        // UART_read(uart, &input, 1);
        DELAY_S(2);
        UART_write(uart, "hhh\r\n", 5);
    }
}

int spiInit()
{
    SPI_Params spiParams;

    SPI_init();  // Initialize the SPI driver
    SPI_Params_init(&spiParams);  // Initialize SPI parameters
    spiParams.dataSize = 8;       // 8-bit data size
    spi = SPI_open(CONFIG_SPI_0, &spiParams);
    if (spi == NULL)
    {
        return -1;
    }
    return 0;
}

void spiTask()
{
    int MSGSIZE = 10;
    SPI_Transaction spiTransaction;
    // Fill in transmitBuffer
    uint8_t transmitBuffer[MSGSIZE];
    uint8_t receiveBuffer[MSGSIZE];
    bool transferOK;

    spiTransaction.count = MSGSIZE;
    spiTransaction.txBuf = (void*) transmitBuffer;
    spiTransaction.rxBuf = (void*) receiveBuffer;
    transferOK = SPI_transfer(spi, &spiTransaction);
    if (!transferOK)
    {
        // Error in SPI or transfer already in progress.
        while (1)
            ;
    }
}
