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

/* PCA9574 Register */
#define IN 0x00
#define INVRT 0x01
#define BKEN 0x02
#define PUPD 0x03
#define CFG 0x04
#define OUT 0x05
#define MSK 0x06
#define INTS 0x07

/* PCA9574 Port */
#define P0 0xFF ^ 0x01
#define P1 0xFF ^ 0x02
#define P2 0xFF ^ 0x04
#define P3 0xFF ^ 0x08
#define P4 0xFF ^ 0x08
#define P5 0xFF ^ 0x10
#define P6 0xFF ^ 0x20
#define P7 0xFF ^ 0x40

/* Task */
#define TASK_STACK_SIZE 512
#define TASK_PRIORITY 1
#define TASK2_PRIORITY 1

/* Task Delay */
#define DELAY_S(i)      (Task_sleep(((i) * 1000000) / Clock_tickPeriod))
#define DELAY_MS(i)      (Task_sleep(((i) * 1000) / Clock_tickPeriod))
#define DELAY_US(i)      (Task_sleep(((i) * 1) / Clock_tickPeriod))
#define MS_2_TICKS(ms)   (((ms) * 1000) / Clock_tickPeriod)

#define PCA9574_ADDR 0x20

/* LCD Size */
#define WIDTH 240
#define HEIGHT 280
#define OFFSETX 52
#define OFFSETY 40

/* Function */
void gpio_init();
void PCA9574_reset();

int spi_init();
void spi_write_command(uint8_t data);
void spi_write_data(uint8_t *data, size_t size);
void spi_write_data_byte(uint8_t data);
void spi_master_write_addr(uint16_t addr1, uint16_t addr2);
void spi_master_write_color(uint16_t color, uint16_t size);
void lcdDrawFillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
//void spi_read();

void mainTask();

int i2c_init();
void i2c_write(uint_least8_t address, uint8_t writeBuffer[], size_t count);
void i2c_read(uint_least8_t address, uint8_t writeBuffer[],
              uint8_t readBuffer[], size_t count);

void PCA9574_config_reg();
void ST7789V2_reset();

void uartInit();
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
    gpio_init();
    i2c_init();
    spi_init();

    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = appTaskStack;
    taskParams.stackSize = TASK_STACK_SIZE;
    taskParams.priority = TASK_PRIORITY;

    Task_construct(&task0, mainTask, &taskParams, NULL);

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
void gpio_init()
{
    //Initialize GPIO
    GPIO_init();
    GPIO_setConfig(CONFIG_PCA9574_RESET, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH); //I/O Exp Set Config
    GPIO_setConfig(CONFIG_GPIO_LCD_DC, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW); //LCD DC Set Config
}

int i2c_init()
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

void PCA9574_reset()
{
    // Initialize slave address of transaction
    DELAY_US(10);

    GPIO_write(CONFIG_PCA9574_A0, 0);
    DELAY_US(1);

    GPIO_write(CONFIG_PCA9574_RESET, 0);
    DELAY_US(1);
    GPIO_write(CONFIG_PCA9574_RESET, 1);
}

void i2c_write(uint_least8_t address, uint8_t writeBuffer[], size_t count)
{
    I2C_Transaction transaction = { 0 };
    bool status = false;

    transaction.writeBuf = writeBuffer;
    transaction.writeCount = count;
    transaction.readBuf = NULL;
    transaction.readCount = 0;
    transaction.slaveAddress = address; //0x40 write bit 를 제외하고 7bit
    status = I2C_transfer(i2cHandle, &transaction);
}

void i2c_read(uint_least8_t address, uint8_t writeBuffer[],
              uint8_t readBuffer[], size_t count)
{
    I2C_Transaction transaction = { 0 };
    bool status = false;

    transaction.writeBuf = writeBuffer;
    transaction.writeCount = 1;
    transaction.readBuf = readBuffer;
    transaction.readCount = count;
    transaction.slaveAddress = address; //0x40 write bit 를 제외하고 7bit
    status = I2C_transfer(i2cHandle, &transaction);
}

void PCA9574_config_reg()
{

    uint8_t writeBuffer[2] = { CFG, P1 };
    i2c_write(PCA9574_ADDR, writeBuffer, 2);
}

void ST7789V2_reset()
{
    uint8_t writeBuffer[2] = { OUT, P1 };

    /*Hardware reset*/
    DELAY_MS(25);
    i2c_write(PCA9574_ADDR, writeBuffer, 2);

    DELAY_MS(25);
    writeBuffer[1] = ~P1;
    i2c_write(PCA9574_ADDR, writeBuffer, 2);

}
void mainTask()
{
    PCA9574_reset();
    PCA9574_config_reg();
    ST7789V2_reset();

    spi_write_command(0x01);    //Software Reset
    DELAY_MS(150);

    spi_write_command(0x11);    //Sleep Out
    DELAY_MS(255);

    spi_write_command(0x3A);    //Interface Pixel Format
    spi_write_data_byte(0x55);
    DELAY_MS(10);

    spi_write_command(0x36);    //Memory Data Access Control
    spi_write_data_byte(0x00);

    spi_write_command(0x2A);    //Column Address Set
    spi_write_data_byte(0x00);
    spi_write_data_byte(0x00);
    spi_write_data_byte(0x00);
    spi_write_data_byte(0xF0);

    spi_write_command(0x2B);    //Row Address Set
    spi_write_data_byte(0x00);
    spi_write_data_byte(0x00);
    spi_write_data_byte(0x00);
    spi_write_data_byte(0xF0);

    spi_write_command(0x21);    //Display Inversion On
    DELAY_MS(10);

    spi_write_command(0x13);    //Normal Display Mode On
    DELAY_MS(10);

    spi_write_command(0x29);    //Display ON
    DELAY_MS(255);
//    uint8_t data[4] = {0x00, 0x01, 0x02, 0x03};
//    spi_write_data(data, 4);

    lcdDrawFillRect(0, 0, WIDTH-1, HEIGHT-1, 0xf800);

}

void lcdDrawFillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {

    if (x1 >= WIDTH) return;
    if (x2 >= WIDTH) x2=WIDTH-1;
    if (y1 >= HEIGHT) return;
    if (y2 >= HEIGHT) y2=HEIGHT-1;

    uint16_t _x1 = x1 + OFFSETX;
    uint16_t _x2 = x2 + OFFSETX;
    uint16_t _y1 = y1 + OFFSETY;
    uint16_t _y2 = y2 + OFFSETY;

    spi_write_command(0x2A);    // set column(x) address
    spi_master_write_addr(_x1, _x2);

    spi_write_command(0x2B);    // set Page(y) address
    spi_master_write_addr(_y1, _y2);

    spi_write_command(0x2C);    //  Memory Write
    int i = 0;
    for(i =_x1; i <= _x2; i++){
        uint16_t size = _y2-_y1+1;
        spi_master_write_color(color, size);
    }
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

/* spi function */
int spi_init()
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
void spi_write_command(uint8_t data)
{
    GPIO_write(CONFIG_GPIO_LCD_DC, 0);

    SPI_Transaction spiTransaction;
    // Fill in transmitBuffer
    uint8_t transmitBuffer;
    uint8_t receiveBuffer;
    bool transferOK;

    receiveBuffer = 0;
    transmitBuffer = data;

    spiTransaction.count = 1;
    spiTransaction.txBuf = &transmitBuffer;
    spiTransaction.rxBuf = &receiveBuffer;
    transferOK = SPI_transfer(spi, &spiTransaction);
    if (!transferOK)
    {
        // Error in SPI or transfer already in progress.
        while (1)
            ;
    }
}
void spi_write_data_byte(uint8_t data)
{
//    DELAY_US(10);
    GPIO_write(CONFIG_GPIO_LCD_DC, 1);

    SPI_Transaction spiTransaction;
    // Fill in transmitBuffer
    uint8_t transmitBuffer;
    uint8_t receiveBuffer;
    bool transferOK;

    receiveBuffer = 0;
    transmitBuffer = data;
    spiTransaction.count = 1;
    spiTransaction.txBuf = &transmitBuffer;
    spiTransaction.rxBuf = &receiveBuffer;
    transferOK = SPI_transfer(spi, &spiTransaction);
    if (!transferOK)
    {
        // Error in SPI or transfer already in progress.
        while (1)
            ;
    }
}
void spi_write_data(uint8_t* data, size_t size)
{
    //    DELAY_US(10);
    GPIO_write(CONFIG_GPIO_LCD_DC, 1);

    SPI_Transaction spiTransaction;
    // Fill in transmitBuffer
    uint8_t* transmitBuffer;
    uint8_t receiveBuffer;
    bool transferOK;

    receiveBuffer = 0;
    transmitBuffer = data;
    spiTransaction.count = size;
    spiTransaction.txBuf = data;
    spiTransaction.rxBuf = &receiveBuffer;
    transferOK = SPI_transfer(spi, &spiTransaction);
    if (!transferOK)
    {
        // Error in SPI or transfer already in progress.
        while (1)
            ;
    }
}
void spi_master_write_addr(uint16_t addr1, uint16_t addr2)
{
    static uint8_t Byte[4];
    Byte[0] = (addr1 >> 8) & 0xFF;
    Byte[1] = addr1 & 0xFF;
    Byte[2] = (addr2 >> 8) & 0xFF;
    Byte[3] = addr2 & 0xFF;
    GPIO_write(CONFIG_GPIO_LCD_DC, 1);
    return spi_write_data(Byte, 4);
}

void spi_master_write_color(uint16_t color, uint16_t size)
{
    static uint8_t Byte[1024];
    int index = 0;
    int i = 0 ;
    for(i=0; i<size; i++) {
        Byte[index++] = (color >> 8) & 0xFF;
        Byte[index++] = color & 0xFF;
    }
    GPIO_write(CONFIG_GPIO_LCD_DC, 1);
    return spi_write_data(Byte, size*2);
}
//void spi_read()
//{
//    int MSGSIZE = 8;
//    SPI_Transaction spiTransaction;
//    // Fill in transmitBuffer
//    uint8_t transmitBuffer[MSGSIZE];
//    uint8_t receiveBuffer[MSGSIZE];
//    bool transferOK;
//
//    spiTransaction.count = MSGSIZE;
//    spiTransaction.txBuf = (void*) transmitBuffer;
//    spiTransaction.rxBuf = (void*) receiveBuffer;
//    transferOK = SPI_transfer(spi, &spiTransaction);
//    if (!transferOK)
//    {
//        // Error in SPI or transfer already in progress.
//        while (1)
//            ;
//    }
//}
