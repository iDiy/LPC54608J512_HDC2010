/*
 * hdc2010.c
 *
 *  Created on: 2019年10月22日
 *      Author: alan
 */
#include <string.h>

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/*  SDK Included Files */
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "fsl_i2c_freertos.h"

#include "pin_mux.h"

#include "hdc2010.h"

//Define Register Map
#define TEMP_LOW 0x00
#define TEMP_HIGH 0x01
#define HUMID_LOW 0x02
#define HUMID_HIGH 0x03
#define INTERRUPT_DRDY 0x04
#define TEMP_MAX 0x05
#define HUMID_MAX 0x06
#define INTERRUPT_CONFIG 0x07
#define TEMP_OFFSET_ADJUST 0x08
#define HUM_OFFSET_ADJUST 0x09
#define TEMP_THR_L 0x0A
#define TEMP_THR_H 0x0B
#define HUMID_THR_L 0x0C
#define HUMID_THR_H 0x0D
#define CONFIG 0x0E
#define MEASUREMENT_CONFIG 0x0F
#define MID_L 0xFC
#define MID_H 0xFD
#define DEVICE_ID_L 0xFE
#define DEVICE_ID_H 0xFF

static int _addr;                                        // Address of sensor

void HDC2010_init(uint8_t addr)
{
    _addr = addr;
}

float HDC2010_readTemp(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t byte[2];
    uint16_t temp;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = TEMP_LOW;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    byte[0] = buff[0];
    byte[1] = buff[1];

    temp = (unsigned int)byte[1] << 8 | byte[0];

    return (float)(temp)*165 / 65536 - 40;
}

float HDC2010_readHumidity(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t byte[2];
    uint16_t humidity;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = HUMID_LOW;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    byte[0] = buff[0];
    byte[1] = buff[1];

    humidity = (unsigned int)byte[1] << 8 | byte[0];

    return (float)(humidity) / (65536) * 100;
}

void HDC2010_enableHeater(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t configContents; //Stores current contents of config register

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    configContents = buff[0];

    //set bit 3 to 1 to enable heater
    configContents = (configContents | 0x08);

    buff[1] = configContents;
    buff[0] = CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

void HDC2010_disableHeater(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t configContents; //Stores current contents of config register

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    configContents = buff[0];

    //set bit 3 to 0 to disable heater (all other bits 1)
    configContents = (configContents & 0xF7);

    buff[1] = configContents;
    buff[0] = CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

void HDC2010_setLowTemp(i2c_rtos_handle_t *handle, float temp)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t temp_thresh_low;

    // Verify user is not trying to set value outside bounds
    if (temp < -40)
    {
        temp = -40;
    }
    else if (temp > 125)
    {
        temp = 125;
    }

    // Calculate value to load into register
    temp_thresh_low = (uint8_t)(256 * (temp + 40) / 165);

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = TEMP_THR_L;
    masterXfer.dataSize ++;
    buff[1] = temp_thresh_low;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

void HDC2010_setHighTemp(i2c_rtos_handle_t *handle, float temp)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t temp_thresh_high;

    // Verify user is not trying to set value outside bounds
    if (temp < -40)
    {
        temp = -40;
    }
    else if (temp > 125)
    {
        temp = 125;
    }

    // Calculate value to load into register
    temp_thresh_high = (uint8_t)(256 * (temp + 40) / 165);

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = TEMP_THR_H;
    masterXfer.dataSize ++;
    buff[1] = temp_thresh_high;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

void HDC2010_setHighHumidity(i2c_rtos_handle_t *handle, float humid)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t humid_thresh;

    // Verify user is not trying to set value outside bounds
    if (humid < 0)
    {
        humid = 0;
    }
    else if (humid > 100)
    {
        humid = 100;
    }

    // Calculate value to load into register
    humid_thresh = (uint8_t)(256 * (humid) / 100);

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = HUMID_THR_H;
    masterXfer.dataSize ++;
    buff[1] = humid_thresh;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

void HDC2010_setLowHumidity(i2c_rtos_handle_t *handle, float humid)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t humid_thresh;

    // Verify user is not trying to set value outside bounds
    if (humid < 0)
    {
        humid = 0;
    }
    else if (humid > 100)
    {
        humid = 100;
    }

    // Calculate value to load into register
    humid_thresh = (uint8_t)(256 * (humid) / 100);

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = HUMID_THR_L;
    masterXfer.dataSize ++;
    buff[1] = humid_thresh;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

//  Return humidity from the low threshold register
float HDC2010_readLowHumidityThreshold(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t regContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = HUMID_THR_L;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    regContents = buff[0];

    return (float)regContents * 100 / 256;
}

//  Return humidity from the high threshold register
float HDC2010_readHighHumidityThreshold(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t regContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = HUMID_THR_H;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    regContents = buff[0];

    return (float)regContents * 100 / 256;
}

//  Return temperature from the low threshold register
float HDC2010_readLowTempThreshold(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t regContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = TEMP_THR_L;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    regContents = buff[0];

    return (float)regContents * 165 / 256 - 40;
}

//  Return temperature from the high threshold register
float HDC2010_readHighTempThreshold(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t regContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = TEMP_THR_H;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    regContents = buff[0];

    return (float)regContents * 165 / 256 - 40;
}

/* Upper two bits of the MEASUREMENT_CONFIG register controls
   the temperature resolution*/
void HDC2010_setTempRes(i2c_rtos_handle_t *handle, int resolution)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t configContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = MEASUREMENT_CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    configContents = buff[0];

    switch (resolution)
    {
    case FOURTEEN_BIT:
        configContents = (configContents & 0x3F);
        break;

    case ELEVEN_BIT:
        configContents = (configContents & 0x7F);
        configContents = (configContents | 0x40);
        break;

    case NINE_BIT:
        configContents = (configContents & 0xBF);
        configContents = (configContents | 0x80);
        break;

    default:
        configContents = (configContents & 0x3F);
    }

    buff[1] = configContents;
    buff[0] = MEASUREMENT_CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}
/*  Bits 5 and 6 of the MEASUREMENT_CONFIG register controls
    the humidity resolution*/
void HDC2010_setHumidRes(i2c_rtos_handle_t *handle, int resolution)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t configContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = MEASUREMENT_CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    configContents = buff[0];

    switch (resolution)
    {
    case FOURTEEN_BIT:
        configContents = (configContents & 0xCF);
        break;

    case ELEVEN_BIT:
        configContents = (configContents & 0xDF);
        configContents = (configContents | 0x10);
        break;

    case NINE_BIT:
        configContents = (configContents & 0xEF);
        configContents = (configContents | 0x20);
        break;

    default:
        configContents = (configContents & 0xCF);
    }

    buff[1] = configContents;
    buff[0] = MEASUREMENT_CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

/*  Bits 2 and 1 of the MEASUREMENT_CONFIG register controls
    the measurement mode  */
void HDC2010_setMeasurementMode(i2c_rtos_handle_t *handle, int mode)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t configContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = MEASUREMENT_CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    configContents = buff[0];
    configContents = (configContents & 0xF9);
    configContents |= (mode & 0x03) << 1;

    buff[1] = configContents;
    buff[0] = MEASUREMENT_CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

/*  Bit 0 of the MEASUREMENT_CONFIG register can be used
    to trigger measurements  */
void HDC2010_triggerMeasurement(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t configContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = MEASUREMENT_CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    configContents = buff[0];

    configContents = (configContents | 0x01);

    buff[1] = configContents;
    buff[0] = MEASUREMENT_CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

/*  Bit 7 of the CONFIG register can be used to trigger a 
    soft reset  */
void HDC2010_reset(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    buff[1] = (buff[0] | 0x80);
    buff[0] = CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
}

/*  Bit 2 of the CONFIG register can be used to enable/disable 
    the interrupt pin  */
void HDC2010_enableInterrupt(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    buff[1] = (buff[0] | 0x04);
    buff[0] = CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

/*  Bit 2 of the CONFIG register can be used to enable/disable 
    the interrupt pin  */
void HDC2010_disableInterrupt(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    buff[1] = (buff[0] & 0xFB);
    buff[0] = CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

/*  Bits 6-4  of the CONFIG register controls the measurement 
    rate  */
void HDC2010_setRate(i2c_rtos_handle_t *handle, int rate)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t configContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    configContents = buff[0];

    configContents = (configContents & 0x8F);
    configContents |= (rate & 0x07) << 4;

    buff[1] = configContents;
    buff[0] = CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

/*  Bit 1 of the CONFIG register can be used to control the  
    the interrupt pins polarity */
void HDC2010_setInterruptPolarity(i2c_rtos_handle_t *handle, int polarity)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t configContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    configContents = buff[0];

    switch (polarity)
    {
    case ACTIVE_LOW:
        configContents = (configContents & 0xFD);
        break;

    case ACTIVE_HIGH:
        configContents = (configContents | 0x02);
        break;

    default:
        configContents = (configContents & 0xFD);
    }

    buff[1] = configContents;
    buff[0] = CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

/*  Bit 0 of the CONFIG register can be used to control the  
    the interrupt pin's mode */
void HDC2010_setInterruptMode(i2c_rtos_handle_t *handle, int mode)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t configContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    configContents = buff[0];

    switch (mode)
    {
    case LEVEL_MODE:
        configContents = (configContents & 0xFE);
        break;

    case COMPARATOR_MODE:
        configContents = (configContents | 0x01);
        break;

    default:
        configContents = (configContents & 0xFE);
    }

    buff[1] = configContents;
    buff[0] = CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

uint8_t HDC2010_readInterruptStatus(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t regContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = INTERRUPT_DRDY;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    regContents = buff[0];

    return regContents;
}

//  Clears the maximum temperature register
void HDC2010_clearMaxTemp(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;

    buff[1] = 0x00;
    buff[0] = TEMP_MAX;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

//  Clears the maximum humidity register
void HDC2010_clearMaxHumidity(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;

    buff[1] = 0x00;
    buff[0] = HUMID_MAX;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

//  Reads the maximum temperature register
float HDC2010_readMaxTemp(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t regContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = TEMP_MAX;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    regContents = buff[0];

    return (float)regContents * 165 / 256 - 40;
}

//  Reads the maximum humidity register
float HDC2010_readMaxHumidity(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t regContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = HUMID_MAX;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    regContents = buff[0];

    return (float)regContents / 256 * 100;
}

// Enables the interrupt pin for comfort zone operation
void HDC2010_enableThresholdInterrupt(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = INTERRUPT_CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    buff[1] = (buff[0] | 0x78);
    buff[0] = INTERRUPT_CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

// Disables the interrupt pin for comfort zone operation
void HDC2010_disableThresholdInterrupt(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = INTERRUPT_CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    buff[1] = (buff[0] & 0x87);
    buff[0] = INTERRUPT_CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

// enables the interrupt pin for DRDY operation
void HDC2010_enableDRDYInterrupt(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t regContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = INTERRUPT_CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    regContents = buff[0];

    regContents = (regContents | 0x80);

    buff[1] = regContents;
    buff[0] = INTERRUPT_CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}

// disables the interrupt pin for DRDY operation
void HDC2010_disableDRDYInterrupt(i2c_rtos_handle_t *handle)
{
    i2c_master_transfer_t masterXfer;
    uint8_t buff[4];
    status_t status;
    uint8_t regContents;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 0;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    buff[0] = INTERRUPT_CONFIG;
    masterXfer.dataSize ++;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }

    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during read transaction, %d\n", __func__, __LINE__, status);
    }

    regContents = buff[0];

    regContents = (regContents & 0x7F);

    buff[1] = regContents;
    buff[0] = INTERRUPT_CONFIG;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = _addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = buff;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(handle, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("%s() line=%d=>error during write transaction, %d\n", __func__, __LINE__, status);
    }
}
