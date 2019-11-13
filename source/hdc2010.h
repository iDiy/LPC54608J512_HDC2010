/*
 * hdc2010.h
 *
 *  Created on: 2019年10月22日
 *      Author: alan
 */

#ifndef HDC2010_H_
#define HDC2010_H_
#include "fsl_common.h"

//  Constants for setting measurement resolution
#define FOURTEEN_BIT 0
#define ELEVEN_BIT 1
#define NINE_BIT  2

//  Constants for setting sensor mode
#define TEMP_AND_HUMID 0
#define TEMP_ONLY	   1
#define HUMID_ONLY	   2
#define ACTIVE_LOW	   0
#define ACTIVE_HIGH	   1
#define LEVEL_MODE		0
#define COMPARATOR_MODE 1

//  Constants for setting sample rate
#define MANUAL			0
#define TWO_MINS		1
#define ONE_MINS		2
#define TEN_SECONDS		3 
#define	FIVE_SECONDS	4
#define ONE_HZ			5
#define TWO_HZ			6
#define FIVE_HZ			7	

void HDC2010_init(uint8_t addr);					// Initialize the HDC2010
float HDC2010_readTemp(i2c_rtos_handle_t *handle);					// Returns the temperature in degrees C
float HDC2010_readHumidity(i2c_rtos_handle_t *handle);				// Returns the relative humidity
void HDC2010_enableHeater(i2c_rtos_handle_t *handle);				// Enables the heating element
void HDC2010_disableHeater(i2c_rtos_handle_t *handle);				// Disables the heating element
void HDC2010_setLowTemp(i2c_rtos_handle_t *handle, float temp);			// Sets low threshold temperature (in c)
void HDC2010_setHighTemp(i2c_rtos_handle_t *handle, float temp);			// Sets high threshold temperature (in c)
void HDC2010_setHighHumidity(i2c_rtos_handle_t *handle, float humid);		// Sets high Humiditiy threshold
void HDC2010_setLowHumidity(i2c_rtos_handle_t *handle, float humid);		// Sets low Humidity threshold 
float HDC2010_readLowHumidityThreshold(i2c_rtos_handle_t *handle);	// Returns contents of low humidity threshold register
float HDC2010_readHighHumidityThreshold(i2c_rtos_handle_t *handle);	// Returns contents of high humidity threshold register
float HDC2010_readLowTempThreshold(i2c_rtos_handle_t *handle);		// Returns contents of low temperature threshold register (in C)
float HDC2010_readHighTempThreshold(i2c_rtos_handle_t *handle);		// Returns contents of high temperature threshold register (in C)
void HDC2010_triggerMeasurement(i2c_rtos_handle_t *handle);			// Triggers a manual temperature/humidity reading
void HDC2010_reset(i2c_rtos_handle_t *handle); 						// Triggers a software reset
void HDC2010_enableInterrupt(i2c_rtos_handle_t *handle);				// Enables the interrupt/DRDY pin
void HDC2010_disableInterrupt(i2c_rtos_handle_t *handle); 			// Disables the interrupt/DRDY pin (High Z)
uint8_t HDC2010_readInterruptStatus(i2c_rtos_handle_t *handle); 		// Reads the status of the interrupt register
void HDC2010_clearMaxTemp(i2c_rtos_handle_t *handle);				// Clears the Maximum temperature register
void HDC2010_clearMaxHumidity(i2c_rtos_handle_t *handle);			// Clears the Maximum humidity register
float HDC2010_readMaxTemp(i2c_rtos_handle_t *handle); 				// Reads the maximum temperature register
float HDC2010_readMaxHumidity(i2c_rtos_handle_t *handle);			// Reads the maximum humidity register
void HDC2010_enableThresholdInterrupt(i2c_rtos_handle_t *handle);	// Enables high and low temperature/humidity interrupts
void HDC2010_disableThresholdInterrupt(i2c_rtos_handle_t *handle);	// Disables high and low temperature/humidity interrupts
void HDC2010_enableDRDYInterrupt(i2c_rtos_handle_t *handle);			// Enables data ready interrupt
void HDC2010_disableDRDYInterrupt(i2c_rtos_handle_t *handle);		// Disables data ready interrupt



/* Sets Temperature & Humidity Resolution, 3 options
    0 - 14 bit
    1 - 11 bit
    2 - 9 bit
    default - 14 bit							*/
void HDC2010_setTempRes(i2c_rtos_handle_t *handle, int resolution);		
void HDC2010_setHumidRes(i2c_rtos_handle_t *handle, int resolution);	

/* Sets measurement mode, 3 options
    0 - Temperature and Humidity
    1 - Temperature only
    2 - Humidity only
    default - Temperature & Humidity			*/
void HDC2010_setMeasurementMode(i2c_rtos_handle_t *handle, int mode);

/* Sets reading rate, 8 options
    0 - Manual
    1 - reading every 2 minutes
    2 - reading every minute
    3 - reading every ten seconds
    4 - reading every 5 seconds
    5 - reading every second
    6 - reading at 2Hz
    7 - reading at 5Hz
    default - Manual		*/		
void HDC2010_setRate(i2c_rtos_handle_t *handle, int rate);

/* Sets Interrupt polarity, 2 options
    0 - Active Low
    1 - Active High
    default - Active Low			*/		
void HDC2010_setInterruptPolarity(i2c_rtos_handle_t *handle, int polarity);

/* Sets Interrupt mode, 2 options
    0 - Level sensitive
    1 - Comparator mode
    default - Level sensitive	*/		
void HDC2010_setInterruptMode(i2c_rtos_handle_t *handle, int mode);

#endif /* HDC2010_H_ */
