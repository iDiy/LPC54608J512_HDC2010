/*
 * hdc2010_test.c
 *
 *  Created on: 2019年10月23日
 *      Author: alan
 */

/*  Standard C Included Files */
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

#include "fsl_pint.h"
#include "pin_mux.h"
#include "fsl_inputmux.h"

#include "hdc2010.h"

#if (HDC2010_TEST)
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define EXAMPLE_I2C_MASTER_IRQ FLEXCOMM1_IRQn

#define EXAMPLE_I2C_MASTER_BASE (I2C1_BASE)

#define I2C_MASTER_CLOCK_FREQUENCY (12000000)

#define EXAMPLE_I2C_MASTER ((I2C_Type *)EXAMPLE_I2C_MASTER_BASE)

#define I2C_MASTER_SLAVE_ADDR_7BIT (0x41U)
#define I2C_BAUDRATE (100000) /* 100K */
#define I2C_DATA_LENGTH (32)  /* MAX is 256 */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t g_master_buff[I2C_DATA_LENGTH];

i2c_master_handle_t *g_m_handle;

SemaphoreHandle_t i2c_sem;

SemaphoreHandle_t hdc2010_int_sem;    /*!< A semaphore to notify and unblock task when the transfer ends */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Task priorities. */
#define master_task_PRIORITY (configMAX_PRIORITIES - 2)

#define I2C_NVIC_PRIO 2
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void master_task(void *pvParameters);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Call back for PINT Pin interrupt 0-7.
 */
void pint_intr_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    PRINTF("\f\r\nPINT Pin Interrupt %d event detected.\n", pintr);

    if (pintr == kPINT_PinInt0)
    {
        BaseType_t reschedule;
        xSemaphoreGiveFromISR(hdc2010_int_sem, &reschedule);
        portYIELD_FROM_ISR(reschedule);
    }
}

void HDC2010_test(void)
{
    PRINTF("HDC2010_test start\n");
    if (xTaskCreate(master_task, "Master_task", configMINIMAL_STACK_SIZE + 134, NULL, master_task_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("Failed to create master task");
        while (1)
            ;
    }
    vTaskStartScheduler();
}
static void master_task(void *pvParameters)
{
    i2c_rtos_handle_t master_rtos_handle;
    i2c_master_config_t masterConfig;
    // i2c_master_transfer_t masterXfer;
    uint32_t sourceClock;
    status_t status;

    PRINTF("master task start\n");

    NVIC_SetPriority(EXAMPLE_I2C_MASTER_IRQ, I2C_NVIC_PRIO + 1);
    EnableIRQ(EXAMPLE_I2C_MASTER_IRQ);

    /*
     * masterConfig.baudRate_Bps = 100000U;
     * masterConfig.enableStopHold = false;
     * masterConfig.glitchFilterWidth = 0U;
     * masterConfig.enableMaster = true;
     */
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;
    sourceClock               = I2C_MASTER_CLOCK_FREQUENCY;

    status = I2C_RTOS_Init(&master_rtos_handle, EXAMPLE_I2C_MASTER, &masterConfig, sourceClock);
    if (status != kStatus_Success)
    {
        PRINTF("I2C master: error during init, %d", status);
    }
    PRINTF("I2C master init OK");
    PRINTF("\r\n");

    g_m_handle = &master_rtos_handle.drv_handle;

    HDC2010_init(I2C_MASTER_SLAVE_ADDR_7BIT);
    HDC2010_reset(&master_rtos_handle);
    HDC2010_setHighTemp(&master_rtos_handle, 28);
    HDC2010_setLowTemp(&master_rtos_handle, 22);
    HDC2010_setHighHumidity(&master_rtos_handle, 55);
    HDC2010_setLowHumidity(&master_rtos_handle, 44);
    HDC2010_enableInterrupt(&master_rtos_handle);
    // HDC2010_enableThresholdInterrupt(&master_rtos_handle);
    HDC2010_enableDRDYInterrupt(&master_rtos_handle);
    HDC2010_setInterruptPolarity(&master_rtos_handle, ACTIVE_HIGH);
    HDC2010_setInterruptMode(&master_rtos_handle, LEVEL_MODE);
    HDC2010_setMeasurementMode(&master_rtos_handle, TEMP_AND_HUMID);
    HDC2010_setRate(&master_rtos_handle, ONE_HZ);
    HDC2010_setTempRes(&master_rtos_handle, FOURTEEN_BIT);
    HDC2010_setHumidRes(&master_rtos_handle, FOURTEEN_BIT);
    HDC2010_triggerMeasurement(&master_rtos_handle);

    hdc2010_int_sem = xSemaphoreCreateBinary();
    if (hdc2010_int_sem == NULL)
    {
        PRINTF("%s() line=%d=>create hdc2010_int_sem failed!!!\n", __func__, __LINE__);
    }

    NVIC_SetPriority(PIN_INT0_IRQn, I2C_NVIC_PRIO + 1);
    /* Connect trigger sources to PINT */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt0, HDC2010_PINT_PIN_INT0_SRC);
    /* Turnoff clock to inputmux to save power. Clock is only needed to make changes */
    INPUTMUX_Deinit(INPUTMUX);

    /* Initialize PINT */
    PINT_Init(PINT);

    /* Setup Pin Interrupt 0 for rising edge */
    PINT_PinInterruptConfig(PINT, kPINT_PinInt0, kPINT_PinIntEnableRiseEdge, pint_intr_callback);
    /* Enable callbacks for PINT0 by Index */
    PINT_EnableCallbackByIndex(PINT, kPINT_PinInt0);

    while (true)
    {
        (void)xSemaphoreTake(hdc2010_int_sem, portMAX_DELAY);
        float t = HDC2010_readTemp(&master_rtos_handle);
        float h = HDC2010_readHumidity(&master_rtos_handle);
        PRINTF("%s() line=%d=>t=%.1f h=%.1f\n", __func__, __LINE__, t, h);
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    vTaskSuspend(NULL);
}

#endif