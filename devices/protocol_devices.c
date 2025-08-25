/*
 * protocol_devices.c
 *
 *  Created on: Aug 25, 2025
 *      Author: ROJ030
 */

#include "protocol_devices.h"

#include <cyhal.h>
#include <cybsp.h>

#include "dev_bmm350.h"
#include "dev_bmi323.h"

static cyhal_i2c_t i2c;
static cyhal_i2c_cfg_t i2c_cfg =
{
		.is_slave = false,
		.address = 0,
		.frequencyhal_hz = 400000UL,
};

int protocol_devices_init(protocol_t* protocol)
{
	// Force I2C to stop in case we have a sensor hanging
	cy_rslt_t result = cyhal_gpio_init(ARDU_SDA, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
	if (result != CY_RSLT_SUCCESS) return -1;

	result = cyhal_gpio_init(ARDU_SCL, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
	if (result != CY_RSLT_SUCCESS) return -2;

	cyhal_gpio_write(ARDU_SDA,false);
	CyDelay(1);
	cyhal_gpio_write(ARDU_SCL,false);
	CyDelay(1);
	cyhal_gpio_free(ARDU_SDA);
	cyhal_gpio_free(ARDU_SCL);

	// Initialize I2C Master
	result = cyhal_i2c_init(&i2c, ARDU_SDA, ARDU_SCL, NULL);
	if (result != CY_RSLT_SUCCESS) return -3;

	result = cyhal_i2c_configure(&i2c, &i2c_cfg);
	if (result != CY_RSLT_SUCCESS) return -4;

	int dev_retval = dev_bmm350_register(protocol, &i2c);
	if(dev_retval != 0) return -5;

	dev_retval = dev_bmi323_register(protocol, &i2c);
	if(dev_retval != 0) return -6;

	return 0;
}


