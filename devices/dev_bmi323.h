/*
 * dev_bmi323.h
 *
 *  Created on: Aug 22, 2025
 *      Author: ROJ030
 */

#ifndef DEVICES_DEV_BMI323_H_
#define DEVICES_DEV_BMI323_H_

#include <stdbool.h>
#include <cyhal_hw_types.h>

#include "tensor_streaming_protocol/protocol.h"

int dev_bmi323_register(protocol_t* protocol, cyhal_i2c_t* i2c);

#endif /* DEVICES_DEV_BMI323_H_ */
