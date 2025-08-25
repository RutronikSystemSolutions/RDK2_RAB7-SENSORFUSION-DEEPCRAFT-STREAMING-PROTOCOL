/*
 * board.h
 *
 *  Created on: Aug 21, 2025
 *      Author: ROJ030
 */

#ifndef BOARD_H_
#define BOARD_H_

#include <stdint.h>
#include "tensor_streaming_protocol/protocol.h"

uint8_t* board_get_serial_uuid(void);

void board_reset(protocol_t* protocol);

#endif /* BOARD_H_ */
