/*
 * board.c
 *
 *  Created on: Aug 21, 2025
 *      Author: ROJ030
 */

#include "board.h"
#include <string.h>
#include <cyhal.h>
#include <cybsp.h>

uint8_t* board_get_serial_uuid(void)
{
    /* Create serial UUID {290DE5CB-460B-41BF-XXXX-XXXXXXXXXXXX}. */
    /* Last part is silicon unique ID. */
    uint64_t serial64 = Cy_SysLib_GetUniqueId();
    static uint8_t serial[16] = {
            0x29, 0x0d, 0xe5, 0xcb, 0x46, 0x0b, 0x41, 0xbf,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    memcpy(serial + 8, &serial64, 8);

    return serial;
}

void board_reset(protocol_t* protocol)
{
    UNUSED(protocol);
    NVIC_SystemReset();
}
