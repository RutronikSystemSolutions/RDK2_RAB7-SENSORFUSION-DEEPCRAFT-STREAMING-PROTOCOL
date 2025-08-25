/*******************************************************************************
* File Name:   main.c
*
* Description:	This project implements the "protocol version 2"
* 				enabling to collect data from the RAB7 SensorFusion
* 				using Deepcraft Studio
*
* Related Document: See README.md
*
*  Created on: 2025-08-25
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Industriestraße 2, 75228 Ispringen, Germany
*  Author: ROJ030
*
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "tensor_streaming_protocol/protocol.h"
#include "board.h"
#include "usbd.h"
#include "clock.h"
#include "devices/protocol_devices.h"

int main(void)
{
    // Initialize the device and board peripherals
    cy_rslt_t result = cybsp_init();

    // Board init failed. Stop program execution
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    // Clock initialization: enable to monitor the time
	if(!clock_init())
	{
		CY_ASSERT(0);
	}

    // Enable global interrupts
    __enable_irq();

    // Initialize LEDs
    result = cyhal_gpio_init( LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    //Enable debug output via KitProg UART
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    printf("\x1b[2J\x1b[;H"); // Reset screen
    printf("RDK2 RAB7 Sensor Fusion - Deepcraft Streaming protocol - V1.0\r\n");

    protocol_Version firmware_version =
    {
    	.major = 1,
        .minor = 2,
        .build = 0,
        .revision = 0
    };

    // Serial UUID
    uint8_t* serial = board_get_serial_uuid();
    printf("Serial UUID:\r\n");
    for(int i = 0; i < 16; ++i)
    {
    	printf("0x%x:", serial[i]);
    }
    printf("\r\n");


    // Create a protocol instance
    protocol_t* protocol = protocol_create("PSOC 62 (RDK2-RAB7)", serial, firmware_version);

    // Add reset function
    protocol->board_reset = board_reset;

    int init_retval = protocol_devices_init(protocol);
    if ( init_retval != 0)
    {
    	printf("Cannot initialize devices: %d \r\n", init_retval);
    	for(;;){}
    }

    // Initialize the streaming interface
    usbd_t* usb = usbd_create(protocol);

    // Process events
    for (;;)
    {
        int proto_retval = protocol_process_request(protocol, &usb->istream, &usb->ostream);
        if (proto_retval != PROTOCOL_STATUS_SUCCESS)
        {
        	printf("Something went wrong: %d \r\n", proto_retval);
        }
    }

}
/* [] END OF FILE */
