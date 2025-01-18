/*
 * can_fd.cpp
 *
 *  Created on: Dec 24, 2024
 *      Author: WINDOWS 11
 */

#include <can_fd.h>

can_fd::can_fd(int can_id)
{
	motor_id = can_id;
}

void can_fd::Config()
{
	// Configure the filter to accept all messages (optional, adjust as needed)
	FDCAN_FilterTypeDef filterConfig;
    filterConfig.IdType = FDCAN_STANDARD_ID;       // Standard Identifier (11 bits)
    filterConfig.FilterIndex = 0;
    filterConfig.FilterType = FDCAN_FILTER_MASK;
    filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filterConfig.FilterID1 = 0x1;               // Accept all IDs
    filterConfig.FilterID2 = 0x7ff;               // Mask for all bits

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    // Start the FDCAN module
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    // Activate the notification for RX FIFO 0
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}


