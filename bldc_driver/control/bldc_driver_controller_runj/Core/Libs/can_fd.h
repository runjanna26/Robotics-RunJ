/*
 * can_fd.h
 *
 *  Created on: Dec 24, 2024
 *      Author: WINDOWS 11
 */


#ifndef CAN_FD_H_
#define CAN_FD_H_

#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;


#define RX_FIFO0_NEW_MESSAGE 1


class can_fd {
public:
	can_fd(int can_id);
	void Config(void);
//	void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

	uint8_t Data[8]; 		// Buffer to store received data
	uint32_t motor_id;

	bool test;

};



#endif /* CAN_FD_H_ */
