/*
 * reciever.h
 *
 *  Created on: Jul 4, 2023
 *      Author: mfact
 */

#pragma once

#include "main.h"

void Reciever_Init(SPI_HandleTypeDef *hspi);
void Reciever_Handler();

/* defined in main.h function to handle byte recieved */
void Reciever_Callback(uint8_t data);
