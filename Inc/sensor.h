/*
 * sensor.h
 *
 *  Created on: 9 Nov 2017
 *      Author: tinova
 */
#include "stm32f4xx_hal.h"
#include "main.h"
#ifndef SENSOR_H_
#define SENSOR_H_

uint8_t SSBtn(void);
uint8_t RstBtn(void);
uint8_t Ssr1(void);
uint8_t Ssr2(void);
uint8_t Ssr3(void);

#endif /* SENSOR_H_ */
