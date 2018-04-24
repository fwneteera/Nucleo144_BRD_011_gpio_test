/*
 * nucleoPinIN.h
 *
 * defines input pins comming from NET-BRD-011
 *
 *  Created on: Apr 15, 2018
 *      Author: Neteera
 */

#ifndef NUCLEOPININ_H_
#define NUCLEOPININ_H_

#define NET_BRD_011_ADC_OUT_BITS_NUM (12)

typedef struct NucleoGPIO
{
	GPIO_TypeDef* port;
	uint16_t pin;
}tNucleoGPIO;

typedef struct RADAR_ADC_GPIOin
{
	const tNucleoGPIO* gpio;
}tRADAR_ADC_GPIOin;

#endif /* NUCLEOPININ_H_ */
