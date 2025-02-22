#ifndef __DHT11_H
#define __DHT11_H

#include "stm32f1xx_hal.h"

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} DHT11_HandleTypeDef;

HAL_StatusTypeDef DHT11_Init(DHT11_HandleTypeDef* dht11, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef DHT11_Read(DHT11_HandleTypeDef* dht11, float* temperature, float* humidity);


#endif // __DHT11_H
