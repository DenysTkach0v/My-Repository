#include "dht11.h"

#define DHT11_START_SIGNAL_LENGTH 18
#define DHT11_RESPONSE_TIME_LENGTH 40

static void DHT11_Send_Start_Signal(DHT11_HandleTypeDef* dht11) {
    HAL_GPIO_WritePin(dht11->port, dht11->pin, GPIO_PIN_RESET);
    HAL_Delay(18); // Delay for 18 ms
    HAL_GPIO_WritePin(dht11->port, dht11->pin, GPIO_PIN_SET);
    HAL_Delay(20); // Delay for 20 ms
}

static uint8_t DHT11_Check_Response(DHT11_HandleTypeDef* dht11) {
    HAL_GPIO_ReadPin(dht11->port, dht11->pin); // Wait for response from DHT11
    HAL_Delay(30);
    if (HAL_GPIO_ReadPin(dht11->port, dht11->pin) == GPIO_PIN_RESET) {
        HAL_Delay(80);
        return (HAL_GPIO_ReadPin(dht11->port, dht11->pin) == GPIO_PIN_SET) ? 1 : 0; // Return response status
    }
    return 0;
}

static uint8_t DHT11_Read_Bit(DHT11_HandleTypeDef* dht11) {
    while (HAL_GPIO_ReadPin(dht11->port, dht11->pin) == GPIO_PIN_RESET); // Wait for high level
    HAL_Delay(30); // Measure delay
    uint8_t bit = HAL_GPIO_ReadPin(dht11->port, dht11->pin) == GPIO_PIN_SET ? 1 : 0; // Read bit
    while (HAL_GPIO_ReadPin(dht11->port, dht11->pin) == GPIO_PIN_SET); // Wait for low level
    return bit;
}

HAL_StatusTypeDef DHT11_Init(DHT11_HandleTypeDef* dht11, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    dht11->port = GPIOx;
    dht11->pin = GPIO_Pin;
    return HAL_OK;
}

HAL_StatusTypeDef DHT11_Read(DHT11_HandleTypeDef* dht11, float* temperature, float* humidity) {
    uint8_t data[5] = {0};

    DHT11_Send_Start_Signal(dht11);

    if (DHT11_Check_Response(dht11) == 0) {
        return HAL_ERROR; // Failed to get response
    }

    for (int i = 0; i < 5; i++) {
        data[i] = 0;
        for (int j = 0; j < 8; j++) {
            data[i] <<= 1; // Shift left
            data[i] |= DHT11_Read_Bit(dht11); // Read bit
        }
    }

    // Check checksum
    if ((data[0] + data[1] + data[2] + data[3]) != data[4]) {
        return HAL_ERROR; // Checksum does not match
    }

    *humidity = data[0]; // Humidity
    *temperature = data[2]; // Temperature

    return HAL_OK;
}
