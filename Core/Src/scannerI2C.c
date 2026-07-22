/*
 * scannerI2C.c
 *
 *  Created on: Jul 17, 2026
 *      Author: andres-e-ibarra
 */
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <scannerI2C.h>

extern I2C_HandleTypeDef hi2c1;   // tu handle ya inicializado
extern UART_HandleTypeDef huart2; // si usas printf por UART para ver resultados

void I2C_Scan(I2C_HandleTypeDef *hi2c) {
    char msg[64];
    uint8_t found = 0;

    printf("Escaneando bus I2C...\r\n");

    for (uint8_t addr = 1; addr < 128; addr++) {
        // HAL_I2C_IsDeviceReady intenta un START + dirección + STOP
        if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 2, 10) == HAL_OK) {
            sprintf(msg, "Dispositivo encontrado en: 0x%02X\r\n", addr);
            printf("%s", msg);
            found++;
        }
    }

    if (found == 0) {
        printf("No se encontraron dispositivos. Revisa cableado/pull-ups.\r\n");
    } else {
        sprintf(msg, "Total dispositivos encontrados: %d\r\n", found);
        printf("%s", msg);
    }
}
