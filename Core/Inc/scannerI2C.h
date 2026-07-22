/*
 * scannerI2C.h
 *
 *  Created on: Jul 17, 2026
 *      Author: andres-e-ibarra
 */

#ifndef INC_SCANNERI2C_H_
#define INC_SCANNERI2C_H_

extern I2C_HandleTypeDef hi2c1;   // tu handle ya inicializado
extern UART_HandleTypeDef huart2; // si usas printf por UART para ver resultados

void I2C_Scan(I2C_HandleTypeDef *hi2c);

#endif /* INC_SCANNERI2C_H_ */
