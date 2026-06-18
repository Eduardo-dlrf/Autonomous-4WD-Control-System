/*
 * Directions.c
 *
 *  Created on: Nov 5, 2023
 *      Author: Jorge
 */

#include "main.h"

void Forward(void){
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); //izq atras
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1); //izq atras
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1); //derehca atras
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0); //derecha atrasa
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0); //derecha enfrente
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1); //derecha enfrente
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); //izq enfrente
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0); // ezq enfrente
}

void RightTurn(void){
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
}

void Stop(void){
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);


}
