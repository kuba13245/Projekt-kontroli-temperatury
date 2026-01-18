/*
 * heater.h
 *
 *  Created on: Jan 18, 2026
 *      Author: kalus
 */

#ifndef INC_HEATER_H_
#define INC_HEATER_H_

#ifndef HEATER_H_
#define HEATER_H_

#include "main.h" // Zawiera definicje HAL i stm32fxxx.h

// Struktura reprezentująca naszą grzałkę
typedef struct {
    TIM_HandleTypeDef *tim;  // Wskaźnik do timera (tu będzie &htim2)
    uint32_t channel;        // Kanał (tu będzie TIM_CHANNEL_1)
    uint32_t max_pwm;        // Maksymalna wartość ARR (np. 1000)
} Heater_TypeDef;

// Funkcje
void Heater_Init(Heater_TypeDef *heater, TIM_HandleTypeDef *tim, uint32_t channel, uint32_t max_pwm_val);
void Heater_SetPower(Heater_TypeDef *heater, float pid_output);

#endif



#endif /* INC_HEATER_H_ */
