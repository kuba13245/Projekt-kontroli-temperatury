/*
 * heater.c
 *
 *  Created on: Jan 18, 2026
 *      Author: kalus
 */


#include "heater.h"

// Inicjalizacja PWM dla grzałki
void Heater_Init(Heater_TypeDef *heater, TIM_HandleTypeDef *tim, uint32_t channel, uint32_t max_pwm_val) {
    heater->tim = tim;
    heater->channel = channel;
    heater->max_pwm = max_pwm_val;

    // Startujemy PWM na TIM2 CH1
    HAL_TIM_PWM_Start(heater->tim, heater->channel);
}

// Ustawienie mocy grzania na podstawie wyjścia z PID
void Heater_SetPower(Heater_TypeDef *heater, float pid_output) {
    int32_t pwm_duty = 0;

    // 1. Obsługa wartości ujemnych (PID chce chłodzić -> grzałka OFF)
    if (pid_output < 0.0f) {
        pwm_duty = 0;
    }
    // 2. Obsługa nasycenia (PID chce grzać mocniej niż 100%)
    else if (pid_output > (float)heater->max_pwm) {
        pwm_duty = heater->max_pwm;
    }
    // 3. Normalna praca
    else {
        pwm_duty = (int32_t)pid_output;
    }

    // Wpisanie wartości do rejestru CCR1 (Capture Compare Register 1) dla TIM2
    // To makro jest szybsze i bezpieczniejsze niż bezpośrednie pisanie do rejestrów
    __HAL_TIM_SET_COMPARE(heater->tim, heater->channel, pwm_duty);
}
