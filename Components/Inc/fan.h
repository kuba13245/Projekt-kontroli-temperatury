#ifndef FAN_H_
#define FAN_H_

#include "main.h"

typedef struct {
    TIM_HandleTypeDef *tim;
    uint32_t channel;
    uint32_t max_pwm;
} Fan_TypeDef;

void Fan_Init(Fan_TypeDef *fan, TIM_HandleTypeDef *tim, uint32_t channel, uint32_t max_pwm_val);
void Fan_SetPower(Fan_TypeDef *fan, float pwm_val);

#endif
