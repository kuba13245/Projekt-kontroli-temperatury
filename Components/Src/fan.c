#include "fan.h"

void Fan_Init(Fan_TypeDef *fan, TIM_HandleTypeDef *tim, uint32_t channel, uint32_t max_pwm_val) {
    fan->tim = tim;
    fan->channel = channel;
    fan->max_pwm = max_pwm_val;
    HAL_TIM_PWM_Start(fan->tim, fan->channel);
}

void Fan_SetPower(Fan_TypeDef *fan, float pwm_val) {
    int32_t pwm_duty = (int32_t)pwm_val;

    // Zabezpieczenia
    if (pwm_duty < 0) pwm_duty = 0;
    if (pwm_duty > fan->max_pwm) pwm_duty = fan->max_pwm;

    __HAL_TIM_SET_COMPARE(fan->tim, fan->channel, pwm_duty);
}
