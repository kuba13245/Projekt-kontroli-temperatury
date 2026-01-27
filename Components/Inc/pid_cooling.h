#ifndef PID_COOLING_H
#define PID_COOLING_H

#include <stdint.h>

// Struktura regulatora chłodzenia (może być identyczna jak grzania,
// ale inna nazwa pozwala uniknąć pomyłek w kodzie)
typedef struct {
    float prevError;      // Poprzedni błąd
    float integral;       // Suma błędów
    float maxOutput;      // Max PWM wentylatora
    float maxIntegral;    // Limit całki
} PID_Cooling_TypeDef;

// Inicjalizacja
void PID_Cooling_Init(PID_Cooling_TypeDef *pid, float max_pwm_val);

// Obliczenie PID dla chłodzenia
// UWAGA: Logika w środku jest odwrócona względem grzania
float PID_Cooling_Calculate(PID_Cooling_TypeDef *pid, float setpoint, float measurement, float Kp, float Ki, float Kd);

#endif // PID_COOLING_H
