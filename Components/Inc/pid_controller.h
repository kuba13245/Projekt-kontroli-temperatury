#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

// Struktura przechowywująca stan regulatora (pamięć błędów)
typedef struct {
    float prevError;      // Poprzedni błąd (dla członu D)
    float integral;       // Suma błędów (dla członu I)
    float maxOutput;      // Maksymalna wartość wyjścia (np. dla PWM)
    float maxIntegral;    // Limit dla anty-windup
} PID_TypeDef;

// Inicjalizacja: Ustawiamy limity i zerujemy pamięć
// Kp, Ki, Kd nie są tu potrzebne, bo przekazujesz je w pętli main
void PID_Init(PID_TypeDef *pid, float max_pwm_val);

// Obliczenie PID
// Funkcja przyjmuje Kp, Ki, Kd jako argumenty, zgodnie z Twoim main.c
float PID_Calculate(PID_TypeDef *pid, float setpoint, float measurement, float Kp, float Ki, float Kd);

#endif // PID_CONTROLLER_H
