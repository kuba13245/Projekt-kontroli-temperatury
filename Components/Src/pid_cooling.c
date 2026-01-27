#include "pid_cooling.h"

void PID_Cooling_Init(PID_Cooling_TypeDef *pid, float max_pwm_val) {
    pid->prevError = 0.0f;
    pid->integral = 0.0f;
    pid->maxOutput = max_pwm_val;

    // Limit całki - zazwyczaj równy max sterowaniu
    pid->maxIntegral = max_pwm_val;
}

float PID_Cooling_Calculate(PID_Cooling_TypeDef *pid, float setpoint, float measurement, float Kp, float Ki, float Kd) {
    // --- KLUCZOWA RÓŻNICA: Logika Chłodzenia ---
    // Error jest dodatni, gdy temperatura (measurement) jest wyższa niż cel (setpoint).
    // Dzięki temu dodatni wynik PID włączy wentylator.
    float error = measurement - setpoint;

    // --- Człon Proporcjonalny (P) ---
    float P = Kp * error;

    // --- Człon Całkujący (I) ---
    pid->integral += error;

    // Anti-Windup
    if (pid->integral > pid->maxIntegral) {
        pid->integral = pid->maxIntegral;
    } else if (pid->integral < -pid->maxIntegral) {
        pid->integral = -pid->maxIntegral;
    }

    float I = Ki * pid->integral;

    // --- Człon Różniczkujący (D) ---
    float derivative = error - pid->prevError;
    float D = Kd * derivative;

    // Zapamiętanie błędu
    pid->prevError = error;

    // --- Suma PID ---
    float output = P + I + D;

    // Ograniczenie wyjścia (0 - MAX)
    // Wentylator nie może kręcić się ujemnie
    if (output > pid->maxOutput) {
        output = pid->maxOutput;
    } else if (output < 1000.0f) {
        output = 1000.0f;
    }

    return output;
}
