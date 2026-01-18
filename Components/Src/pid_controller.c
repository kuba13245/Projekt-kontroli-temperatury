#include "pid_controller.h"

void PID_Init(PID_TypeDef *pid, float max_pwm_val) {
    pid->prevError = 0.0f;
    pid->integral = 0.0f;
    pid->maxOutput = max_pwm_val;

    // Zabezpieczenie członu całkującego (można dobrać eksperymentalnie)
    // Ustawiamy np. na 100% wartości wyjściowej
    pid->maxIntegral = max_pwm_val;
}

float PID_Calculate(PID_TypeDef *pid, float setpoint, float measurement, float Kp, float Ki, float Kd) {
    float error = setpoint - measurement;

    // --- Człon Proporcjonalny (P) ---
    float P = Kp * error;

    // --- Człon Całkujący (I) ---
    pid->integral += error;

    // Anti-Windup (Zabezpieczenie przed nasyceniem całki)
    if (pid->integral > pid->maxIntegral) {
        pid->integral = pid->maxIntegral;
    } else if (pid->integral < -pid->maxIntegral) {
        pid->integral = -pid->maxIntegral;
    }

    float I = Ki * pid->integral;

    // --- Człon Różniczkujący (D) ---
    float derivative = error - pid->prevError;
    float D = Kd * derivative;

    // Zapamiętanie błędu do następnego kroku
    pid->prevError = error;

    // --- Suma PID ---
    float output = P + I + D;

    // Ograniczenie wyjścia do zakresu PWM (0 - MAX)
    // Grzałka nie może grzać ujemnie, ani powyżej 100%
    if (output > pid->maxOutput) {
        output = pid->maxOutput;
    } else if (output < 0.0f) {
        output = 0.0f;
    }

    return output;
}
