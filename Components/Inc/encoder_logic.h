/*
 * encoder_logic.h
 *
 *  Created on: Jan 20, 2026
 *      Author: kalus
 */

#ifndef INC_ENCODER_LOGIC_H_
#define INC_ENCODER_LOGIC_H_

#include "main.h"
#include <stdbool.h>

typedef struct {
    // Konfiguracja
    TIM_HandleTypeDef *htim;
    GPIO_TypeDef *btnPort;
    uint16_t btnPin;
    float step;
    float minLimit;
    float maxLimit;

    // Stan systemu
    float set_temp;
    bool isEditMode;
    bool isHeatingEnabled; // <--- NOWE: Czy grzanie jest aktywne?

    // Wewnetrzne - Mechanika
    uint32_t lastCounter;
    uint32_t lastBtnTick;    // Czas ostatniego wciśnięcia
    bool buttonDown;         // Czy przycisk jest fizycznie wciśnięty
    uint8_t clickCount;      // <--- NOWE: Licznik kliknięć w krótkim czasie
} UI_StateTypeDef;

void EncoderInit(UI_StateTypeDef *ui, TIM_HandleTypeDef *htim, GPIO_TypeDef *btnPort, uint16_t btnPin, float defaultSetpoint, float step, float min, float max);
void EncoderUpdate(UI_StateTypeDef *ui);

#endif /* INC_ENCODER_LOGIC_H_ */
