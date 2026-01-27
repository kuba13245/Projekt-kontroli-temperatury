#include "encoder_logic.h"

// Czas oczekiwania na drugie kliknięcie (w ms)
#define DOUBLE_CLICK_TIME 400

void EncoderInit(UI_StateTypeDef *ui, TIM_HandleTypeDef *htim, GPIO_TypeDef *btnPort, uint16_t btnPin, float defaultSetpoint, float step, float min, float max) {
    ui->htim = htim;
    ui->btnPort = btnPort;
    ui->btnPin = btnPin;
    ui->set_temp = defaultSetpoint;
    ui->step = step;
    ui->minLimit = min;
    ui->maxLimit = max;

    ui->isEditMode = true;
    ui->isHeatingEnabled = false; // Domyślnie grzanie wyłączone

    ui->lastBtnTick = 0;
    ui->buttonDown = false;
    ui->clickCount = 0;           // Reset licznika kliknięć
    ui->lastCounter = ui->htim->Instance->CNT;

    HAL_TIM_Encoder_Start(ui->htim, TIM_CHANNEL_ALL);
}

void EncoderUpdate(UI_StateTypeDef *ui) {
    // --- 1. WYKRYWANIE KLIKNIĘĆ (Bez zmian) ---
    bool isPressedNow = (HAL_GPIO_ReadPin(ui->btnPort, ui->btnPin) == GPIO_PIN_RESET);
    uint32_t currentTick = HAL_GetTick();

    if (isPressedNow) {
        if (!ui->buttonDown) {
            if (currentTick - ui->lastBtnTick > 50) {
                ui->clickCount++;
                ui->lastBtnTick = currentTick;
                ui->buttonDown = true;
            }
        }
    } else {
        ui->buttonDown = false;
    }

    // --- 2. LOGIKA DECYZYJNA (Bez zmian) ---
    if (ui->clickCount > 0 && (currentTick - ui->lastBtnTick > DOUBLE_CLICK_TIME)) {
        if (ui->clickCount == 1) {
            ui->isEditMode = !ui->isEditMode;
        }
        else if (ui->clickCount >= 2) {
            ui->isHeatingEnabled = !ui->isHeatingEnabled;
            if (!ui->isHeatingEnabled) {
                ui->isEditMode = false;
            }
        }
        ui->clickCount = 0;
    }

    // --- 3. OBSŁUGA POKRĘTŁA (POPRAWIONA) ---

    // 1. Pobieramy SUROWY licznik (nie dzielimy go jeszcze!)
    uint32_t rawCounter = ui->htim->Instance->CNT;

    // 2. Obliczamy różnicę na surowych danych.
    // Dzięki rzutowaniu na int16_t, przejście z 0 na 65535 da poprawny wynik -1.
    int16_t rawDiff = (int16_t)(rawCounter - ui->lastCounter);

    // 3. Sprawdzamy, czy różnica jest wystarczająca (4 impulsy = 1 klik)
    // Dzielenie całkowite (rawDiff / 4) zwróci 0, jeśli przesunięcie jest mniejsze niż 4 impulsy
    int16_t steps = rawDiff / 4;

    if (steps != 0) {
        // Jeśli wykonano ruch (steps nie jest zerem)
        if (ui->isEditMode) {
            ui->set_temp += (float)steps * ui->step;

            // Limity (Hard Stop)
            if (ui->set_temp > ui->maxLimit) ui->set_temp = ui->maxLimit;
            if (ui->set_temp < ui->minLimit) ui->set_temp = ui->minLimit;
        }

        // 4. Aktualizacja historii
        // WAŻNE: Dodajemy do lastCounter tylko tyle kroków, ile faktycznie przetworzyliśmy.
        // steps * 4 to liczba "zużytych" impulsów.
        // Dzięki temu nie gubimy resztek (np. jak zrobisz 3 impulsy teraz i 1 w kolejnej pętli).
        ui->lastCounter += (steps * 4);
    }
}
