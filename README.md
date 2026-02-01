# Projekt-kontroli-temperatury

![Language](https://img.shields.io/badge/language-C-blue.svg)
![Platform](https://img.shields.io/badge/platform-STM32-green.svg)

System regulacji temperatury w pętli zamkniętej oparty na mikrokontrolerze STM32. Projekt wykorzystuje **algorytm PID** do sterowania elementem grzejnym oraz wentylatorem chłodzącym, utrzymując zadaną temperaturę z wysoką precyzją na podstawie odczytów z czujnika **BMP280**.

Projekt oferuje interfejs lokalny (LCD + Enkoder) oraz rozbudowany **protokół komunikacyjny UART z weryfikacją CRC-16**, zaprojektowany z myślą o łatwej integracji ze środowiskiem **MATLAB/Simulink** oraz narzędziami do logowania danych.

## Główne Funkcjonalności

* **Podwójny Regulator PID:** Osobne pętle sterowania dla Grzania i Chłodzenia.
* **Zakres Regulacji:** System zaprojektowany do precyzyjnej pracy w przedziale **25°C – 38°C**.
* **Akwizycja Danych:** Odczyt temperatury i ciśnienia z czujnika **BMP280** (I2C).
* **Niezawodna Komunikacja:**
    * Własny protokół UART (115200 baud).
    * Weryfikacja sumy kontrolnej **CRC-16 ARC** dla komend przychodzących.
    * Format danych wyjściowych zoptymalizowany pod parsowanie w **Simulinku**.
* **Interfejs Użytkownika:**
    * Wyświetlacz LCD 16x2 (I2C) prezentujący temperaturę zadaną, aktualną oraz wysterowanie wyjść.
    * Enkoder obrotowy do zmiany nastaw.
    * Wizualny wskaźnik trybu edycji (`<`) na ekranie.
* **Konfigurowalność:** Zmienny czas próbkowania realizowany w pętli głównej.

## Wymagania Sprzętowe

* **Mikrokontroler:** STM32 F7.
* **Czujnik:** BMP280 (Interfejs I2C).
* **Elementy Wykonawcze:**
    * **Grzałka:** Zestaw rezystorów mocy połączonych równolegle (2x 15Ω oraz 2x 10Ω, wszystkie 5W), sterowany sygnałem PWM.
    * **Wentylator:** Sterowany sygnałem PWM.
* **Wyświetlacz:** LCD 16x2 ze sterownikiem PCF8574 (I2C).
* **Wejście:** Enkoder obrotowy z przyciskiem.

## Protokół Komunikacji UART

**Ustawienia:** 115200 Baud, 8 bitów danych, brak parzystości, 1 bit stopu (8N1).

### 1. Dane Wyjściowe
Sterownik wysyła ramkę danych w każdym cyklu próbkowania (domyślnie 100ms). Format zaprojektowano pod parsowanie wyrażeniami regularnymi (Regex).

Format: ` $ZZZZ,AAAA,HHH,FFF\r\n`

| Pole | Opis | Przykład |
| :--- | :--- | :--- |
| **$** | Znak początku ramki | `$` |
| **ZZZZ** | Temperatura Zadana * 100 | `2550` (25.50°C) |
| **AAAA** | Temperatura Aktualna * 100 | `2480` (24.80°C) |
| **HHH** | Wypełnienie PWM Grzałki (0-100%) | `045` |
| **FFF** | Wypełnienie PWM Wentylatora (0-100%) | `000` |

**Przykładowa surowa ramka:** `$2550,2480,045,000`

---

### 2. Komendy Sterujące (Wejściowe)

System akceptuje komendy w dwóch formatach:
* **Zabezpieczony:** `KOMENDA*CRC` (Zalecany dla automatyki/Simulinka)
* **Niezabezpieczony:** `KOMENDA` (Do ręcznych testów w terminalu)

| Komenda | Opis | Przykład (Z CRC) |
| :--- | :--- | :--- |
| **ON** | Włącz pętlę PID | `ON*04B4` |
| **OFF** | Wyłącz pętlę PID (STOP) | `OFF*4582` |
| **Txxx** | Ustaw temperaturę (x10) | `T300*2BF4` |


## Konfiguracja

Projekt został wygenerowany przy użyciu **STM32CubeMX**.
* **Pętla Główna:** Znajduje się w pliku `main.c`.
* **Strojenie PID:** Stałe `Kp`, `Ki`, `Kd` są zdefiniowane na początku pliku `main.c`.
* **Czas Próbkowania:** Możliwość zmiany poprzez definicję `#define LOOP_TIME_MS`.

## Zdjęcia układu
![układ](https://github.com/user-attachments/assets/ede39513-c3c2-4d17-9924-415e6430b8a7)
![układ 2](https://github.com/user-attachments/assets/0519e626-1003-4be4-8797-3cc5c9c2c6ff)

