#include "i2c-lcd.h"

extern I2C_HandleTypeDef hi2c2;  // Uchwyt z main.c

// ADRES LCD: Najczęściej 0x27 (4E przy przesunięciu) lub 0x3F (7E)
#define SLAVE_ADDRESS_LCD 0x4E // 0x27 << 1

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (cmd&0xf0);
  data_l = ((cmd<<4)&0xf0);

  // UWAGA: Bit 3 (0x08) to zazwyczaj Podświetlenie (Backlight). Musi być ON.
  // Bit 2 (0x04) to zazwyczaj Enable.
  // Bit 0 (0x01) to RS.

  // Wysyłamy górną połówkę bajtu
  data_t[0] = data_u | 0x0C;  // en=1, rs=0, bl=1 (0x08 | 0x04 = 0x0C)
  data_t[1] = data_u | 0x08;  // en=0, rs=0, bl=1

  // Wysyłamy dolną połówkę bajtu
  data_t[2] = data_l | 0x0C;  // en=1, rs=0, bl=1
  data_t[3] = data_l | 0x08;  // en=0, rs=0, bl=1

  HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (data&0xf0);
  data_l = ((data<<4)&0xf0);

  // Tu RS=1 (0x01), więc sumujemy: 0x08(BL) + 0x04(EN) + 0x01(RS) = 0x0D

  data_t[0] = data_u | 0x0D;  // en=1, rs=1, bl=1
  data_t[1] = data_u | 0x09;  // en=0, rs=1, bl=1 (0x08 + 0x01)

  data_t[2] = data_l | 0x0D;  // en=1, rs=1, bl=1
  data_t[3] = data_l | 0x09;  // en=0, rs=1, bl=1

  HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}
void lcd_clear (void)
{
	lcd_send_cmd (0x01); //Clear display
	HAL_Delay(2); // Komenda clear wymaga min 2ms
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    lcd_send_cmd (col);
}

void lcd_init (void)
{
	HAL_Delay(50);  // Czekamy aż napięcie się ustabilizuje

	// 1. Procedura resetu (magiczna sekwencja 0x33, 0x32)
	lcd_send_cmd (0x33);
	HAL_Delay(5);
	lcd_send_cmd (0x32); // Przełączenie w tryb 4-bit
	HAL_Delay(10);

	// 2. Konfiguracja właściwa
	lcd_send_cmd (0x28); // Function set: 4-bit, 2 lines, 5x8 dots
	HAL_Delay(1);
	lcd_send_cmd (0x08); // Display OFF (ważne przed czyszczeniem)
	HAL_Delay(1);
	lcd_send_cmd (0x01); // Clear Display
	HAL_Delay(2);        // Ta komenda wymaga aż 2ms!
	lcd_send_cmd (0x06); // Entry Mode: Auto-increment cursor
	HAL_Delay(1);
	lcd_send_cmd (0x0C); // Display ON, Cursor OFF, Blink OFF
	HAL_Delay(1);
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}
