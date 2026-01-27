#ifndef I2C_LCD_H_
#define I2C_LCD_H_

#include "stm32f7xx_hal.h" // Zmień na f4/f1 zależnie od procka, jeśli Cube sam nie podpowie

void lcd_init (void);   // Inicjalizacja
void lcd_send_cmd (char cmd);  // Wysłanie komendy
void lcd_send_data (char data);  // Wysłanie znaku
void lcd_send_string (char *str);  // Wysłanie całego napisu
void lcd_put_cur(int row, int col);  // Ustawienie kursora (0-1 wiersz, 0-15 kolumna)
void lcd_clear (void);

#endif /* I2C_LCD_H_ */
