/*
 * lcd.h
 *
 *  Created on: 2023/07/18
 *      Author: ryuta
 */

#ifndef LCD_H
#define LCD_H

void lcd_clear(void);
void lcd_init(void);
void lcd_suspend(void);
void lcd_resume(void);
bool_t lcd_set_cursor(uint8_t row, uint8_t col, bool_t visible, bool_t blink);
void lcd_draw_text(uint8_t row, uint8_t* text);

#endif /* LCD_H */
