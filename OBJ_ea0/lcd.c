/*
 * lcd.c
 *
 *  Created on: 2023/07/18
 *      Author: ryuta
 */

#include <string.h>
#include <kernel.h>
#include "t_i2c.h"

#include "lcd.h"

#define CMD 0x00
#define DAT 0x40

extern I2C_HandleTypeDef i2c;

static bool_t m_visible = false;
static bool_t m_blink = false;


void lcd_clear(void)
{
    uint8_t buf[2];

    buf[0] = CMD;
    buf[1] = 0x01;  // Clear Display
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);
}

void lcd_init(void)
{
    uint8_t buf[2];

    dly_tsk(40);

    buf[0] = CMD;
    buf[1] = 0x38;  // Function Set(IS=0)
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);

    buf[0] = CMD;
    buf[1] = 0x39;  // Function Set(IS=1)
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);

    buf[0] = CMD;
    buf[1] = 0x14;  // Internal OSC freq
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);

    buf[0] = CMD;
    buf[1] = 0x70;  // Contrast set
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);

    buf[0] = CMD;
    buf[1] = 0x56;  // Power/ICON control/Conterast set
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);

    buf[0] = CMD;
    buf[1] = 0x6C;  // Follower control
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(300);

    buf[0] = CMD;
    buf[1] = 0x38;  // Function Set(IS=0)
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);

    buf[0] = CMD;
    buf[1] = 0x0C;  // Display ON/OFF
//    buf[1] = 0x0F;  // Display ON/OFF
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);
/*
    buf[0] = CMD;
    buf[1] = 0x01;  // Clear Display
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);
*/
    lcd_clear();
}

bool_t lcd_set_cursor(uint8_t row, uint8_t col, bool_t visible, bool_t blink)
{
    uint8_t buf[2];
    uint8_t tmp = 0x0C;

    if (row > 1)
        return false;
    if (col > 16)
        return false;

    buf[0] = CMD;

    if (row == 0)
        buf[1] = 0x80 + (uint8_t)col;
    else
        buf[1] = 0xC0 + (uint8_t)col;

    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);

    if (visible)
        tmp += 0x02;

    if (blink)
        tmp += 0x01;

    buf[0] = CMD;
    buf[1] = tmp;   // Display ON/OFF
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);

    m_visible = visible;
    m_blink = blink;

    return true;
}

void lcd_draw_text(uint8_t row, uint8_t col, uint8_t* text)
{
    uint8_t buf[2];
    uint8_t *p;
    int cnt = 0;
    bool_t eot = false;

    if (strlen((const char*)text) == 0)
        return;

    if (!lcd_set_cursor(row, col, m_visible, m_blink))
        return;

#if 0
    if (row > 1)
        return;
    if (col > 16)
        return;

    buf[0] = CMD;

    if (row == 0)
        buf[1] = 0x80 + (uint8_t)col;
    else
        buf[1] = 0xC0 + (uint8_t)col;

    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);
#endif

    p = text;

    do {
        if (*p == 0)
            eot = true;
        buf[0] = DAT;
        if (!eot)
            buf[1] = *p++;
        else
            buf[1] = 0x20;  // SPACE
        wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
        cnt++;
        //dly_tsk(10);
    } while ((col + cnt) < 16);
}
