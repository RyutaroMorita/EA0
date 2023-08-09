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
#define DLY 5000U

extern I2C_HandleTypeDef i2c;

static bool_t m_init = false;
static bool_t m_visible = false;
static bool_t m_blink = false;
static uint8_t m_backup[2][17];
static uint8_t m_row;
static uint8_t m_col;


__attribute__ ((section (".subtext"))) void lcd_clear(void)
{
    uint8_t buf[2];
    //volatile int i;

    if (!m_init)
        return;

    buf[0] = CMD;
    buf[1] = 0x01;  // Clear Display
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);
    //for (i = 0; i < DLY; i++);
}

__attribute__ ((section (".subtext"))) void lcd_init(void)
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
    m_init = true;

    //lcd_clear();
}

__attribute__ ((section (".subtext"))) void lcd_suspend(void)
{
    uint8_t buf[2];

    lcd_clear();

    buf[0] = CMD;
    buf[1] = 0x38;  // Function Set(IS=0)
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);

    buf[0] = CMD;
    buf[1] = 0x08;  // Display ON/OFF
//    buf[1] = 0x0F;  // Display ON/OFF
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);

    buf[0] = CMD;
    buf[1] = 0x39;  // Function Set(IS=1)
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);

    buf[0] = CMD;
    buf[1] = 0x50;  // Power/ICON control/Conterast set
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);
}

__attribute__ ((section (".subtext"))) void lcd_resume(void)
{
    //uint8_t buf[2];
    uint8_t row;
    uint8_t col;

    /*
    buf[0] = CMD;
    buf[1] = 0x38;  // Function Set(IS=0)
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);

    buf[0] = CMD;
    buf[1] = 0x0C;  // Display ON/OFF
//    buf[1] = 0x0F;  // Display ON/OFF
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);
    */

    lcd_init();

    row = m_row;
    col = m_col;
    lcd_draw_text(0, m_backup[0]);
    lcd_draw_text(1, m_backup[1]);
    lcd_set_cursor(row, col, m_visible, m_blink);
}

__attribute__ ((section (".subtext"))) bool_t lcd_set_cursor(uint8_t row, uint8_t col, bool_t visible, bool_t blink)
{
    uint8_t buf[2];
    uint8_t tmp = 0x0C;
    //volatile int i;

    if (!m_init)
        return false;
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
    //for (i = 0; i < DLY; i++);

    if (visible)
        tmp += 0x02;

    if (blink)
        tmp += 0x01;

    buf[0] = CMD;
    buf[1] = tmp;   // Display ON/OFF
    wrt_i2c(&i2c, 0x7c, &buf[0], 2, 10);
    dly_tsk(10);
    //for (i = 0; i < DLY; i++);

    m_visible = visible;
    m_blink = blink;
    m_row = row;
    m_col = col;

    return true;
}

__attribute__ ((section (".subtext"))) void lcd_draw_text(uint8_t row, uint8_t* text)
{
    uint8_t buf[2];
    uint8_t *p;
    int cnt = 0;
    bool_t eot = false;

    if (!m_init)
        return;

    if (strlen((const char*)text) == 0)
        return;

    if (!lcd_set_cursor(row, 0, m_visible, m_blink))
        return;

    memcpy(m_backup[row], text, 16);
    m_backup[row][16] = 0;

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
        //dly_tsk(10);
        cnt++;
    } while (cnt < 16);
}
