/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2012 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *
 *  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 *
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: sample1.c 2728 2015-12-30 01:46:11Z ertl-honda $
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <kernel.h>
#include <msp430.h>
#include "kernel_cfg.h"
#include "driverlib.h"
#include "ea0.h"
#include "t_adc.h"
#include "t_i2c.h"
#include "t_uart.h"
#include "lcd.h"
#include "modbus.h"
#include "function.h"

#include "main.h"


#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

extern void target_fput_log(char c);

const uint8_t drum_polarity[] = {
    (uint8_t)'+',
    (uint8_t)'-',
};

const uint8_t drum_decimal[] = {
    (uint8_t)'0',
    (uint8_t)'1',
    (uint8_t)'2',
    (uint8_t)'3',
    (uint8_t)'4',
    (uint8_t)'5',
    (uint8_t)'6',
    (uint8_t)'7',
    (uint8_t)'8',
    (uint8_t)'9',
};

const uint8_t drum_hex[] = {
    (uint8_t)'0',
    (uint8_t)'1',
    (uint8_t)'2',
    (uint8_t)'3',
    (uint8_t)'4',
    (uint8_t)'5',
    (uint8_t)'6',
    (uint8_t)'7',
    (uint8_t)'8',
    (uint8_t)'9',
    (uint8_t)'A',
    (uint8_t)'B',
    (uint8_t)'C',
    (uint8_t)'D',
    (uint8_t)'E',
    (uint8_t)'F',
};

const uint8_t drum_binary[] = {
    (uint8_t)'0',
    (uint8_t)'1',
};

const uint8_t drum_float[] = {
    (uint8_t)'0',
    (uint8_t)'1',
    (uint8_t)'2',
    (uint8_t)'3',
    (uint8_t)'4',
    (uint8_t)'5',
    (uint8_t)'6',
    (uint8_t)'7',
    (uint8_t)'8',
    (uint8_t)'9',
    (uint8_t)'.',
};

#define MAX_RECORD  80

REG_RECORD  g_table[MAX_RECORD];
DSP_MODE g_dsp = Signed;
OPT_MODE g_opt = Address;
int g_current = 0;
int g_records = 0;

ADC_HandleTypeDef adc;
I2C_HandleTypeDef i2c;
UART_HandleTypeDef uart;

static uint8_t m_buf[32];
static uint8_t m_row = 0;

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
    target_fput_log((char)ch);
    return ch;
}

void usci_a1_isr(intptr_t exinf)
{
    uart_isr(&uart);
}

void usci_b1_isr(intptr_t exinf)
{
    i2c_isr(&i2c);
}

void adc12_isr(intptr_t exinf)
{
    adc_isr(&adc);
}

static bool_t debug = false;
void port2_isr(intptr_t exinf)
{
    debug = !debug;
    if (GPIO_getInterruptStatus(GPIO_PORT_P2, GPIO_PIN0)) {
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN0);
    }
    if (GPIO_getInterruptStatus(GPIO_PORT_P2, GPIO_PIN2)) {
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN2);
    }
    if (GPIO_getInterruptStatus(GPIO_PORT_P2, GPIO_PIN4)) {
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN4);
    }
    if (GPIO_getInterruptStatus(GPIO_PORT_P2, GPIO_PIN5)) {
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN5);
    }
    if (GPIO_getInterruptStatus(GPIO_PORT_P2, GPIO_PIN6)) {
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);
    }
}

static void main_init(void)
{
    // uart
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P4,
            GPIO_PIN4 + GPIO_PIN5
    );

    // I2C
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P4,
            GPIO_PIN1 + GPIO_PIN2
    );

    // ADC
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P6,
            GPIO_PIN0
    );

    // _RE
    GPIO_setAsOutputPin(
            GPIO_PORT_P8,
            GPIO_PIN2
    );

    // DE
    GPIO_setAsOutputPin(
            GPIO_PORT_P3,
            GPIO_PIN7
    );

    // Button = Red
    GPIO_setAsInputPin(
            GPIO_PORT_P2,
            GPIO_PIN6
    );

    // Button = White
    GPIO_setAsInputPin(
            GPIO_PORT_P2,
            GPIO_PIN2
    );

    // Button = Yellow
    GPIO_setAsInputPin(
            GPIO_PORT_P2,
            GPIO_PIN4
    );

    // Button = Green
    GPIO_setAsInputPin(
            GPIO_PORT_P2,
            GPIO_PIN0
    );

    // Button = Blue
    GPIO_setAsInputPin(
            GPIO_PORT_P2,
            GPIO_PIN5
    );

    GPIO_selectInterruptEdge(
            GPIO_PORT_P2,
            GPIO_PIN0 + GPIO_PIN2 + GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6,
            GPIO_HIGH_TO_LOW_TRANSITION
    );
/*
    GPIO_enableInterrupt(
            GPIO_PORT_P2,
            GPIO_PIN0 + GPIO_PIN2 + GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6
    );
*/
    uart.reg = (uint32_t)USCI_A1_BASE;
    ini_sio(&uart, "9600 B8 PN S1");
    ctl_sio(&uart, TUART_RXE | TUART_TXE | TUART_DTRON | TUART_RTSON);

    dly_tsk(40);
    i2c.reg = (uint32_t)USCI_B1_BASE;
    ini_i2c(&i2c);

    adc.reg = (uint32_t)ADC12_A_BASE;
    ini_adc(&adc);

    lcd_init();
    function_init();
}

static uint8_t get_next_drum(DSP_MODE mode, uint8_t* value, uint8_t digit, uint8_t current)
{
    int i;
    int siz;
    const uint8_t* p;

    switch (mode) {
    case Signed:
    case Long:
    case Long_Inverse:
        if (digit == 0) {
            siz = sizeof(drum_polarity);
            p = &drum_polarity[0];
        } else {
            siz = sizeof(drum_decimal);
            p = &drum_decimal[0];
        }
        break;
    case Unsigned:
        siz = sizeof(drum_decimal);
        p = &drum_decimal[0];
        break;
    case Hex:
        siz = sizeof(drum_hex);
        p = &drum_hex[0];
        break;
    case Binary:
        siz = sizeof(drum_binary);
        p = &drum_binary[0];
        break;
    //case Float:
    //case Float_Inverse:
    //case Double:
    //case Double_Inverse:
    default:
        if (digit == 0) {
            siz = sizeof(drum_polarity);
            p = &drum_polarity[0];
        } else {
            if (strchr((const char*)value, (int)'.') == NULL) {
                siz = sizeof(drum_float);
                p = &drum_float[0];
            } else {
                siz = sizeof(drum_decimal);
                p = &drum_decimal[0];
            }
        }
        break;
    }

    if (current == (uint8_t)'.')
        return (uint8_t)'0';

    for (i = 0; i < siz; i++) {
        if (current == p[i]) {
            i++;
            if (i == siz)
                i = 0;
            return p[i];
        }
    }

    return (uint8_t)'0';
}

static uint8_t get_back_drum(DSP_MODE mode, uint8_t* value, uint8_t digit, uint8_t current)
{
    int i;
    int siz;
    const uint8_t* p;

    switch (mode) {
    case Signed:
    case Long:
    case Long_Inverse:
        if (digit == 0) {
            siz = sizeof(drum_polarity);
            p = &drum_polarity[0];
        } else {
            siz = sizeof(drum_decimal);
            p = &drum_decimal[0];
        }
        break;
    case Unsigned:
        siz = sizeof(drum_decimal);
        p = &drum_decimal[0];
        break;
    case Hex:
        siz = sizeof(drum_hex);
        p = &drum_hex[0];
        break;
    case Binary:
        siz = sizeof(drum_binary);
        p = &drum_binary[0];
        break;
    //case Float:
    //case Float_Inverse:
    //case Double:
    //case Double_Inverse:
    default:
        if (digit == 0) {
            siz = sizeof(drum_polarity);
            p = &drum_polarity[0];
        } else {
            if (strchr((const char*)value, (int)'.') == NULL) {
                siz = sizeof(drum_float);
                p = &drum_float[0];
            } else {
                siz = sizeof(drum_decimal);
                p = &drum_decimal[0];
            }
        }
        break;
    }

    if (current == (uint8_t)'.')
        return (uint8_t)'9';

    for (i = 0; i < siz; i++) {
        if (current == p[i]) {
            i--;
            if (i < 0)
                i = (siz - 1);
            return p[i];
        }
    }

    return (uint8_t)'0';
}

static int get_records(void)
{
    int i;
    for (i = 0; i < MAX_RECORD; i++) {
        if (!g_table[i].registered)
            break;
    }
    return i;
}

static int get_digits(DSP_MODE mode)
{
    int dgt;
    switch (mode) {
    case Signed:
        dgt = 6;
        break;
    case Unsigned:
        dgt = 5;
        break;
    case Hex:
        dgt = 4;
        break;
    case Binary:
        dgt = 16;
        break;
    case Long:
    case Long_Inverse:
        dgt = 9;
        break;
    case Float:
    case Float_Inverse:
        dgt = 8;
        break;
    //case Double:
    //case Double_Inverse:
    default:
        dgt = 16;
        break;
    }
    return dgt;
}

static void draw(uint8_t* pAdr, uint8_t* pVal)
{
    //uint8_t adr[8];
    uint8_t idx[8];

    //sprintf((char*)adr, "%05u", g_table[g_current].address);
    if (pAdr != NULL) {
        sprintf((char*)idx, "[%02d/%02d]", g_current + 1, g_records);
        sprintf((char*)m_buf, "R:%s  %s", pAdr, idx);
        lcd_draw_text(0, 0, (uint8_t*)m_buf);
    }
    if (pVal != NULL) {
        strcpy((char*)m_buf, (const char*)pVal);
        lcd_draw_text(1, 0, (uint8_t*)m_buf);
    }
//    lcd_set_cursor(row, col, visible, blink);
}

/*
 *  printf()やsprintf()で「%f」や「%g」を使用する場合は
 *  リンカのオプションとして「-u _printf_float」を追記すること
 */
void main_task(intptr_t exinf)
{
    int i;
    uint8_t adr[8];
    //uint8_t idx[8];
    uint8_t dgt = 0;
    FLGPTN p_flgptn;
    FLGPTN sw = 0xFFFF;
    uint8_t val[32];

    main_init();

    for (i = 0; i < MAX_RECORD; i++)
        g_table[i].address = 1;

    sprintf((char*)adr, "%05u", g_table[g_current].address);
    draw(adr, NULL);
    lcd_set_cursor(0, (2 + dgt), false, true);

    act_tsk(TSK_POL);

    while (1) {
        if (E_TMOUT != twai_flg(FLG_INP, sw, TWF_ORW, &p_flgptn, 100)) {
            //wai_sem(SEM_DRW);
            if (p_flgptn & EVENT_ESC_OFF) {
                //strcpy((char*)m_buf, "RED : High\r\n");
            } else
            if (p_flgptn & EVENT_ESC_ON) {
                switch (g_opt) {
                case Address:
                    if (get_records() == 0)
                        break;
                    if (g_current == 0)
                        g_current = get_records() - 1;
                    else
                        g_current--;
                    g_opt = View;
                    sprintf((char*)adr, "%05u", g_table[g_current].address);
                    draw(adr, NULL);
                    lcd_set_cursor(m_row, 0, true, false);
                    sig_sem(SEM_POL);
                    break;
                case Value:
                    g_opt = View;
                    sprintf((char*)adr, "%05u", g_table[g_current].address);
                    draw(adr, NULL);
                    lcd_set_cursor(m_row, 0, true, false);
                    sig_sem(SEM_POL);
                    break;
                }
            } else
            if (p_flgptn & EVENT_SHF_OFF) {
                //strcpy((char*)m_buf, "WHITE : High\r\n");
            } else
            if (p_flgptn & EVENT_SHF_ON) {
                switch (g_opt) {
                case Address:
                    dgt++;
                    if (dgt == 5)
                        dgt = 0;
                    lcd_set_cursor(0, (2 + dgt), false, true);
                    break;
                case Value:
                    dgt++;
                    if (dgt == get_digits(g_dsp))
                        dgt = 0;
                    lcd_set_cursor(1, dgt, false, true);
                    break;
                }
            } else
            if (p_flgptn & EVENT_UP__OFF) {
                //strcpy((char*)m_buf, "YELLOW : High\r\n");
            } else
            if (p_flgptn & EVENT_UP__ON) {
                switch (g_opt) {
                case Address:
                    adr[dgt] = get_next_drum(Unsigned, adr, dgt, adr[dgt]);
                    draw(adr, NULL);
                    lcd_set_cursor(0, (2 + dgt), false, true);
                    break;
                case Value:
                    val[dgt] = get_next_drum(g_dsp, val, dgt, val[dgt]);
                    draw(NULL, val);
                    lcd_set_cursor(1, dgt, false, true);
                    break;
                //case View:
                default:
                    if (m_row == 1) {
                        wai_sem(SEM_DRW);
                        m_row = 0;
                        lcd_set_cursor(m_row, 0, false, true);
                        sig_sem(SEM_DRW);
                    } else {
                        //rel_wai(TSK_POL);
                        wai_sem(SEM_POL);
                        if (g_current == 0)
                            g_current = get_records() - 1;
                        else
                            g_current--;
                        m_row = 0;
                        sprintf((char*)adr, "%05u", g_table[g_current].address);
                        draw(adr, NULL);
                        lcd_set_cursor(m_row, 0, true, false);
                        sig_sem(SEM_POL);
                    }
                    break;
                }
            } else
            if (p_flgptn & EVENT_DWN_OFF) {
                //strcpy((char*)m_buf, "GREEN : High\r\n");
            } else
            if (p_flgptn & EVENT_DWN_ON) {
                switch (g_opt) {
                case Address:
                    adr[dgt] = get_back_drum(Unsigned, adr, dgt, adr[dgt]);
                    draw(adr, NULL);
                    lcd_set_cursor(0, (2 + dgt), false, true);
                    break;
                case Value:
                    val[dgt] = get_back_drum(g_dsp, val, dgt, val[dgt]);
                    draw(NULL, val);
                    lcd_set_cursor(1, dgt, false, true);
                    break;
                //case View:
                default:
                    if (m_row == 0) {
                        wai_sem(SEM_DRW);
                        m_row = 1;
                        lcd_set_cursor(m_row, 0, false, true);
                        sig_sem(SEM_DRW);
                    } else {
                        //rel_wai(TSK_POL);
                        wai_sem(SEM_POL);
                        if (g_current == (MAX_RECORD - 1))
                            g_current = 0;
                        else
                            g_current++;
                        m_row = 0;
                        if (!g_table[g_current].registered) {
                            g_opt = Address;
                            lcd_clear();
                            dgt = 0;
                            sprintf((char*)adr, "%05u", g_table[g_current].address);
                            draw(adr, NULL);
                            lcd_set_cursor(0, (2 + dgt), false, true);
                        } else {
                            sprintf((char*)adr, "%05u", g_table[g_current].address);
                            draw(adr, NULL);
                            lcd_set_cursor(m_row, 0, true, false);
                            sig_sem(SEM_POL);
                        }
                    }
                    break;
                }
            } else
            if (p_flgptn & EVENT_ENT_OFF) {
                //strcpy((char*)m_buf, "BLUE : High\r\n");
            } else
            if (p_flgptn & EVENT_ENT_ON) {
                switch (g_opt) {
                case Address:
                    g_table[g_current].address = strtoul((const char*)adr, NULL, 10);
                    g_table[g_current].registered = true;
                    g_records = get_records();
                    g_opt = View;
                    sprintf((char*)adr, "%05u", g_table[g_current].address);
                    draw(adr, NULL);
                    lcd_set_cursor(m_row, 0, true, false);
                    sig_sem(SEM_POL);
                    break;
                case Value:
                    //
                    break;
                //case View:
                default:
                    //rel_wai(TSK_POL);
                    wai_sem(SEM_POL);
                    if (m_row == 0) {
                        g_opt = Address;
                        lcd_clear();
                        dgt = 0;
                        sprintf((char*)adr, "%05u", g_table[g_current].address);
                        draw(adr, NULL);
                        lcd_set_cursor(0, (2 + dgt), false, true);
                    } else {
                        g_opt = Value;
                        lcd_clear();
                        dgt = 0;
                        sprintf((char*)adr, "%05u", g_table[g_current].address);
                        sprintf((char*)val, "%+06d", g_table[g_current].data[0]);
                        draw(adr, val);
                        lcd_set_cursor(1, dgt, false, true);
                    }
                    break;
                }
            } else
            if (p_flgptn & EVENT_MNU) {
                //strcpy((char*)m_buf, "GOTO MENU\r\n");
            }
            //sig_sem(SEM_DRW);
        }
    }
}

void poll_task(intptr_t exinf)
{
    uint16_t top;
    ER ret;
    //uint16_t reg[4];

    while (1) {
        wai_sem(SEM_POL);
        top = g_table[g_current].address / 10000;
        switch (top) {
        case 0:
            break;
        case 1:
            break;
        case 3:
            break;
        case 4:
            ret = modbus_read_register(
                    HOLDING_REGISTER,
                    0x0001U,
                    (g_table[g_current].address - 40001),
                    1,
                    &g_table[g_current].data[0],
                    1000
                    );
            if (ret != E_OK)
                break;
            sprintf((char*)m_buf, "%d", g_table[g_current].data[0]);
            wai_sem(SEM_DRW);
            lcd_set_cursor(m_row, 0, false, false);
            lcd_draw_text(1, 0, (uint8_t*)m_buf);
            lcd_set_cursor(m_row, 0, true, false);
            sig_sem(SEM_DRW);
            break;
        default:
            break;
        }
        sig_sem(SEM_POL);
        //dly_tsk(100);
    }
}

#if 0
void main_task(intptr_t exinf)
{
    uint8_t buf[32];
    float tmp = 1.23456;
    uint8_t* p;
    int i;
    uint8_t c;
    uint16_t val;
    int bytes;
    uint16_t reg[4];
    uint8_t sts[4];
    int len;
    ER ret;
    FLGPTN p_flgptn;
    FLGPTN sw = 0xFFFF;

    main_init();

    //modbus_set_mode(true);

    sprintf((char*)buf, "sprintf() Test = %g\r\n", tmp);
//#if 0
    p = &buf[0];
    for (i = 0; i < strlen((const char*)buf); i++) {
        put_sio(&uart, *p, 10);
        p++;
    }
//#endif
#if 0
    c = 'T';
    while (1) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);   // DE High
        put_sio(&uart, c, 10);
        if (E_OK == get_sio(&uart, &c, 1000)) {
            if (c == 'T')
                lcd_draw_text(0, 0, (uint8_t*)"OK!");
            else
                lcd_draw_text(0, 0, (uint8_t*)"NG!");
        } else {
            lcd_draw_text(0, 0, (uint8_t*)"NG!");
        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);    // DE Low
        dly_tsk(500);
        lcd_clear();
        dly_tsk(500);
    }
#endif

    // Display
    //lcd_draw_text(0, 0, (uint8_t*)"0123456789ABCDEF");
    //lcd_set_cursor(1, 0, false, true);

    //sta_adc(&adc);
    //dly_tsk(10);

    //__bis_SR_register(LPM4_bits + GIE);

    while (1) {
        if (E_TMOUT != twai_flg(FLG_INP, sw, TWF_ORW, &p_flgptn, 100)) {
            if (p_flgptn & EVENT_ESC_OFF) {
                strcpy((char*)buf, "RED : High\r\n");
            } else
            if (p_flgptn & EVENT_ESC_ON) {
                strcpy((char*)buf, "RED : Low\r\n");
            } else
            if (p_flgptn & EVENT_SHF_OFF) {
                strcpy((char*)buf, "WHITE : High\r\n");
            } else
            if (p_flgptn & EVENT_SHF_ON) {
                strcpy((char*)buf, "WHITE : Low\r\n");
            } else
            if (p_flgptn & EVENT_UP__OFF) {
                strcpy((char*)buf, "YELLOW : High\r\n");
            } else
            if (p_flgptn & EVENT_UP__ON) {
                strcpy((char*)buf, "YELLOW : Low\r\n");
            } else
            if (p_flgptn & EVENT_DWN_OFF) {
                strcpy((char*)buf, "GREEN : High\r\n");
            } else
            if (p_flgptn & EVENT_DWN_ON) {
                strcpy((char*)buf, "GREEN : Low\r\n");
            } else
            if (p_flgptn & EVENT_ENT_OFF) {
                strcpy((char*)buf, "BLUE : High\r\n");
            } else
            if (p_flgptn & EVENT_ENT_ON) {
                strcpy((char*)buf, "BLUE : Low\r\n");
            } else
            if (p_flgptn & EVENT_MNU) {
                strcpy((char*)buf, "GOTO MENU\r\n");
            }
            p = &buf[0];
            for (i = 0; i < strlen((const char*)buf); i++) {
                put_sio(&uart, *p, 10);
                p++;
            }
        }

#if 0
        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) == GPIO_INPUT_PIN_HIGH)
            sprintf((char*)buf, "RED : High\r\n");
        else
            sprintf((char*)buf, "RED : Low\r\n");
        p = &buf[0];
        for (i = 0; i < strlen((const char*)buf); i++) {
            put_sio(&uart, *p, 10);
            p++;
        }
        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN2) == GPIO_INPUT_PIN_HIGH)
            sprintf((char*)buf, "WHITE : High\r\n");
        else
            sprintf((char*)buf, "WHITE : Low\r\n");
        p = &buf[0];
        for (i = 0; i < strlen((const char*)buf); i++) {
            put_sio(&uart, *p, 10);
            p++;
        }
        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN4) == GPIO_INPUT_PIN_HIGH)
            sprintf((char*)buf, "YELLOW : High\r\n");
        else
            sprintf((char*)buf, "YELLOW : Low\r\n");
        p = &buf[0];
        for (i = 0; i < strlen((const char*)buf); i++) {
            put_sio(&uart, *p, 10);
            p++;
        }
        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN0) == GPIO_INPUT_PIN_HIGH)
            sprintf((char*)buf, "GREEN : High\r\n");
        else
            sprintf((char*)buf, "GREEN : Low\r\n");
        p = &buf[0];
        for (i = 0; i < strlen((const char*)buf); i++) {
            put_sio(&uart, *p, 10);
            p++;
        }
        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == GPIO_INPUT_PIN_HIGH)
            sprintf((char*)buf, "BLUE : High\r\n");
        else
            sprintf((char*)buf, "BLUE : Low\r\n");
        p = &buf[0];
        for (i = 0; i < strlen((const char*)buf); i++) {
            put_sio(&uart, *p, 10);
            p++;
        }
        dly_tsk(100);
#endif
#if 0
        ret = modbus_read_register(HOLDING_REGISTER, 0x0001U, 0x0000U, 2, &reg[0], 1000);
        if (E_OK == ret) {
            sprintf((char*)buf, "%d", reg[0]);
            lcd_draw_text(0, 0, buf);
            sprintf((char*)buf, "%d", reg[1]);
            lcd_draw_text(1, 0, buf);
        } else {
            switch (ret) {
            case E_MODBUS_BFNC:
                sprintf((char*)buf, "Illegal Func.");
                lcd_draw_text(0, 0, buf);
                break;
            case E_MODBUS_BADR:
                sprintf((char*)buf, "Illegal DataAdr.");
                lcd_draw_text(0, 0, buf);
                break;
            case E_MODBUS_BDAT:
                sprintf((char*)buf, "Illegal DataVal.");
                lcd_draw_text(0, 0, buf);
                break;
            }
        }
#endif
#if 0
        if (E_OK == modbus_read_status(COIL_STATUS, 0x0001U, 0x0000U, 10, &sts[0], 1000)) {
            sprintf((char*)buf, "%d", ((sts[0] >> 0) & 0x0001));
            lcd_draw_text(0, 0, buf);
            sprintf((char*)buf, "%d", ((sts[1] >> 1) & 0x0001));
            lcd_draw_text(1, 0, buf);
        }
#endif
#if 0
        reg[0] = 0x1234;
        //reg[1] = 0x5678;
        modbus_write_register(HOLDING_REGISTER, 0x0001U, 0x0000U, 1, &reg[0], 1000);
#endif
#if 0
        sts[0] = 0xAA;
        sts[1] = 0x02;
        ret = modbus_write_status(COIL_STATUS, 0x0001U, 0x0000U, 11, &sts[0], 1000);
        if (E_OK == ret) {
            //
        } else {
            switch (ret) {
            case E_MODBUS_BFNC:
                sprintf((char*)buf, "Illegal Func.");
                lcd_draw_text(0, 0, buf);
                break;
            case E_MODBUS_BADR:
                sprintf((char*)buf, "Illegal DataAdr.");
                lcd_draw_text(0, 0, buf);
                break;
            case E_MODBUS_BDAT:
                sprintf((char*)buf, "Illegal DataVal.");
                lcd_draw_text(0, 0, buf);
                break;
            }
        }
        dly_tsk(100);
#endif
#if 0
        if (E_OK == get_sio(&uart, &c, 1000)) {
            put_sio(&uart, c, 10);
        }
#endif
#if 0
//        sprintf((char*)buf, "val = %u\r\n", red_adc(&adc));
        if (E_OK == get_adc(&adc, &val, 10))
            sprintf((char*)buf, "val = %u\r\n", val);
        p = &buf[0];
        for (i = 0; i < strlen((const char*)buf); i++) {
            put_sio(&uart, *p, 10);
            p++;
        }
        dly_tsk(1000);
#endif
    }
}
#endif
