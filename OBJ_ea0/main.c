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
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
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

#define VERSION "01.00.00.00"

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

ADC_HandleTypeDef adc;
I2C_HandleTypeDef i2c;
UART_HandleTypeDef uart;

REG_RECORD  g_table[MAX_RECORD];
int g_current = 0;
ER g_err = E_OK;
uint8_t g_row = 0;
DSP_MODE g_dsp = Signed;
int g_hold = 0;
uint16_t g_id = 1;
uint8_t g_buf[32];

static int m_records = 0;
static MNU_ITEM_BAUDRATE m_baudrate = _9600;
static MNU_ITEM_PARITY m_parity = None;
static MNU_ITEM_STOPBIT m_stopbit = _1;
static OPT_MODE m_opt = Address;
static MNU_ITEM m_itm = Exit;
static MNU_ITEM_TIMER m_timer_b;
static MNU_ITEM_WIRING m_wiring_b;
static MNU_ITEM_BAUDRATE m_baudrate_b;
static MNU_ITEM_PARITY m_parity_b;
static MNU_ITEM_STOPBIT m_stopbit_b;
static MNU_ITEM_TIMER m_timer = _10Min;
static MNU_ITEM_WIRING m_wiring = RS232C;
static uint8_t m_row_b;
static float mVperUnit = (3300.0 / 4096.0);

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

//static bool_t debug = false;
void port2_isr(intptr_t exinf)
{
    //debug = !debug;
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

__attribute__ ((section (".subtext"))) static void main_init(void)
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
    lcd_clear();
    function_init();
}

__attribute__ ((section (".subtext"))) static uint8_t get_next_drum(DSP_MODE mode, uint8_t* value, uint8_t digit, uint8_t current)
{
    int i;
    int siz;
    const uint8_t* p;

    switch (mode) {
    //case Signed:
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
    case Float:
    case Float_Inverse:
    case Double:
    case Double_Inverse:
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
    //case Signed:
    default:
        if (digit == 0) {
            siz = sizeof(drum_polarity);
            p = &drum_polarity[0];
        } else {
            siz = sizeof(drum_decimal);
            p = &drum_decimal[0];
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

__attribute__ ((section (".subtext"))) static uint8_t get_back_drum(DSP_MODE mode, uint8_t* value, uint8_t digit, uint8_t current)
{
    int i;
    int siz;
    const uint8_t* p;

    switch (mode) {
    //case Signed:
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
    case Float:
    case Float_Inverse:
    case Double:
    case Double_Inverse:
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
    //case Signed:
    default:
        if (digit == 0) {
            siz = sizeof(drum_polarity);
            p = &drum_polarity[0];
        } else {
            siz = sizeof(drum_decimal);
            p = &drum_decimal[0];
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

__attribute__ ((section (".subtext"))) static int get_records(void)
{
    int i;
    for (i = 0; i < MAX_RECORD; i++) {
        if (!g_table[i].registered)
            break;
    }
    return i;
}

__attribute__ ((section (".subtext"))) static int get_digits(DSP_MODE mode)
{
    int dgt;
    switch (mode) {
    //case Signed:
    //    dgt = 6;
    //    break;
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
        dgt = 11;
        break;
    case Float:
    case Float_Inverse:
        dgt = 16;
        break;
    case Double:
    case Double_Inverse:
        dgt = 16;
        break;
    //case Signed:
    default:
        dgt = 6;
        break;
    }
    return dgt;
}

__attribute__ ((section (".subtext"))) static void draw_view(uint8_t* pAdr)
{
    uint8_t idx[8];
    uint8_t buf[32];

    if (pAdr != NULL) {
        sprintf((char*)idx, "[%02d/%02d]", g_current + 1, m_records);
        sprintf((char*)buf, "R:%s  %s", pAdr, idx);
        lcd_draw_text(0, (uint8_t*)buf);
    }
}

__attribute__ ((section (".subtext"))) static void draw_item(MNU_ITEM item)
{
    if (item != Null) {
        switch (item) {
        case Exit:
            lcd_draw_text(0, (uint8_t*)"Exit     [01/10]");
            break;
        case Suspend:
            lcd_draw_text(0, (uint8_t*)"Suspend  [02/10]");
            break;
        case Timer:
            lcd_draw_text(0, (uint8_t*)"Timer    [03/10]");
            break;
        case TargetID:
            lcd_draw_text(0, (uint8_t*)"TargetID [04/10]");
            break;
        case Wiring:
            lcd_draw_text(0, (uint8_t*)"Wiring   [05/10]");
            break;
        case Baudrate:
            lcd_draw_text(0, (uint8_t*)"Boudrate [06/10]");
            break;
//        case DataBit:
//            lcd_draw_text(0, (uint8_t*)"DataBit  [07/10]");
//            break;
        case Parity:
            lcd_draw_text(0, (uint8_t*)"Parity   [07/10]");
            break;
        case StopBit:
            lcd_draw_text(0, (uint8_t*)"StopBit  [08/10]");
            break;
        case Battery:
            lcd_draw_text(0, (uint8_t*)"Battery  [09/10]");
            break;
        //case Version:
        default:
            lcd_draw_text(0, (uint8_t*)"Version  [10/10]");
            break;
        }
    }
}

__attribute__ ((section (".subtext"))) static void draw_item_timer(MNU_ITEM_TIMER timer)
{
    switch (timer) {
    case _1Min:
        lcd_draw_text(1, (uint8_t*)"1 Min.          ");
        break;
    case _3Min:
        lcd_draw_text(1, (uint8_t*)"3 Min.          ");
        break;
    case _5Min:
        lcd_draw_text(1, (uint8_t*)"5 Min.          ");
        break;
    case _10Min:
        lcd_draw_text(1, (uint8_t*)"10 Min.         ");
        break;
    default:
        lcd_draw_text(1, (uint8_t*)"None            ");
        break;
    }
}

__attribute__ ((section (".subtext"))) static void draw_item_wiring(MNU_ITEM_WIRING wiring)
{
    switch (wiring) {
    case RS485:
        lcd_draw_text(1, (uint8_t*)"RS485           ");
        break;
    default:
        lcd_draw_text(1, (uint8_t*)"RS232C          ");
        break;
    }
}

__attribute__ ((section (".subtext"))) static void draw_item_baudrate(MNU_ITEM_BAUDRATE baudrate)
{
    switch (baudrate) {
//    case _14400:
//        lcd_draw_text(1, (uint8_t*)"14400 Baud      ");
//        break;
    case _19200:
        lcd_draw_text(1, (uint8_t*)"19200 Baud      ");
        break;
    case _38400:
        lcd_draw_text(1, (uint8_t*)"38400 Baud      ");
        break;
//    case _56000:
//        lcd_draw_text(1, (uint8_t*)"56000 Baud      ");
//        break;
    case _57600:
        lcd_draw_text(1, (uint8_t*)"57600 Baud      ");
        break;
    case _115200:
        lcd_draw_text(1, (uint8_t*)"115200 Baud     ");
        break;
    default:
        lcd_draw_text(1, (uint8_t*)"9600 Baud       ");
        break;
    }
}

/*
__attribute__ ((section (".subtext"))) static void draw_item_databit(MNU_ITEM_DATABIT databit)
{
    switch (databit) {
    case _7:
        lcd_draw_text(1, (uint8_t*)"7 Data bits     ");
        break;
    default:
        lcd_draw_text(1, (uint8_t*)"8 Data bits     ");
        break;
    }
}
*/

__attribute__ ((section (".subtext"))) static void draw_item_parity(MNU_ITEM_PARITY parity)
{
    switch (parity) {
    case Odd:
        lcd_draw_text(1, (uint8_t*)"Odd Parity      ");
        break;
    case Even:
        lcd_draw_text(1, (uint8_t*)"Even Parity     ");
        break;
    default:
        lcd_draw_text(1, (uint8_t*)"None Parity     ");
        break;
    }
}

__attribute__ ((section (".subtext"))) static void draw_item_stopbit(MNU_ITEM_STOPBIT stopbit)
{
    switch (stopbit) {
    case _2:
        lcd_draw_text(1, (uint8_t*)"2 Stop bits     ");
        break;
    default:
        lcd_draw_text(1, (uint8_t*)"1 Stop bit      ");
        break;
    }
}

__attribute__ ((section (".subtext"))) static void disp_error(void)
{
    switch (g_err) {
    case E_MODBUS_BFNC:
        lcd_draw_text(1, (uint8_t*)"<Illegal Func.> ");
        break;
    case E_MODBUS_BADR:
        lcd_draw_text(1, (uint8_t*)"<Illegal Addr.> ");
        break;
    case E_MODBUS_BDAT:
        lcd_draw_text(1, (uint8_t*)"<Illegal Data>  ");
        break;
    case E_MODBUS_BCRC:
        lcd_draw_text(1, (uint8_t*)"<CRC not match> ");
        break;
    case E_MODBUS_BUNK:
        lcd_draw_text(1, (uint8_t*)"<Unknown Excp.> ");
        break;
    default:
        lcd_draw_text(1, (uint8_t*)"<Comm. Timeout> ");
        break;
    }
    lcd_set_cursor(g_row, 0, true, false);
}

__attribute__ ((section (".subtext"))) static void get_formatted(uint8_t* value)
{
    int i;
    uint8_t buf[32];
    bool_t find = false;
    uint8_t* p = value;

    memset(buf, '0', 32);

    for (i = 0; i < 16; i++) {
        if (!find) {
            if (*p == 0)
                find = true;
        }
        if (find)
            buf[i] = '0';
        else
            buf[i] = *p;
        p++;
    }

    buf[16] = 0;

    strcpy((char*)value, (const char*)buf);
}

__attribute__ ((section (".subtext"))) static void suspend(void)
{
    //stp_adc(&adc);

    wai_sem(SEM_DRW);

    lcd_suspend();
    GPIO_enableInterrupt(
            GPIO_PORT_P2,
            GPIO_PIN0 + GPIO_PIN2 + GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6
    );
    __bis_SR_register(LPM4_bits + GIE);
    GPIO_disableInterrupt(
            GPIO_PORT_P2,
            GPIO_PIN0 + GPIO_PIN2 + GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6
    );
    lcd_resume();

    //if ((m_opt == Setup) || (m_opt == Edit))
    //    sta_adc(&adc);

    sig_sem(SEM_DRW);
}

__attribute__ ((section (".subtext"))) static void setup_serial(void)
{
    uint8_t buf[32];
    ext_sio(&uart);

    switch (m_baudrate) {
//    case _14400:
//        strcpy((char*)buf, (const char*)"14400 ");
//        break;
    case _19200:
        strcpy((char*)buf, (const char*)"19200 ");
        break;
    case _38400:
        strcpy((char*)buf, (const char*)"38400 ");
        break;
//    case _56000:
//        strcpy((char*)buf, (const char*)"56000 ");
//        break;
    case _57600:
        strcpy((char*)buf, (const char*)"57600 ");
        break;
    case _115200:
        strcpy((char*)buf, (const char*)"115200 ");
        break;
    //case _9600:
    default:
        strcpy((char*)buf, (const char*)"9600 ");
        break;
    }

    strcat((char*)buf, (const char*)"B8 ");

    switch (m_parity) {
    case Odd:
        strcat((char*)buf, (const char*)"PO ");
        break;
    case Even:
        strcat((char*)buf, (const char*)"PE ");
        break;
    //case _8:
    default:
        strcat((char*)buf, (const char*)"PN ");
        break;
    }

    switch (m_stopbit) {
    case _2:
        strcat((char*)buf, (const char*)"S2");
        break;
    //case _1:
    default:
        strcat((char*)buf, (const char*)"S1");
        break;
    }

    ini_sio(&uart, (const char*)buf);
    ctl_sio(&uart, TUART_RXE | TUART_TXE | TUART_DTRON | TUART_RTSON);
}

/*
 *  printf()やsprintf()で「%f」や「%g」を使用する場合は
 *  リンカのオプションとして「-u _printf_float」を追記すること
 */
__attribute__ ((section (".subtext"))) void main_task(intptr_t exinf)
{
    int i;
    uint8_t adr[8];
    uint8_t dgt = 0;
    FLGPTN sw = 0x83FF;
    FLGPTN p_flgptn;
    int count_timer = 0;
    bool_t cancel;
    uint16_t top;
    uint8_t val[32];
    MODBUS_FUNC fnc;
    uint16_t crr;
    int num;
    long ltmp;
    uint16_t reg[4];
    float ftmp;
    double dtmp;
    bool_t sus = false;
    uint16_t tmp;
    uint8_t bat[32];
    int count_blink = 0;
    uint8_t id[32];
    int timer;

    main_init();

    modbus_set_mode(false); // RS232C

    for (i = 0; i < MAX_RECORD; i++)
        g_table[i].address = 1;

    sprintf((char*)adr, "%05u", g_table[g_current].address);
    draw_view(adr);
    lcd_set_cursor(0, (2 + dgt), false, true);

    act_tsk(TSK_POL);

    while (1) {
        if (E_TMOUT != twai_flg(FLG_INP, sw, TWF_ORW, &p_flgptn, 100)) {
            count_timer = 0;
            if (cancel) {
                cancel = false;
                continue;
            }
            if (p_flgptn & EVENT_ESC_OFF) {
                //
            } else
            if (p_flgptn & EVENT_ESC_ON) {
                switch (m_opt) {
                case Address:
                    if (get_records() == 0)
                        break;
                    if (g_current == 0) {
                        g_current = get_records() - 1;
                        g_row = 0;
                    } else {
                        g_current--;
                        g_row = 1;
                    }
                    m_opt = View;
                    sprintf((char*)adr, "%05u", g_table[g_current].address);
                    draw_view(adr);
//                    g_row = 0;
                    lcd_set_cursor(g_row, 0, true, false);
                    sig_sem(SEM_POL);
                    break;
                case Value:
                    m_opt = View;
                    sprintf((char*)adr, "%05u", g_table[g_current].address);
                    draw_view(adr);
                    g_row = 1;
                    lcd_set_cursor(g_row, 0, true, false);
                    sig_sem(SEM_POL);
                    break;
                case View:
                case Setup:
                    break;
                //case Edit:
                default:
                    m_opt = Setup;
                    g_row = 1;
                    lcd_set_cursor(g_row, 0, true, false);
                    break;
                }
            } else
            if (p_flgptn & EVENT_SHF_OFF) {
                //
            } else
            if (p_flgptn & EVENT_SHF_ON) {
                switch (m_opt) {
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
                case View:
                    if (g_err == E_TMOUT)
                        rel_wai(TSK_POL);
                    wai_sem(SEM_POL);
                    lcd_set_cursor(0, 0, false, false);
                    g_dsp++;
                    if (g_dsp > Double_Inverse)
                        g_dsp = Signed;
                    switch (g_dsp) {
                    case Unsigned:
                        lcd_draw_text(1, (uint8_t*)"      > Unsigned");
                        break;
                    case Hex:
                        lcd_draw_text(1, (uint8_t*)"           > Hex");
                        break;
                    case Binary:
                        lcd_draw_text(1, (uint8_t*)"        > Binary");
                        break;
                    case Long:
                        lcd_draw_text(1, (uint8_t*)"          > Long");
                        break;
                    case Long_Inverse:
                        lcd_draw_text(1, (uint8_t*)"     > Long Inv.");
                        break;
                    case Float:
                        lcd_draw_text(1, (uint8_t*)"         > Float");
                        break;
                    case Float_Inverse:
                        lcd_draw_text(1, (uint8_t*)"    > Float Inv.");
                        break;
                    case Double:
                        lcd_draw_text(1, (uint8_t*)"        > Double");
                        break;
                    case Double_Inverse:
                        lcd_draw_text(1, (uint8_t*)"   > Double Inv.");
                        break;
                    //case Signed:
                    default:
                        lcd_draw_text(1, (uint8_t*)"        > Signed");
                        break;
                    }
//                    dly_tsk(1000);
                    g_hold = 10;
                    sig_sem(SEM_POL);
                    break;
                case Setup:
                    break;
                //case Edit:
                default:
                    if (m_itm == TargetID) {
                        dgt++;
                        if (dgt == get_digits(Unsigned))
                            dgt = 0;
                        lcd_set_cursor(1, dgt, false, true);
                    }
                    break;
                }
            } else
            if (p_flgptn & EVENT_UP__OFF) {
                //
            } else
            if (p_flgptn & EVENT_UP__ON) {
                switch (m_opt) {
                case Address:
                    adr[dgt] = get_next_drum(Unsigned, adr, dgt, adr[dgt]);
                    draw_view(adr);
                    lcd_set_cursor(0, (2 + dgt), false, true);
                    break;
                case Value:
                    top = g_table[g_current].address / 10000;
                    switch (top) {
                    case 0:
                    case 1:
                        val[dgt] = get_next_drum(Binary, val, dgt, val[dgt]);
                        break;
//                    case 3:
//                    case 4:
                    default:
                        val[dgt] = get_next_drum(g_dsp, val, dgt, val[dgt]);
                        break;
                    }
                    //draw_view(NULL, val);
                    lcd_draw_text(1, (uint8_t*)val);
                    lcd_set_cursor(1, dgt, false, true);
                    break;
                case View:
                    if (g_row == 1) {
                        wai_sem(SEM_DRW);
                        g_row = 0;
                        lcd_set_cursor(g_row, 0, true, false);
                        sig_sem(SEM_DRW);
                    } else {
                        if (g_err == E_TMOUT)
                            rel_wai(TSK_POL);
                        wai_sem(SEM_POL);
                        lcd_clear();
                        if (g_current == 0)
                            g_current = get_records() - 1;
                        else
                            g_current--;
                        sprintf((char*)adr, "%05u", g_table[g_current].address);
                        draw_view(adr);
                        g_row = 1;
                        lcd_set_cursor(g_row, 0, true, false);
                        sig_sem(SEM_POL);
                    }
                    break;
                case Setup:
                    if (g_row == 1) {
                        g_row = 0;
                        lcd_set_cursor(g_row, 0, true, false);
                    } else {
                        if (m_itm == Exit)
                            m_itm = Version;
                        else
                            m_itm--;
                        draw_item(m_itm);
                        g_row = 1;
                        lcd_set_cursor(g_row, 0, true, false);
                    }
                    break;
                //case Edit:
                default:
                    switch (m_itm) {
                    case Timer:
                        if (m_timer_b == _10Min)
                            m_timer_b = Never;
                        else
                            m_timer_b++;
                        draw_item_timer(m_timer_b);
                        lcd_set_cursor(1, 0, false, true);
                        break;
                    case TargetID:
                        val[dgt] = get_next_drum(Unsigned, val, dgt, val[dgt]);
                        lcd_draw_text(1, val);
                        lcd_set_cursor(1, dgt, false, true);
                        break;
                    case Wiring:
                        if (m_wiring_b == RS232C)
                            m_wiring_b = RS485;
                        else
                            m_wiring_b = RS232C;
                        draw_item_wiring(m_wiring_b);
                        lcd_set_cursor(1, 0, false, true);
                        break;
                    case Baudrate:
                        if (m_baudrate_b == _115200)
                            m_baudrate_b = _9600;
                        else
                            m_baudrate_b++;
                        draw_item_baudrate(m_baudrate_b);
                        lcd_set_cursor(1, 0, false, true);
                        break;
/*
                    case DataBit:
                        if (m_databit_b == _8)
                            m_databit_b = _7;
                        else
                            m_databit_b = _8;
                        draw_item_databit(m_databit_b);
                        lcd_set_cursor(1, 0, false, true);
                        break;
*/
                    case Parity:
                        if (m_parity_b == Even)
                            m_parity_b = None;
                        else
                            m_parity_b++;
                        draw_item_parity(m_parity_b);
                        lcd_set_cursor(1, 0, false, true);
                        break;
                    case StopBit:
                        if (m_stopbit_b == _1)
                            m_stopbit_b = _2;
                        else
                            m_stopbit_b = _1;
                        draw_item_stopbit(m_stopbit_b);
                        lcd_set_cursor(1, 0, false, true);
                        break;
                    default:
                        break;
                    }
                    break;
                }
            } else
            if (p_flgptn & EVENT_DWN_OFF) {
                //
            } else
            if (p_flgptn & EVENT_DWN_ON) {
                switch (m_opt) {
                case Address:
                    adr[dgt] = get_back_drum(Unsigned, adr, dgt, adr[dgt]);
                    draw_view(adr);
                    lcd_set_cursor(0, (2 + dgt), false, true);
                    break;
                case Value:
                    top = g_table[g_current].address / 10000;
                    switch (top) {
                    case 0:
                    case 1:
                        val[dgt] = get_back_drum(Binary, val, dgt, val[dgt]);
                        break;
//                    case 3:
//                    case 4:
                    default:
                        val[dgt] = get_back_drum(g_dsp, val, dgt, val[dgt]);
                        break;
                    }
                    //draw_view(NULL, val);
                    lcd_draw_text(1, (uint8_t*)val);
                    lcd_set_cursor(1, dgt, false, true);
                    break;
                case View:
                    if (g_row == 0) {
                        wai_sem(SEM_DRW);
                        g_row = 1;
                        lcd_set_cursor(g_row, 0, true, false);
                        sig_sem(SEM_DRW);
                    } else {
                        if (g_err == E_TMOUT)
                            rel_wai(TSK_POL);
                        wai_sem(SEM_POL);
                        lcd_clear();
                        if (g_current == (MAX_RECORD - 1))
                            g_current = 0;
                        else
                            g_current++;
                        if (!g_table[g_current].registered) {
                            m_opt = Address;
                            dgt = 0;
                            sprintf((char*)adr, "%05u", g_table[g_current].address);
                            draw_view(adr);
                            g_row = 0;
                            lcd_set_cursor(g_row, (2 + dgt), false, true);
                        } else {
                            sprintf((char*)adr, "%05u", g_table[g_current].address);
                            draw_view(adr);
                            g_row = 0;
                            lcd_set_cursor(g_row, 0, true, false);
                            sig_sem(SEM_POL);
                        }
                    }
                    break;
                case Setup:
                    if (g_row == 0) {
                        g_row = 1;
                        lcd_set_cursor(g_row, 0, true, false);
                    } else {
                        if (m_itm == Version)
                            m_itm = Exit;
                        else
                            m_itm++;
                        draw_item(m_itm);
                        g_row = 0;
                        lcd_set_cursor(g_row, 0, true, false);
                    }
                    break;
                //case Edit:
                default:
                    switch (m_itm) {
                    case Timer:
                        if (m_timer_b == Never)
                            m_timer_b = _10Min;
                        else
                            m_timer_b--;
                        draw_item_timer(m_timer_b);
                        lcd_set_cursor(1, 0, false, true);
                        break;
                    case TargetID:
                        val[dgt] = get_back_drum(Unsigned, val, dgt, val[dgt]);
                        lcd_draw_text(1, val);
                        lcd_set_cursor(1, dgt, false, true);
                        break;
                    case Wiring:
                        if (m_wiring_b == RS232C)
                            m_wiring_b = RS485;
                        else
                            m_wiring_b = RS232C;
                        draw_item_wiring(m_wiring_b);
                        lcd_set_cursor(1, 0, false, true);
                        break;
                    case Baudrate:
                        if (m_baudrate_b == _9600)
                            m_baudrate_b = _115200;
                        else
                            m_baudrate_b--;
                        draw_item_baudrate(m_baudrate_b);
                        lcd_set_cursor(1, 0, false, true);
                        break;
/*
                    case DataBit:
                        if (m_databit_b == _8)
                            m_databit_b = _7;
                        else
                            m_databit_b = _8;
                        draw_item_databit(m_databit_b);
                        lcd_set_cursor(1, 0, false, true);
                        break;
*/
                    case Parity:
                        if (m_parity_b == None)
                            m_parity_b = Even;
                        else
                            m_parity_b--;
                        draw_item_parity(m_parity_b);
                        lcd_set_cursor(1, 0, false, true);
                        break;
                    case StopBit:
                        if (m_stopbit_b == _1)
                            m_stopbit_b = _2;
                        else
                            m_stopbit_b = _1;
                        draw_item_stopbit(m_stopbit_b);
                        lcd_set_cursor(1, 0, false, true);
                        break;
                    default:
                        break;
                    }
                    break;
                }
            } else
            if (p_flgptn & EVENT_ENT_OFF) {
                //
            } else
            if (p_flgptn & EVENT_ENT_ON) {
                switch (m_opt) {
                case Address:
                    g_table[g_current].address = strtoul((const char*)adr, NULL, 10);
                    g_table[g_current].registered = true;
                    m_records = get_records();
                    m_opt = View;
                    sprintf((char*)adr, "%05u", g_table[g_current].address);
                    draw_view(adr);
                    lcd_set_cursor(g_row, 0, true, false);
                    sig_sem(SEM_POL);
                    break;
                case Value:
                    top = g_table[g_current].address / 10000;
                    switch (top) {
                    case 0:
                    case 1:
                        if (top == 0) {
                            fnc = COIL_STATUS;
                            crr = 1;
                        } else {
                            fnc = INPUT_STATUS;
                            crr = 10001;
                        }
                        if (val[0] == (uint8_t)'1')
                            g_table[g_current].data[0] = 0xFF01;
                        else
                            g_table[g_current].data[0] = 0xFF00;
                        g_err = modbus_write_status(
                                fnc,
                                g_id,
                                (g_table[g_current].address - crr),
                                1,
                                (uint8_t*)&g_table[g_current].data[0],
                                1000
                                );
                        break;
//                    case 3:
//                    case 4:
                    default:
                        if (top == 3) {
                            fnc = INPUT_REGISTER;
                            crr = 30001;
                        } else {
                            fnc = HOLDING_REGISTER;
                            crr = 40001;
                        }
                        switch (g_dsp) {
                        case Unsigned:
                            g_table[g_current].data[0] = (uint16_t)strtoul((const char*)val, NULL, 10);
                            num = 1;
                            break;
                        case Hex:
                            g_table[g_current].data[0] = (uint16_t)strtoul((const char*)val, NULL, 16);
                            num = 1;
                            break;
                        case Binary:
                            g_table[g_current].data[0] = 0;
                            for (i = 0; i < 16; i++) {
                                if (val[i] == (uint8_t)'1')
                                    g_table[g_current].data[0] += (0x8000 >> i);
                            }
                            num = 1;
                            break;
                        case Long:
                            ltmp = strtol((const char*)val, NULL, 10);
                            memcpy(&g_table[g_current].data[0], &ltmp, sizeof(long));
                            num = 2;
                            break;
                        case Long_Inverse:
                            ltmp = strtol((const char*)val, NULL, 10);
                            memcpy(&reg[0], &ltmp, sizeof(long));
                            g_table[g_current].data[0] = reg[1];
                            g_table[g_current].data[1] = reg[0];
                            num = 2;
                            break;
                        case Float:
                            ftmp = strtof((const char*)val, NULL);
                            memcpy(&g_table[g_current].data[0], &ftmp, sizeof(float));
                            num = 2;
                            break;
                        case Float_Inverse:
                            ftmp = strtof((const char*)val, NULL);
                            memcpy(&reg[0], &ftmp, sizeof(float));
                            g_table[g_current].data[0] = reg[1];
                            g_table[g_current].data[1] = reg[0];
                            num = 2;
                            break;
                        case Double:
                            dtmp = strtod((const char*)val, NULL);
                            memcpy(&g_table[g_current].data[0], &dtmp, sizeof(double));
                            num = 4;
                            break;
                        case Double_Inverse:
                            dtmp = strtod((const char*)val, NULL);
                            memcpy(&reg[0], &dtmp, sizeof(double));
                            g_table[g_current].data[0] = reg[3];
                            g_table[g_current].data[1] = reg[2];
                            g_table[g_current].data[2] = reg[1];
                            g_table[g_current].data[3] = reg[0];
                            num = 4;
                            break;
                        //case Signed:
                        default:
                            g_table[g_current].data[0] = (uint16_t)strtol((const char*)val, NULL, 10);
                            num = 1;
                            break;
                        }
                        g_err = modbus_write_register(
                                fnc,
                                g_id,
                                (g_table[g_current].address - crr),
                                num,
                                &g_table[g_current].data[0],
                                1000
                        );
                        break;
                    }
                    if (g_err != E_OK) {
                        disp_error();
                        dly_tsk(1000);
                    }
                    dly_tsk(10);
                    m_opt = View;
                    sprintf((char*)adr, "%05u", g_table[g_current].address);
                    draw_view(adr);
                    lcd_set_cursor(g_row, 0, true, false);
                    sig_sem(SEM_POL);
                    break;
                case View:
                    if (g_err == E_TMOUT)
                        rel_wai(TSK_POL);
                    wai_sem(SEM_POL);
                    lcd_clear();
                    if (g_row == 0) {
                        m_opt = Address;
                        dgt = 0;
                        sprintf((char*)adr, "%05u", g_table[g_current].address);
                        draw_view(adr);
                        lcd_set_cursor(0, (2 + dgt), false, true);
                    } else {
                        m_opt = Value;
                        dgt = 0;
                        sprintf((char*)adr, "%05u", g_table[g_current].address);
                        top = g_table[g_current].address / 10000;
                        switch (top) {
                        case 0:
                        case 1:
                            sprintf((char*)val, "%u", (g_table[g_current].data[0] & 0x0001U));
                            break;
                //        case 3:
                //        case 4:
                        default:
                            switch (g_dsp) {
                            case Unsigned:
                            case Hex:
                            case Binary:
                            case Long:
                            case Long_Inverse:
                                strcpy((char*)val, (const char*)g_buf);
                                break;
                            case Float:
                            case Float_Inverse:
                            case Double:
                            case Double_Inverse:
                                strcpy((char*)val, (const char*)g_buf);
                                if ((strchr((const char*)val, (int)'e') != NULL) ||
                                    (strchr((const char*)val, (int)'n') != NULL)) {
                                    val[0] = (uint8_t)'+';
                                    for (i = 1; i < 16; i++)
                                        val[i] = (uint8_t)'0';
                                    val[16] = 0;
                                } else {
                                    get_formatted(val);
                                }
                                break;
                            //case Signed:
                            default:
                                strcpy((char*)val, (const char*)g_buf);
                                break;
                            }
                            break;
                        }
                        draw_view(adr);
                        lcd_draw_text(1, (uint8_t*)val);
                        lcd_set_cursor(1, dgt, false, true);
                    }
                    break;
                case Setup:
                    if ((m_itm == Battery) || (m_itm == Version))
                        break;
                    m_opt = Edit;
                    m_timer_b = m_timer;
                    //strcpy((char*)val, (const char*)g_buf);
                    sprintf((char*)val, "%05u", g_id);
                    m_wiring_b = m_wiring;
                    m_baudrate_b = m_baudrate;
                    //m_databit_b = m_databit;
                    m_parity_b = m_parity;
                    m_stopbit_b = m_stopbit;
                    if (m_itm == TargetID) {
                        lcd_clear();
                        dgt = 0;
                        draw_item(m_itm);
                        lcd_draw_text(1, (uint8_t*)val);
                        g_row = 1;
                        lcd_set_cursor(g_row, dgt, false, true);
                    } else {
                        draw_item(m_itm);
                        g_row = 1;
                        lcd_set_cursor(g_row, 0, false, true);
                    }
                    break;
                //case Edit:
                default:
                    if (m_itm == Exit) {
                        //wai_sem(SEM_BAT);
                        stp_adc(&adc);
                        modbus_set_mode((m_wiring == RS485));
                        setup_serial();
                        lcd_clear();
                        if (!g_table[g_current].registered) {
                            m_opt = Address;
                            dgt = 0;
                            sprintf((char*)adr, "%05u", g_table[g_current].address);
                            draw_view(adr);
                            g_row = 0;
                            lcd_set_cursor(g_row, (2 + dgt), false, true);
                        } else {
                            m_opt = View;
                            sprintf((char*)adr, "%05u", g_table[g_current].address);
                            draw_view(adr);
                            g_row = m_row_b;
                            lcd_set_cursor(g_row, 0, true, false);
                            sig_sem(SEM_POL);
                        }
                        break;
                    }
                    if (m_itm == Suspend) {
                        sus = true;
                    }
                    m_opt = Setup;
                    m_timer = m_timer_b;
                    g_id = (uint16_t)strtoul((const char*)val, NULL, 10);
                    m_wiring = m_wiring_b;
                    m_baudrate = m_baudrate_b;
                    //m_databit = m_databit_b;
                    m_parity = m_parity_b;
                    m_stopbit = m_stopbit_b;
                    lcd_set_cursor(g_row, 0, true, false);
                    break;
                }
            } else
            if (p_flgptn & EVENT_MNU) {
                if ((m_opt != Setup) && (m_opt != Edit)) {
                    if (m_opt == View) {
                        if (g_err == E_TMOUT)
                            rel_wai(TSK_POL);
                        wai_sem(SEM_POL);
                    }
                    lcd_clear();
                    lcd_draw_text(0, (uint8_t*)"  <Setup Menu>  ");
                    dly_tsk(1000);
                    m_opt = Setup;
                    m_itm = Exit;
                    draw_item(m_itm);
                    m_row_b = g_row;
                    g_row = 0;
                    lcd_set_cursor(g_row, 0, true, false);
                    sta_adc(&adc);
                    //sig_sem(SEM_BAT);
                } else
                if ((m_opt != Address) && (m_opt != Value) && (m_opt != View)) {
                    //wai_sem(SEM_BAT);
                    stp_adc(&adc);
                    modbus_set_mode((m_wiring == RS485));
                    setup_serial();
                    lcd_clear();
                    if (!g_table[g_current].registered) {
                        m_opt = Address;
                        dgt = 0;
                        sprintf((char*)adr, "%05u", g_table[g_current].address);
                        draw_view(adr);
                        g_row = 0;
                        lcd_set_cursor(g_row, (2 + dgt), false, true);
                    } else {
                        m_opt = View;
                        sprintf((char*)adr, "%05u", g_table[g_current].address);
                        draw_view(adr);
                        g_row = m_row_b;
                        lcd_set_cursor(g_row, 0, true, false);
                        sig_sem(SEM_POL);
                    }
                }
            }
        }

        if (m_opt == View) {
            if (g_err != E_OK) {
                if (count_blink == 0) {
                    wai_sem(SEM_DRW);
                    if (g_hold == 0)
                        disp_error();
                    sig_sem(SEM_DRW);
                } else
                if (count_blink == 5) {
                    wai_sem(SEM_DRW);
                    if (g_hold == 0) {
                        lcd_draw_text(1, (uint8_t*)"                ");
                        lcd_set_cursor(g_row, 0, true, false);
                    }
                    sig_sem(SEM_DRW);
                }
            }
        }

        if (m_opt == Setup) {
            lcd_set_cursor(g_row, 0, false, false);
            switch (m_itm) {
            case Exit:
            case Suspend:
                lcd_draw_text(1, (uint8_t*)">               ");
                break;
            case Timer:
                draw_item_timer(m_timer);
                break;
            case TargetID:
                sprintf((char*)id, "%u", g_id);
                //sprintf((char*)g_buf, "%05u", g_id);
                lcd_draw_text(1, id);
                break;
            case Wiring:
                draw_item_wiring(m_wiring);
                break;
            case Baudrate:
                draw_item_baudrate(m_baudrate);
                break;
//            case DataBit:
//                draw_item_databit(m_databit);
//                break;
            case Parity:
                draw_item_parity(m_parity);
                break;
            case StopBit:
                draw_item_stopbit(m_stopbit);
                break;
            case Battery:
                if (E_OK == get_adc(&adc, &tmp, 10)) {
                    ftmp = (float)tmp * mVperUnit;
                    ftmp *= 2.0;
                    if (ftmp > 2800.0) {
                        tmp = 100;
                    } else
                    if (ftmp < 1800.0) {
                        tmp = 0;
                    } else {
                        ftmp -= 1800.0;
                        ftmp /= 1000;
                        tmp = (uint16_t)(ftmp * 100.0);
                    }
                    sprintf((char*)bat, "%u%%", tmp);
                    lcd_draw_text(1, (uint8_t*)bat);
                }
                break;
            //case Version:
            default:
                lcd_draw_text(1, (uint8_t*)VERSION);
                break;
            }
            lcd_set_cursor(g_row, 0, true, false);
        }

        count_blink++;
        if (count_blink == 10)
            count_blink = 0;
        if ((m_opt != Setup) && (m_opt != Edit)) {
            count_timer++;
            switch (m_timer) {
            case _1Min:
                timer = 600;
                break;
            case _3Min:
                timer = 1800;
                break;
            case _5Min:
                timer = 3000;
                break;
            case _10Min:
                timer = 6000;
                break;
            default:
                timer = -1;
                break;
            }
            if ((timer > 0) && (count_timer == timer)) {
                suspend();
                cancel = true;
            }
        }
        if (sus) {
            sus = false;
            suspend();
            cancel = true;
        }
        if (g_hold > 0)
            g_hold--;
    }
}

__attribute__ ((section (".subtext"))) void poll_task(intptr_t exinf)
{
    uint16_t top;
    MODBUS_FUNC fnc;
    uint16_t crr;
    uint16_t top_b = 0xFFFFU;
    uint8_t buf[32];
    DSP_MODE dsp_b = -1;
    int num;
    int i;
    long ltmp;
    uint16_t reg[4];
    float ftmp;
    double dtmp;

    while (1) {
        wai_sem(SEM_POL);
        top = g_table[g_current].address / 10000;
        switch (top) {
        case 0:
        case 1:
            if (top == 0) {
                fnc = COIL_STATUS;
                crr = 1;
            } else {
                fnc = INPUT_STATUS;
                crr = 10001;
            }
            if (top != top_b) {
                top_b = top;
                sprintf((char*)g_buf, "%u", false);
            }
            g_err = modbus_read_status(
                    fnc,
                    g_id,
                    (g_table[g_current].address - crr),
                    1,
                    (uint8_t*)&g_table[g_current].data[0],
                    1000
                    );
            if (g_err != E_OK)
                break;
            sprintf((char*)buf, "%u", (g_table[g_current].data[0] & 0x0001U));
            sprintf((char*)g_buf, "%u", (g_table[g_current].data[0] & 0x0001U));
            wai_sem(SEM_DRW);
            if (g_hold == 0) {
                lcd_set_cursor(g_row, 0, false, false);
                lcd_draw_text(1, (uint8_t*)buf);
                lcd_set_cursor(g_row, 0, true, false);
            }
            sig_sem(SEM_DRW);
            break;
//        case 3:
//        case 4:
        default:
            if (top == 3) {
                fnc = INPUT_REGISTER;
                crr = 30001;
            } else {
                fnc = HOLDING_REGISTER;
                crr = 40001;
            }
            switch (g_dsp) {
            case Unsigned:
                if ((top != top_b) || (g_dsp != dsp_b)) {
                    top_b = top;
                    dsp_b = g_dsp;
                    sprintf((char*)g_buf, "%05u", 0);
                }
                num = 1;
                break;
            case Hex:
                if ((top != top_b) || (g_dsp != dsp_b)) {
                    top_b = top;
                    dsp_b = g_dsp;
                    sprintf((char*)g_buf, "%04X", 0);
                }
                num = 1;
                break;
            case Binary:
                if ((top != top_b) || (g_dsp != dsp_b)) {
                    top_b = top;
                    dsp_b = g_dsp;
                    for (i = 0; i < 16; i++)
                            g_buf[i] = (uint8_t)'0';
                    g_buf[i] = 0;
                }
                num = 1;
                break;
            case Long:
            case Long_Inverse:
                if ((top != top_b) || (g_dsp != dsp_b)) {
                    top_b = top;
                    dsp_b = g_dsp;
                    sprintf((char*)g_buf, "%+011ld", (long)0);
                }
                num = 2;
                break;
            case Float:
            case Float_Inverse:
                if ((top != top_b) || (g_dsp != dsp_b)) {
                    top_b = top;
                    dsp_b = g_dsp;
                    sprintf((char*)g_buf, "%+f", 0.0);
                }
                num = 2;
                break;
            case Double:
            case Double_Inverse:
                if ((top != top_b) || (g_dsp != dsp_b)) {
                    top_b = top;
                    dsp_b = g_dsp;
                    sprintf((char*)g_buf, "%+f", 0.0);
                }
                num = 4;
                break;
            //case Signed:
            default:
                if ((top != top_b) || (g_dsp != dsp_b)) {
                    top_b = top;
                    dsp_b = g_dsp;
                    sprintf((char*)g_buf, "%+06d", 0);
                }
                num = 1;
                break;
            }
            g_err = modbus_read_register(
                    fnc,
                    g_id,
                    (g_table[g_current].address - crr),
                    num,
                    &g_table[g_current].data[0],
                    1000
                    );
            if (g_err != E_OK)
                break;
            switch (g_dsp) {
            case Unsigned:
                sprintf((char*)buf, "%u", g_table[g_current].data[0]);
                sprintf((char*)g_buf, "%05u", g_table[g_current].data[0]);
                break;
            case Hex:
                sprintf((char*)buf, "%X", g_table[g_current].data[0]);
                sprintf((char*)g_buf, "%04X", g_table[g_current].data[0]);
                break;
            case Binary:
                for (i = 0; i < 16; i++) {
                    if (g_table[g_current].data[0] & (0x8000 >> i)) {
                        g_buf[i] = buf[i] = (uint8_t)'1';
                    } else {
                        g_buf[i] = buf[i] = (uint8_t)'0';
                    }
                }
                g_buf[i] = buf[16] = 0;
                break;
            case Long:
                memcpy(&ltmp, &g_table[g_current].data[0], sizeof(long));
                sprintf((char*)buf, "%ld", ltmp);
                sprintf((char*)g_buf, "%+011ld", ltmp);
               break;
            case Long_Inverse:
                reg[1] = g_table[g_current].data[0];
                reg[0] = g_table[g_current].data[1];
                memcpy(&ltmp, &reg[0], sizeof(long));
                sprintf((char*)buf, "%ld", ltmp);
                sprintf((char*)g_buf, "%+011ld", ltmp);
                break;
            case Float:
                memcpy(&ftmp, &g_table[g_current].data[0], sizeof(float));
                if (snprintf((char*)buf, 17, "%.6f", ftmp) > 15)
                    strcpy((char*)buf, "<Overflowed>    ");
                if (snprintf((char*)g_buf, 17, "%+f", ftmp) > 16)
                    sprintf((char*)g_buf, "%+f", 0.0);
               break;
            case Float_Inverse:
                reg[1] = g_table[g_current].data[0];
                reg[0] = g_table[g_current].data[1];
                memcpy(&ftmp, &reg[0], sizeof(float));
                if (snprintf((char*)buf, 17, "%.6f", ftmp) > 15)
                    strcpy((char*)buf, "<Overflowed>    ");
                if (snprintf((char*)g_buf, 17, "%+f", ftmp) > 16)
                    sprintf((char*)g_buf, "%+f", 0.0);
                break;
            case Double:
                memcpy(&dtmp, &g_table[g_current].data[0], sizeof(double));
                if (snprintf((char*)buf, 17, "%.6f", (float)dtmp) > 15)
                    strcpy((char*)buf, "<Overflowed>    ");
                if (snprintf((char*)g_buf, 17, "%+f", (float)dtmp) > 16)
                    sprintf((char*)g_buf, "%+f", 0.0);
                break;
            case Double_Inverse:
                reg[3] = g_table[g_current].data[0];
                reg[2] = g_table[g_current].data[1];
                reg[1] = g_table[g_current].data[2];
                reg[0] = g_table[g_current].data[3];
                memcpy(&dtmp, &reg[0], sizeof(double));
                if (snprintf((char*)buf, 17, "%.6f", (float)dtmp) > 15)
                    strcpy((char*)buf, "<Overflowed>    ");
                if (snprintf((char*)g_buf, 17, "%+f", (float)dtmp) > 16)
                    sprintf((char*)g_buf, "%+f", 0.0);
                break;
            //case Signed:
            default:
                sprintf((char*)buf, "%d", g_table[g_current].data[0]);
                sprintf((char*)g_buf, "%+06d", g_table[g_current].data[0]);
               break;
            }
            wai_sem(SEM_DRW);
            if (g_hold == 0) {
                lcd_set_cursor(g_row, 0, false, false);
                lcd_draw_text(1, (uint8_t*)buf);
                lcd_set_cursor(g_row, 0, true, false);
            }
            sig_sem(SEM_DRW);
            break;
        }
        sig_sem(SEM_POL);
    }
}
