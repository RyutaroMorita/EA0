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

#include <kernel.h>
#include <msp430.h>
#include "kernel_cfg.h"
#include "driverlib.h"
#include "nosio.h"

#include "main.h"


#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

extern void target_fput_log(char c);

NOSIO_HandleTypeDef usci_a1;

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

void nosio_isr(intptr_t exinf)
{
    NOSIO_IRQHandler(&usci_a1);
}

static void main_init(void)
{
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P4,
            GPIO_PIN4+GPIO_PIN5
    );

    usci_a1.reg = (uint32_t)USCI_A1_BASE;
    ini_sio(&usci_a1, "9600 B8 PN S1");
    ctl_sio(&usci_a1, TSIO_RXE | TSIO_TXE | TSIO_DTRON | TSIO_RTSON);
}
/*
 *  printf()やsprintf()で「%f」や「%g」を使用する場合は
 *  リンカのオプションとして「-u _printf_float」を追記すること
 */
void main_task(intptr_t exinf)
{
    uint8_t buf[16];
//    uint32_t tmp = 123456789U;
    float tmp = 1.23456;
    int count = 0;
    uint8_t* p;
    uint8_t c;
    int i;

    main_init();

    //printf("EA0 start.\r\n", tmp);

    strcpy(buf, "TEST MESSAGE\r\n");

    while (1) {
#if 0
//        printf("%d - Test Message\r\n", count);
//        printf("%ld - Test Message\r\n", tmp);
        printf("%g - Test Message\r\n", tmp);
        tmp += 0.00001;
        //sprintf(buf, "%d\r\n", count);
        //printf(buf);
        count++;
        dly_tsk(1000);
#endif

        if (E_OK == get_sio(&usci_a1, &c, 1000)) {
            put_sio(&usci_a1, c, 10);
        }

        p = &buf[0];
        for (i = 0; i < strlen(buf); i++) {
            put_sio(&usci_a1, *p, 1000);
            p++;
        }
//        dly_tsk(1000);

    }
}
