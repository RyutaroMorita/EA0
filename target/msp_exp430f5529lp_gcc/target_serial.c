/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2011 by Embedded and Real-Time Systems Laboratory
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
 *	2022/10/12 Ryutaro Morita
 */

/*
 *		シリアルI/Oデバイス（SIO）ドライバ（msp-exp430f5529lp用）
 */

#include <kernel.h>
#include <t_syslog.h>
#include "driverlib.h"
#include "target_syssvc.h"
#include "target_serial.h"

/*
 * レジスタ設定値
 */
#define INDEX_PORT(x)   ((x) - 1)
#define GET_SIOPCB(x)   (&siopcb_table[INDEX_PORT(x)])

/*
 *  シリアルポートの管理ブロック
 */
struct sio_port_control_block {
    ID port;
    uint16_t reg;
    intptr_t exinf;
};

/*
 * シリアルI/Oポート管理ブロックエリア
 */
SIOPCB siopcb_table[TNUM_PORT];

static const uint32_t sioreg_table[TNUM_PORT] = {
    USCI_A1_BASE,
};

/*
 *  SIOドライバの初期化
 */
void
sio_initialize(intptr_t exinf)
{
    int i;

    for (i = 0; i < TNUM_PORT; i++) {
        siopcb_table[i].port = i;
        siopcb_table[i].reg = sioreg_table[i];
        siopcb_table[i].exinf = 0;
    }
}

/*
 *  シリアルI/Oポートのオープン
 */
SIOPCB *
sio_opn_por(ID siopid, intptr_t exinf)
{
	SIOPCB	*siopcb;
//	bool_t	opnflg;
//	ER		ercd;

    if (siopid > TNUM_PORT) {
        return NULL;
    }

    siopcb = GET_SIOPCB(siopid);
    siopcb->exinf = exinf;

/*
    USCI_A_UART_disable(siopcb->reg);

    // From Table 36-4 in the family user's manual where UCOS16 = 0 and
    //            baudrate = 9600
    //            clock freq = 4MHz
    // UCBRx = 416, UCBRFx = 6, UCBRSx = 0, UCOS16 = 0
    // http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
    USCI_A_UART_initParam param = {0};
    param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 416;                                             // UCBRx
    param.firstModReg = 6;                                                  // UCBRFx
    param.secondModReg = 0;                                                 // UCBRSx
    param.parity = USCI_A_UART_NO_PARITY;
    param.msborLsbFirst = USCI_A_UART_LSB_FIRST;
    param.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
    param.uartMode = USCI_A_UART_MODE;
    param.overSampling = USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;     // UCOS16 = 0

    USCI_A_UART_init(siopcb->reg, &param);

    //Enable UART module for operation
    USCI_A_UART_enable(siopcb->reg);

    //Disable Receive Interrupt
    USCI_A_UART_clearInterrupt(
            siopcb->reg,
            USCI_A_UART_TRANSMIT_INTERRUPT
    );
    USCI_A_UART_clearInterrupt(
            siopcb->reg,
            USCI_A_UART_RECEIVE_INTERRUPT
    );
*/
/*
    USCI_A_UART_disableInterrupt(
            siopcb->reg,
            USCI_A_UART_TRANSMIT_INTERRUPT
    );
    USCI_A_UART_disableInterrupt(
            siopcb->reg,
            USCI_A_UART_RECEIVE_INTERRUPT
    );
*/
    return(siopcb);
}

/*
 *  シリアルI/Oポートのクローズ
 */
void
sio_cls_por(SIOPCB *p_siopcb)
{
    //Disable Receive Interrupt
    USCI_A_UART_disableInterrupt(
            p_siopcb->reg,
            USCI_A_UART_TRANSMIT_INTERRUPT
    );
    USCI_A_UART_disableInterrupt(
            p_siopcb->reg,
            USCI_A_UART_RECEIVE_INTERRUPT
    );
    USCI_A_UART_disable(p_siopcb->reg);
}

/*
 *  SIOの割込みサービスルーチン
 */
void
sio_isr(intptr_t exinf)
{
    SIOPCB* siopcb = GET_SIOPCB(exinf);

    if (USCI_A_UART_getInterruptStatus(siopcb->reg, USCI_A_UART_RECEIVE_INTERRUPT_FLAG)) {
        sio_irdy_rcv(siopcb->exinf);
    } else {
        USCI_A_UART_clearInterrupt(siopcb->reg, USCI_A_UART_TRANSMIT_INTERRUPT_FLAG);
        sio_irdy_snd(siopcb->exinf);
    }
}

/*
 *  シリアルI/Oポートへの文字送信
 */
bool_t
sio_snd_chr(SIOPCB *siopcb, char c)
{
    USCI_A_UART_transmitData(siopcb->reg, c);
	return true;
}

/*
 *  シリアルI/Oポートからの文字受信
 */
int_t
sio_rcv_chr(SIOPCB *siopcb)
{
    int_t c = -1;

    c = USCI_A_UART_receiveData(siopcb->reg);

	return c;
}

/*
 *  シリアルI/Oポートからのコールバックの許可
 */
void
sio_ena_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
    switch (cbrtn) {
    case SIO_RDY_SND:
        USCI_A_UART_enableInterrupt(
                siopcb->reg,
                USCI_A_UART_TRANSMIT_INTERRUPT
        );
        break;
    case SIO_RDY_RCV:
        USCI_A_UART_enableInterrupt(
                siopcb->reg,
                USCI_A_UART_RECEIVE_INTERRUPT
        );
        break;
    default:
        break;
    }
}

/*
 *  シリアルI/Oポートからのコールバックの禁止
 */
void
sio_dis_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
    switch (cbrtn) {
    case SIO_RDY_SND:
        USCI_A_UART_disableInterrupt(
                siopcb->reg,
                USCI_A_UART_TRANSMIT_INTERRUPT
        );
        break;
    case SIO_RDY_RCV:
        USCI_A_UART_disableInterrupt(
                siopcb->reg,
                USCI_A_UART_RECEIVE_INTERRUPT
        );
        break;
    default:
        break;
    }
}
