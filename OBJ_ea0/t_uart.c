//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//
//		MSP430用シリアル通信ドライバ
//		t_uart.c (Ver.00.00.00)
//
//		2023/07/13	Ver.00.00.00	初版
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#include <string.h>

#include <kernel.h>
#include <msp430.h>
#include "driverlib.h"

#include "t_uart.h"

/* 送受信バッファサイズの定義 */

#if 0
#ifndef BUFSZ
#define BUFSZ       1024        /* 受信バッファ長 */
#endif
#ifndef TXBUFSZ
#define TXBUFSZ     BUFSZ       /* 送信バッファ長 */
#endif
#endif

/* 内部変数 */


/*****************************************************************************
* シリアル入出力制御ブロック初期化（内部関数）
*
******************************************************************************/

static void init_buf(UART_HandleTypeDef* uart)
{
	int init;

	/* 制御ブロッククリア */

	init = uart->uartb.flag & TSF_INIT;
	memset(&uart->uartb, 0, sizeof(T_UARTB));
	uart->uartb.flag = (uint8_t)init;

	/* ポインタ類セットアップ */

	uart->uartb.rxbuf = uart->rxbuf;		/* 受信バッファ */
	uart->uartb.rxputp = uart->uartb.rxbuf;	/* 受信バッファ格納ポインタ初期値 */
	uart->uartb.rxgetp = uart->uartb.rxbuf;	/* 受信バッファ取得ポインタ初期値 */
}

/*****************************************************************************
* ステータス入力 & 編集（内部関数）
*
* DSR, CD は 無いので常時 1 とする
******************************************************************************/

static int get_stat(UART_HandleTypeDef* uart)
{
	int stat;
	int sr;

	/* ステータス入力 */

	sr = USCI_A_UART_queryStatusFlags(uart->reg, (UCBRK|UCOE|UCFE|UCPE));
	sr |= (uart->uartb.rxsts & (UCPE|UCFE));	/* 受信時のステータスとOR */

	/* ステータスビット編集 */

	stat = TUART_DSR|TUART_CD;  /* DSR, CD 常時ON */

	if (sr & UCPE)			    /* パリティエラー */
		stat |= TUART_PE;
	if (sr & UCOE)			    /* オーバーランエラー */
		stat |= TUART_OE;
	if (sr & UCFE)			    /* フレーミングエラー */
		stat |= TUART_FE;
	if (sr & UCBRK)
		stat |= TUART_BD;

	return stat;
}

/*****************************************************************************
* ＳＩＯデバイス初期化（内部関数）
*
******************************************************************************/
static int init_sio(UART_HandleTypeDef* uart, uint8_t parity, uint8_t numberofStopBits, uint32_t baud)
{
    // From Table 36-4 in the family user's manual where UCOS16 = 0 and
    //            baudrate = 9600
    //            clock freq = 4MHz
    // UCBRx = 416, UCBRFx = 6, UCBRSx = 0, UCOS16 = 0
    // http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
    USCI_A_UART_initParam param = {0};

    param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
    switch (baud) {
    case 19200:
        param.clockPrescalar = 208; // UCBRx
        param.firstModReg = 3;      // UCBRFx
        param.secondModReg = 0;     // UCBRSx
        break;
    case 38400:
        param.clockPrescalar = 104; // UCBRx
        param.firstModReg = 1;      // UCBRFx
        param.secondModReg = 0;     // UCBRSx
        break;
    case 57600:
        param.clockPrescalar = 69;  // UCBRx
        param.firstModReg = 4;      // UCBRFx
        param.secondModReg = 0;     // UCBRSx
        break;
    case 115200:
        param.clockPrescalar = 34;  // UCBRx
        param.firstModReg = 6;      // UCBRFx
        param.secondModReg = 0;     // UCBRSx
        break;
    default:
        param.clockPrescalar = 416; // UCBRx
        param.firstModReg = 6;      // UCBRFx
        param.secondModReg = 0;     // UCBRSx
        break;
    }

    param.parity = parity;
    param.msborLsbFirst = USCI_A_UART_LSB_FIRST;
    param.numberofStopBits = numberofStopBits;
    param.uartMode = USCI_A_UART_MODE;
    param.overSampling = USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;     // UCOS16 = 0

    USCI_A_UART_init(uart->reg, &param);

    //Enable UART module for operation
    USCI_A_UART_enable(uart->reg);

	return true;
}

/*****************************************************************************
* シリアル割込み禁止（内部関数）
*
******************************************************************************/

static void dis_int_sio(UART_HandleTypeDef* uart)
{
    USCI_A_UART_disableInterrupt(
            uart->reg,
            USCI_A_UART_TRANSMIT_INTERRUPT
    );
    USCI_A_UART_disableInterrupt(
            uart->reg,
            USCI_A_UART_RECEIVE_INTERRUPT
    );
    USCI_A_UART_disableInterrupt(
            uart->reg,
            USCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT
    );
}

/*****************************************************************************
* シリアル割込み許可（内部関数）
*
******************************************************************************/

static void ena_int_sio(UART_HandleTypeDef* uart)
{
    USCI_A_UART_enableInterrupt(
            uart->reg,
            USCI_A_UART_TRANSMIT_INTERRUPT
    );
    USCI_A_UART_enableInterrupt(
            uart->reg,
            USCI_A_UART_RECEIVE_INTERRUPT
    );
    USCI_A_UART_enableInterrupt(
            uart->reg,
            USCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT
    );
}

/*****************************************************************************
* 受信バッファクリア（内部関数）
*
******************************************************************************/
static void clr_rxbuf(UART_HandleTypeDef* uart)
{
	/* 受信バッファクリア */

	uart->uartb.rxsts = 0;
	uart->uartb.oldsts = 0;
	uart->uartb.eotcnt = 0;
	uart->uartb.rxcnt = 0;
	uart->uartb.rxgetp = uart->uartb.rxputp;
}

/*****************************************************************************
* 受信バッファへ受信文字/ステータス格納（内部関数）
*
******************************************************************************/

static void put_rxbuf(UART_HandleTypeDef* uart)
{
	int cnt;
	uint8_t *p;

	/* 連続ブレークなら格納しない */

	if ((uart->uartb.rxsts & UCBRK) && (uart->uartb.oldsts & UCBRK))
		return;

	/* バッファ満杯チェック */

	cnt = uart->uartb.rxcnt;
	if (cnt == BUFSZ)
		return;
	if (++cnt == BUFSZ)
		uart->uartb.rxsts |= UCBUSY;	/* オーバフローは UCBUSY に割り当て */

	/* バッファ内文字数 + 1 */

	uart->uartb.rxcnt = (uint16_t)cnt;

	/* バッファへ格納 */
	/* 終端文字検出+1 */

	p = uart->uartb.rxputp;
	if ((*p = uart->uartb.rxchr) == uart->uartb.eot)
		uart->uartb.eotcnt++;
	*(p + BUFSZ) = uart->uartb.rxsts;

	/* 格納ポインタを１つ進める */

	if (++p >= uart->uartb.rxbuf + BUFSZ)
		p = uart->uartb.rxbuf;
	uart->uartb.rxputp = p;
}

/*****************************************************************************
* 受信バッファから１文字取得（内部関数）
*
* バッファ空で取得できなかった場合は、-1 を返す。
******************************************************************************/

static int get_rxbuf(UART_HandleTypeDef* uart, uint8_t *c)
{
	int cnt;
	int sts;
	uint8_t *p;

	/* 受信バッファ空チェック */

	cnt = uart->uartb.rxcnt;
	if (--cnt == -1)
		return cnt;

	/* 受信バッファ内文字数 - 1 */

	uart->uartb.rxcnt = (uint16_t)cnt;


	/* 受信バッファから取得 */
	/* 終端文字検出数-1 */

	p = uart->uartb.rxgetp;
	if ((*c = *p) == uart->uartb.eot)
		uart->uartb.eotcnt--;
	sts = *(p + BUFSZ);

	/* 取得ポインタを１つ進める */

	if (++p >= uart->uartb.rxbuf + BUFSZ)
		p = uart->uartb.rxbuf;
	uart->uartb.rxgetp = p;

	return sts;
}

/*****************************************************************************
* 受信割込みハンドラ本体（内部関数）
*
******************************************************************************/

static void rx_int(UART_HandleTypeDef* uart)
{
	int sts;
	int chr;
	int tid;

	/* 受信ステータスと受信文字を入力 */

	sts = USCI_A_UART_queryStatusFlags(uart->reg, (UCBRK|UCOE|UCFE|UCPE));
	chr = USCI_A_UART_receiveData(uart->reg);

	uart->uartb.oldsts = uart->uartb.rxsts;		/* 前回の受信ステータス記憶 */
	uart->uartb.rxsts = (uint8_t)sts;
	uart->uartb.rxchr = (uint8_t)chr;

	/* 受信バッファへ格納 */

	put_rxbuf(uart);

	/* 受信待ち解除 */

	if ((tid = uart->uartb.rxtid) != 0)
	{
		uart->uartb.rxtid = 0;
		iwup_tsk((ID)tid);
	}
}

/*****************************************************************************
* 割込みハンドラ本体
*
******************************************************************************/

void uart_isr(UART_HandleTypeDef* uart)
{
    ID tid;

    if (USCI_A_UART_getInterruptStatus(uart->reg, USCI_A_UART_RECEIVE_INTERRUPT_FLAG)) {
        rx_int(uart);
    } else
    if (USCI_A_UART_getInterruptStatus(uart->reg, USCI_A_UART_TRANSMIT_INTERRUPT_FLAG)) {
        USCI_A_UART_clearInterrupt(uart->reg, USCI_A_UART_TRANSMIT_INTERRUPT_FLAG);
        if ((tid = uart->uartb.txtid) != 0) {
            uart->uartb.txtid = 0;
            iwup_tsk((ID)tid);
        }
    }
}

/*****************************************************************************
* 初期化パラメータ解析（内部関数）
*
******************************************************************************/

static int set_param(const char *s, uint8_t* pParity, uint8_t* pNumberofStopBits, uint32_t *baud)
{
	char c;
	uint32_t b;

	/* モード判別 */

	*pParity = USCI_A_UART_NO_PARITY;
	*pNumberofStopBits = USCI_A_UART_ONE_STOP_BIT;
	if (strstr(s, (char *)"PE") != NULL) {
		*pParity = USCI_A_UART_EVEN_PARITY;
	} else
	if (strstr(s, (char *)"PO") != NULL) {
        *pParity = USCI_A_UART_ODD_PARITY;
	}
	if (strstr(s, (char *)"S2") != NULL) {
		*pNumberofStopBits = USCI_A_UART_TWO_STOP_BITS;
	}

	/* ボーレート判別 */
	/* 3桁以上の数値検索して変換 */

	for (;;)
	{
		c = *s++;
		if (c == '\0')
		{
			*baud = 9600L;
			break;
		}
		if (c < '0' || c > '9')
			continue;
		c = *s++;
		if (c < '0' || c > '9')
			continue;
		c = *s++;
		if (c < '0' || c > '9')
			continue;

		s -= 3;
		b = 0L;
		for (;;)
		{
			c = *s++;
			if (c < '0' || c > '9')
				break;
			b = b * 10 + (c - '0');
		}
		*baud = b;
		break;
	}
	return true;
}

/*****************************************************************************
* シリアル入出力初期化
*
******************************************************************************/

ER ini_sio(UART_HandleTypeDef* uart, const char *param)
{
	uint32_t baud;
	uint8_t parity;
	uint8_t numberofStopBits;

	dis_int_sio(uart);

	/* 制御ブロック初期化 */

	init_buf(uart);

	/* パラメータ解析 */

	if (!set_param(param, &parity, &numberofStopBits, &baud))
		return E_PAR;

	/* デバイス初期化 */

	if (!init_sio(uart, parity, numberofStopBits, baud))
		return E_PAR;

	uart->uartb.flag |= TSF_INIT;

	/* 割込み許可 */
	ena_int_sio(uart);

	return E_OK;
}

/*****************************************************************************
* シリアル入出力終了
*
******************************************************************************/

void ext_sio(UART_HandleTypeDef* uart)
{
	if (!(uart->uartb.flag & TSF_INIT))	/* 未初期化なら何もしない */
		return;
	dis_int_sio(uart);					/* シリアル割込み禁止 */

	uart->uartb.flag &= ~TSF_INIT;		/* 初期化済みフラグクリア */
}

/*****************************************************************************
* シリアル１文字入力
*
******************************************************************************/

ER get_sio(UART_HandleTypeDef* uart, uint8_t *c, TMO tmout)
{
	ER ercd;
	int sts;
	ER_UINT wupcnt;

	for (;;)
	{
		/* 受信バッファから１文字得る */

		sts = get_rxbuf(uart, c);

		if (sts != -1)              /* 受信文字あった場合 */
		{
		    Asm("nop");
		    Asm("dint");
		    Asm("nop");
			/* 受信エラー判別 */

			if (sts & (UCBUSY|UCBRK|UCOE|UCFE|UCPE))
			{
				if (sts & UCBUSY)			    /* オーバフローはUCBUSYに割り当て */
				ercd = EV_SIOOVF;
				else if (sts & UCOE)
				ercd = EV_SIOORN;
				else if (sts & UCBRK)
				ercd = EV_SIOBRK;
				else if (sts & UCFE)
				ercd = EV_SIOFRM;
				else /* FSR_PER */
				ercd = EV_SIOPTY;
			}
			else
				ercd = E_OK;
			return ercd;
		}

		/* 受信割込み待ち */
		get_tid(&uart->uartb.rxtid);
		Asm("nop");
		Asm("eint");
		Asm("nop");
		ercd = tslp_tsk(tmout);
		uart->uartb.rxtid = 0;
		// 複数回 iwup_tsk された場合の対策
		do {
			wupcnt = can_wup(TSK_SELF);
		} while (wupcnt);
		if (ercd)
			return ercd;    /* タイムアウト終了 */
	}
}

/*****************************************************************************
* シリアル１文字出力
*
******************************************************************************/

ER put_sio(UART_HandleTypeDef* uart, uint8_t c, TMO tmout)
{
	ER ercd;
	ER_UINT wupcnt;

    Asm("nop");
    Asm("dint");
    Asm("nop");
	USCI_A_UART_transmitData(uart->reg, c);
    get_tid(&uart->uartb.txtid);
    Asm("nop");
    Asm("eint");
    Asm("nop");
    ercd = tslp_tsk(tmout);
    uart->uartb.txtid = 0;
    // 複数回 iwup_tsk された場合の対策
    do {
        wupcnt = can_wup(TSK_SELF);
    } while (wupcnt);
    if (ercd)
        return ercd;    /* タイムアウト終了 */

    return E_OK;
}

/*****************************************************************************
* シリアル入出力制御
*
******************************************************************************/

ER ctl_sio(UART_HandleTypeDef* uart, uint16_t fncd)
{
    Asm("nop");
    Asm("dint");
    Asm("nop");

	/* バッファクリア */

	if (fncd & TUART_RXCLR)
		clr_rxbuf(uart);

    Asm("nop");
    Asm("eint");
    Asm("nop");

    return E_OK;
}

/*****************************************************************************
* シリアル入出力状態参照
*
******************************************************************************/

ER ref_sio(UART_HandleTypeDef* uart, T_UARTS *pk_sios)
{
	int stat;

    Asm("nop");
    Asm("dint");
    Asm("nop");

	stat = get_stat(uart);

	pk_sios->siostat = (uint8_t)stat;
	pk_sios->rxlen   = uart->uartb.rxcnt;
	pk_sios->eotcnt  = uart->uartb.eotcnt;

    Asm("nop");
    Asm("eint");
    Asm("nop");

    return E_OK;
}
