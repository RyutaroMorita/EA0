//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//
//		MSP430用シリアル通信ドライバ
//		nosio.c (Ver.00.00.00)
//
//		2023/07/13	Ver.00.00.00	初版
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#include <string.h>

#include <kernel.h>
#include <msp430.h>
#include <sil.h>

 #ifdef TOPPERS_ASP
  #define PSW         PRI
 #else
  #define PSW         UINT
 #endif

#include "driverlib.h"
#include "nosio.h"

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

static bool_t tx_ready = false;

/*****************************************************************************
* シリアル入出力制御ブロック初期化（内部関数）
*
******************************************************************************/

static void init_buf(NOSIO_HandleTypeDef* nosio)
{
	int init;

	/* 制御ブロッククリア */

	init = nosio->sio.flag & TSF_INIT;
	memset(&nosio->sio, 0, sizeof(T_SIO));
	nosio->sio.flag = (uint8_t)init;

	/* ポインタ類セットアップ */

	nosio->sio.txbuf = nosio->txbuf;		/* 送信バッファ */
	nosio->sio.rxbuf = nosio->rxbuf;		/* 受信バッファ */
	nosio->sio.txputp = nosio->sio.txbuf;	/* 送信バッファ格納ポインタ初期値 */
	nosio->sio.txgetp = nosio->sio.txbuf;	/* 送信バッファ取得ポインタ初期値 */
	nosio->sio.rxputp = nosio->sio.rxbuf;	/* 受信バッファ格納ポインタ初期値 */
	nosio->sio.rxgetp = nosio->sio.rxbuf;	/* 受信バッファ取得ポインタ初期値 */
}

/*****************************************************************************
* ステータス入力 & 編集（内部関数）
*
* DSR, CD は 無いので常時 1 とする
******************************************************************************/

static int get_stat(NOSIO_HandleTypeDef* nosio)
{
	int stat;
	int sr;

	/* ステータス入力 */

	sr = USCI_A_UART_queryStatusFlags(nosio->reg, (UCBRK|UCOE|UCFE|UCPE));
	sr |= (nosio->sio.rxsts & (UCPE|UCFE));	/* 受信時のステータスとOR */

	/* ステータスビット編集 */

	stat = TSIO_DSR|TSIO_CD;		/* DSR, CD 常時ON */

	if (sr & UCPE)			        /* パリティエラー */
		stat |= TSIO_PE;
	if (sr & UCOE)			        /* オーバーランエラー */
		stat |= TSIO_OE;
	if (sr & UCFE)			        /* フレーミングエラー */
		stat |= TSIO_FE;
	if (sr & UCBRK)
		stat |= TSIO_BD;

	return stat;
}

/*****************************************************************************
* ＳＩＯデバイス初期化（内部関数）
*
******************************************************************************/
static int init_sio(NOSIO_HandleTypeDef* nosio, uint8_t parity, uint8_t numberofStopBits, uint32_t baud)
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

    USCI_A_UART_init(nosio->reg, &param);

    //Enable UART module for operation
    USCI_A_UART_enable(nosio->reg);

	return true;
}

/*****************************************************************************
* シリアル割込み禁止（内部関数）
*
******************************************************************************/

static void dis_int_sio(NOSIO_HandleTypeDef* nosio)
{
    USCI_A_UART_disableInterrupt(
            nosio->reg,
            USCI_A_UART_TRANSMIT_INTERRUPT
    );
    USCI_A_UART_disableInterrupt(
            nosio->reg,
            USCI_A_UART_RECEIVE_INTERRUPT
    );
}

/*****************************************************************************
* シリアル割込み許可（内部関数）
*
******************************************************************************/

static void ena_int_sio(NOSIO_HandleTypeDef* nosio)
{
    USCI_A_UART_enableInterrupt(
            nosio->reg,
            USCI_A_UART_TRANSMIT_INTERRUPT
    );
    USCI_A_UART_enableInterrupt(
            nosio->reg,
            USCI_A_UART_RECEIVE_INTERRUPT
    );
}

/*****************************************************************************
* 送信バッファクリア（内部関数）
*
******************************************************************************/

static void clr_txbuf(NOSIO_HandleTypeDef* nosio)
{
	/* 送信バッファクリア */

	nosio->sio.txcnt = 0;
	nosio->sio.txgetp = nosio->sio.txputp;
}

/*****************************************************************************
* 受信バッファクリア（内部関数）
*
******************************************************************************/
static void clr_rxbuf(NOSIO_HandleTypeDef* nosio)
{
	/* 受信バッファクリア */

	nosio->sio.rxsts = 0;
	nosio->sio.oldsts = 0;
	nosio->sio.eotcnt = 0;
	nosio->sio.rxcnt = 0;
	nosio->sio.rxgetp = nosio->sio.rxputp;
}

/*****************************************************************************
* 送信バッファへ１文字格納（内部関数）
*
* バッファ満杯で格納できなかった場合は、FALSE を返す。
******************************************************************************/
static void tx_ctx(NOSIO_HandleTypeDef* nosio);

static int put_txbuf(NOSIO_HandleTypeDef* nosio, uint8_t c)
{
	uint8_t *p;

	/* バッファ内文字数 + 1 */
	/* バッファ満杯チェック */

	if (++nosio->sio.txcnt > TXBUFSZ)
	{
		nosio->sio.txcnt = TXBUFSZ;
		return false;
	}

	/* バッファへ格納 */

	p = nosio->sio.txputp;
	*p = c;

	/* 格納ポインタを１つ進める */

	if (++p >= nosio->sio.txbuf + TXBUFSZ)
		p = nosio->sio.txbuf;
	nosio->sio.txputp = p;

	/* 送信割込み許可 */
    USCI_A_UART_enableInterrupt(
            nosio->reg,
            USCI_A_UART_TRANSMIT_INTERRUPT
    );

    if (tx_ready) {
        tx_ready = false;
        tx_ctx(nosio);
    }

	return true;
}

/*****************************************************************************
* 受信バッファへ受信文字/ステータス格納（内部関数）
*
******************************************************************************/

static void put_rxbuf(NOSIO_HandleTypeDef* nosio)
{
	int cnt;
	uint8_t *p;

	/* 連続ブレークなら格納しない */

	if ((nosio->sio.rxsts & UCBRK) && (nosio->sio.oldsts & UCBRK))
		return;

	/* バッファ満杯チェック */

	cnt = nosio->sio.rxcnt;
	if (cnt == BUFSZ)
		return;
	if (++cnt == BUFSZ)
		nosio->sio.rxsts |= UCBUSY;	/* オーバフローは UCBUSY に割り当て */

	/* バッファ内文字数 + 1 */

	nosio->sio.rxcnt = (uint16_t)cnt;

	/* バッファへ格納 */
	/* 終端文字検出+1 */

	p = nosio->sio.rxputp;
	if ((*p = nosio->sio.rxchr) == nosio->sio.eot)
		nosio->sio.eotcnt++;
	*(p + BUFSZ) = nosio->sio.rxsts;

	/* 格納ポインタを１つ進める */

	if (++p >= nosio->sio.rxbuf + BUFSZ)
		p = nosio->sio.rxbuf;
	nosio->sio.rxputp = p;
}

/*****************************************************************************
* 受信バッファから１文字取得（内部関数）
*
* バッファ空で取得できなかった場合は、-1 を返す。
******************************************************************************/

static int get_rxbuf(NOSIO_HandleTypeDef* nosio, uint8_t *c)
{
	int cnt;
	int sts;
	uint8_t *p;

	/* 受信バッファ空チェック */

	cnt = nosio->sio.rxcnt;
	if (--cnt == -1)
		return cnt;

	/* 受信バッファ内文字数 - 1 */

	nosio->sio.rxcnt = (uint16_t)cnt;


	/* 受信バッファから取得 */
	/* 終端文字検出数-1 */

	p = nosio->sio.rxgetp;
	if ((*c = *p) == nosio->sio.eot)
		nosio->sio.eotcnt--;
	sts = *(p + BUFSZ);

	/* 取得ポインタを１つ進める */

	if (++p >= nosio->sio.rxbuf + BUFSZ)
		p = nosio->sio.rxbuf;
	nosio->sio.rxgetp = p;

	return sts;
}

/*****************************************************************************
* 受信割込みハンドラ本体（内部関数）
*
******************************************************************************/

static void rx_int(NOSIO_HandleTypeDef* nosio)
{
	int sts;
	int chr;
	int tid;

	/* 受信ステータスと受信文字を入力 */

	sts = USCI_A_UART_queryStatusFlags(nosio->reg, (UCBRK|UCOE|UCFE|UCPE));
	chr = USCI_A_UART_receiveData(nosio->reg);

	nosio->sio.oldsts = nosio->sio.rxsts;		/* 前回の受信ステータス記憶 */
	nosio->sio.rxsts = (uint8_t)sts;
	nosio->sio.rxchr = (uint8_t)chr;

	/* 受信バッファへ格納 */

	put_rxbuf(nosio);

	/* 受信待ち解除 */

	if ((tid = nosio->sio.rxtid) != 0)
	{
		nosio->sio.rxtid = 0;
		iwup_tsk((ID)tid);
	}
}

/*****************************************************************************
* 送信割込みハンドラ本体（内部関数）
*
******************************************************************************/

static void tx_int(NOSIO_HandleTypeDef* nosio)
{
	uint8_t *p;
	int tid;

	/* 送信バッファ内文字数 - 1 */
	/* 送信バッファ空なら送信不可 */

	if (--nosio->sio.txcnt == (uint16_t)-1)
	{
		nosio->sio.txcnt = 0;
	    /* 送信終了待ち解除 */
	    if ( nosio->sio.tetid != 0 )
	        iwup_tsk(nosio->sio.tetid);
		/* 送信割込み禁止 */
        USCI_A_UART_disableInterrupt(
                nosio->reg,
                USCI_A_UART_TRANSMIT_INTERRUPT
        );
        tx_ready = true;
		return;
	}

	/* １文字送信 */

	p = nosio->sio.txgetp;
	USCI_A_UART_transmitData(nosio->reg, *p);

	/* 取得ポインタを１つ進める */

	if (++p >= nosio->sio.txbuf + TXBUFSZ)
		p = nosio->sio.txbuf;
	nosio->sio.txgetp = p;

	/* 送信待ち解除 */

	if ((tid = nosio->sio.txtid) != 0)
	{
		nosio->sio.txtid = 0;
		iwup_tsk((ID)tid);
	}
}

static void tx_ctx(NOSIO_HandleTypeDef* nosio)
{
    uint8_t *p;

    /* １文字送信 */

    p = nosio->sio.txgetp;
    USCI_A_UART_transmitData(nosio->reg, *p);

    /* 取得ポインタを１つ進める */

    if (++p >= nosio->sio.txbuf + TXBUFSZ)
        p = nosio->sio.txbuf;
    nosio->sio.txgetp = p;
}

/*****************************************************************************
* 割込みハンドラ本体
*
******************************************************************************/

//void nosio_isr(intptr_t exinf)
void NOSIO_IRQHandler(NOSIO_HandleTypeDef* nosio)
{
    ID tid;

    if (USCI_A_UART_getInterruptStatus(nosio->reg, USCI_A_UART_RECEIVE_INTERRUPT_FLAG)) {
        rx_int(nosio);
    } else {
        USCI_A_UART_clearInterrupt(nosio->reg, USCI_A_UART_TRANSMIT_INTERRUPT_FLAG);
        //tx_int(nosio);
        if ((tid = nosio->sio.txtid) != 0)
        {
            nosio->sio.txtid = 0;
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

ER ini_sio(NOSIO_HandleTypeDef* nosio, const char *param)
{
	uint32_t baud;
	uint8_t parity;
	uint8_t numberofStopBits;

	dis_int_sio(nosio);

	/* 制御ブロック初期化 */

	init_buf(nosio);

	/* パラメータ解析 */

	if (!set_param(param, &parity, &numberofStopBits, &baud))
		return E_PAR;

	/* デバイス初期化 */

	if (!init_sio(nosio, parity, numberofStopBits, baud))
		return E_PAR;

	/* 割込みハンドラの定義 */
	nosio->sio.flag |= TSF_INIT;

	/* 受信割込み許可 */
/*
    USCI_A_UART_enableInterrupt(
            nosio->reg,
            USCI_A_UART_RECEIVE_INTERRUPT
    );
*/
	ena_int_sio(nosio);

	return E_OK;
}

/*****************************************************************************
* シリアル入出力終了
*
******************************************************************************/

void ext_sio(NOSIO_HandleTypeDef* nosio)
{
	if (!(nosio->sio.flag & TSF_INIT))	/* 未初期化なら何もしない */
		return;
	dis_int_sio(nosio);					/* シリアル割込み禁止 */

	nosio->sio.flag &= ~TSF_INIT;		/* 初期化済みフラグクリア */
}

/*****************************************************************************
* シリアル１文字入力
*
******************************************************************************/

ER get_sio(NOSIO_HandleTypeDef* nosio, uint8_t *c, TMO tmout)
{
	ER ercd;
	int sts;
	ER_UINT wupcnt;

	for (;;)
	{
		/* 受信バッファから１文字得る */

		sts = get_rxbuf(nosio, c);

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
		get_tid(&nosio->sio.rxtid);
		Asm("nop");
		Asm("eint");
		Asm("nop");
		ercd = tslp_tsk(tmout);
		nosio->sio.rxtid = 0;
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

ER put_sio(NOSIO_HandleTypeDef* nosio, uint8_t c, TMO tmout)
{
	ER ercd;
	ER_UINT wupcnt;

    Asm("nop");
    Asm("dint");
    Asm("nop");
	USCI_A_UART_transmitData(nosio->reg, c);
    get_tid(&nosio->sio.txtid);
    Asm("nop");
    Asm("eint");
    Asm("nop");
    ercd = tslp_tsk(tmout);
    nosio->sio.txtid = 0;
    // 複数回 iwup_tsk された場合の対策
    do {
        wupcnt = can_wup(TSK_SELF);
    } while (wupcnt);
    if (ercd)
        return ercd;    /* タイムアウト終了 */

#if 0
	for (;;)
	{
	    Asm("nop");
	    Asm("dint");
	    Asm("nop");

		/* 送信バッファへ１文字格納 */

		if (put_txbuf(nosio, c))	/* 格納できた場合 */
		{
            Asm("nop");
            Asm("eint");
            Asm("nop");
			return E_OK;	/* 正常終了 */
		}

		/* 送信割込み待ち */

        get_tid(&nosio->sio.txtid);
        Asm("nop");
        Asm("eint");
        Asm("nop");
        ercd = tslp_tsk(tmout);
		nosio->sio.txtid = 0;
		// 複数回 iwup_tsk された場合の対策
		do {
			wupcnt = can_wup(TSK_SELF);
		} while (wupcnt);
		if (ercd)
			return ercd;	/* タイムアウト終了 */
	}
#endif
}

/*****************************************************************************
* シリアル入出力制御
*
******************************************************************************/

ER ctl_sio(NOSIO_HandleTypeDef* nosio, uint16_t fncd)
{
    Asm("nop");
    Asm("dint");
    Asm("nop");

	/* バッファクリア */

	if (fncd & TSIO_RXCLR)
		clr_rxbuf(nosio);
	if (fncd & TSIO_TXCLR)
		clr_txbuf(nosio);

    Asm("nop");
    Asm("eint");
    Asm("nop");

    return E_OK;
}

/*****************************************************************************
* シリアル入出力状態参照
*
******************************************************************************/

ER ref_sio(NOSIO_HandleTypeDef* nosio, T_SIOS *pk_sios)
{
	int stat;

    Asm("nop");
    Asm("dint");
    Asm("nop");

	stat = get_stat(nosio);
	if (nosio->sio.txcnt != 0)
		stat &= ~TSIO_TXEMP;

	pk_sios->siostat = (uint8_t)stat;
	pk_sios->rxlen   = nosio->sio.rxcnt;
	pk_sios->frbufsz = (uint16_t)(TXBUFSZ - nosio->sio.txcnt);
	pk_sios->eotcnt  = nosio->sio.eotcnt;

    Asm("nop");
    Asm("eint");
    Asm("nop");

    return E_OK;
}

/*****************************************************************************
* シリアル送信バッファフラッシュ
*
******************************************************************************/

ER fls_sio(NOSIO_HandleTypeDef* nosio, TMO tmout)
{
	ER ercd;
	ER_UINT wupcnt;

	for (;;)
	{
		if (nosio->sio.txcnt == 0)
		{
	        Asm("nop");
	        Asm("eint");
	        Asm("nop");
			break;	/* 正常終了 */
		}

		/* 送信終了割込み待ち */

		get_tid(&nosio->sio.tetid);
        Asm("nop");
        Asm("eint");
        Asm("nop");
		ercd = tslp_tsk(tmout);
		nosio->sio.tetid = 0;
		// 複数回 iwup_tsk された場合の対策
		do {
			wupcnt = can_wup(TSK_SELF);
		} while (wupcnt);
		if (ercd)
			return ercd;	/* タイムアウト終了 */
	}

	return E_OK;
}
