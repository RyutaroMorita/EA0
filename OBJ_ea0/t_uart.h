//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//
//		MSP430用シリアル通信ドライバ
//		t_uart.h (Ver.00.00.00)
//
//		2023/07/13	Ver.00.00.00	初版
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#ifndef T_UART_H
#define T_UART_H


#define NORTiAPI    /**/
#define cdecl       /**/

/* エラーコード */

#define EV_SIOINI   1           /* 未初期化 */
#define EV_SIOOVF   2           /* 受信バッファオーバフロー */
#define EV_SIOPTY   3           /* パリティエラー */
#define EV_SIOORN   4           /* オーバーランエラー */
#define EV_SIOFRM   5           /* フレーミングエラー */
#define EV_SIOBRK   6           /* ブレーク検出 */

/* 機能コード */

#define TUART_RXE    0x01       /* 受信イネーブル */
#define TUART_RXD    0x02       /* 受信ディセーブル */
#define TUART_TXE    0x04       /* 送信イネーブル */
#define TUART_TXD    0x08       /* 送信ディセーブル */
#define TUART_RTSON  0x10       /* RTS信号ON */
#define TUART_RTSOFF 0x20       /* RTS信号OFF */
#define TUART_DTRON  0x40       /* DTR信号ON */
#define TUART_DTROFF 0x80       /* DTR信号OFF */
#define TUART_RXCLR  0x0100     /* 受信バッファクリア */
#define TUART_TXCLR  0x0200     /* 送信バッファクリア */
#define TUART_SBON   0x0400     /* ブレーク送信ON */
#define TUART_SBOFF  0x0800     /* ブレーク送信OFF */

typedef struct t_uarts
{
    uint8_t siostat;            /* シリアル入出力ステータス */

#define TUART_CD     0x01       /* 受信キャリア検出 */
#define TUART_CTS    0x02       /* CTS信号ON(1)/OFF(0) */
#define TUART_TXEMP  0x04       /* 送信バッファ空 */
#define TUART_PE     0x08       /* パリティエラー */
#define TUART_OE     0x10       /* オーバランエラー */
#define TUART_FE     0x20       /* フレーミングエラー */
#define TUART_BD     0x40       /* ブレーク状態検出 */
#define TUART_DSR    0x80       /* DSR信号ON(1)/OFF(0) */

    uint8_t rxchr;              /* 受信バッファの先頭の文字（未使用）*/
    uint16_t rxlen;             /* 受信バッファのデータ長 */
    uint16_t frbufsz;           /* 送信バッファの空きサイズ */
    uint16_t eotcnt;            /* 受信バッファの終端文字個数 */

} T_UARTS;

/* シリアル入出力制御ブロック構造体 */

typedef struct t_uscib
{
	uint8_t ch;                 /* チャネル番号 */
	uint8_t flag;               /* 制御フラグ */

#define TSF_INIT    0x01        /* 初期化済み */
#define TSF_TXREQ   0x02        /* XON/XOFF送信要求 */
#define TSF_RXOFF   0x04        /* XOFF受信した */
#define TSF_TXOFF   0x08        /* XOFF送信した */
#define TSF_XON     0x10        /* XON/OFFによるフロー制御有り */
#define TSF_DTR     0x20        /* DTRによるフロー制御有り */
#define TSF_RTS     0x40        /* RTSによるフロー制御有り */

	uint8_t txchr;              /* 送信文字 */
	uint8_t rxchr;              /* 受信文字 */
	uint8_t rxsts;              /* 受信ステータス */
	uint8_t oldchr;             /* 前回の受信文字 */
	uint8_t oldsts;             /* 前回の受信ステータス */

	uint8_t eot;                /* 終端文字 */
	uint16_t eotcnt;            /* 終端文字検出カウンタ */

	ID txtid;                   /* 送信待ちタスクＩＤ */
	ID rxtid;                   /* 受信待ちタスクＩＤ */
	ID tetid;                   /* 送信終了待ちタスクＩＤ */

	uint8_t cmd[6];             /* SIOコマンドバッファ */
	uint8_t rsv[2];             /* 予備 */

	uint16_t rxcnt;             /* 受信バッファ内の文字数 */

	uint8_t *rxbuf;             /* 受信バッファ（+BUFSZ:受信ステータスバッファ）*/
	uint8_t *rxputp;            /* 受信バッファ格納ポインタ */
	uint8_t *rxgetp;            /* 受信バッファ取得ポインタ */

  #ifdef NORTi86
	int16_t far *vram;          /* モニタ表示用テキストVRAMアドレス */
  #endif

} T_UARTB;

/* 送受信バッファサイズの定義 */

#ifndef BUFSZ
#define BUFSZ       256         /* 受信バッファ長 */
#endif

/*  */
typedef struct _UART_HandleTypeDef
{
	uint32_t reg;
	uint8_t rxbuf[BUFSZ*2];		/* 受信バッファ, 受信ステータスバッファ */
	T_UARTB uartb;
} UART_HandleTypeDef;

/* 関数プロトタイプ */

ER cdecl ini_sio(UART_HandleTypeDef*, const char *);
void cdecl ext_sio(UART_HandleTypeDef*);
ER cdecl get_sio(UART_HandleTypeDef*, uint8_t *, TMO);
ER cdecl put_sio(UART_HandleTypeDef*, uint8_t, TMO);
ER cdecl ctl_sio(UART_HandleTypeDef*, uint16_t);
ER cdecl ref_sio(UART_HandleTypeDef*, T_UARTS *);

void uart_isr(UART_HandleTypeDef* nosio);

#endif /* T_UART_H */
