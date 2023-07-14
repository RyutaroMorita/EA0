/*
 * t_i2c.h
 *
 *  Created on: 2023/07/14
 *      Author: ryuta
 */

#ifndef T_I2C_H
#define T_I2C_H

/* シリアル入出力制御ブロック構造体 */
typedef struct t_i2cb
{
    uint8_t phase;              /* フェーズ */

#define TI2CP_UNINIT        0   /* 未初期化 */
#define TI2CP_READY         1   /* 待機 */
#define TI2CP_SEND_ADDR     2   /* 送信用アドレス送信 */
#define TI2CP_SEND_DATA     3   /* データ送信 */
#define TI2CP_RECV_ADDR     4   /* 受信用アドレス送信 */
#define TI2CP_RECV_READY    5   /* 受信待機 */
#define TI2CP_RECV_DATA     6   /* データ受信 */
#define TI2CP_STOP          7   /* 停止 */
#define TI2CP_ERROR         8   /* エラー */

    uint8_t adr;                /* アドレス */
    ID rdytid;                  /* READY待ちタスクＩＤ */
    uint32_t cnt;               /* 送信バッファ内の文字数 */
    uint8_t *buf;               /* バッファのポインタ */
} T_I2CB;

/* I2C ハンドル構造体 */
typedef struct _I2C_HandleTypeDef
{
    uint32_t reg;
    INTNO rxintno;
    INTNO txintno;
    INTNO teintno;
    INTNO erintno;
    uint32_t rxiels;
    uint32_t txiels;
    uint32_t teiels;
    uint32_t eriels;
    PRI intpri;
    T_I2CB i2cb;
    uint32_t mstpcr;
    uint32_t mstpcrbit;
} I2C_HandleTypeDef;

/* 関数プロトタイプ */
ER ini_i2c(I2C_HandleTypeDef*);
void ext_i2c(I2C_HandleTypeDef*);
ER red_i2c(I2C_HandleTypeDef*, uint8_t, uint8_t*, uint32_t, TMO);
ER wrt_i2c(I2C_HandleTypeDef*, uint8_t, uint8_t*, uint32_t, TMO);

void i2c_rxi_isr(I2C_HandleTypeDef* sci);
void i2c_txi_isr(I2C_HandleTypeDef* sci);
void i2c_tei_isr(I2C_HandleTypeDef* sci);
void i2c_eri_isr(I2C_HandleTypeDef* sci);


#endif /* T_I2C_H */
