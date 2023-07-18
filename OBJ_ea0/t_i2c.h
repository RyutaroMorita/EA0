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
    ID txtid;                   /* READY待ちタスクＩＤ */
    uint32_t cnt;               /* 送信バッファ内の文字数 */
    uint8_t *buf;               /* バッファのポインタ */
} T_I2CB;

/* I2C ハンドル構造体 */
typedef struct _I2C_HandleTypeDef
{
    uint32_t reg;
    uint8_t txbuf[8];           /* 送信バッファ */
    T_I2CB i2cb;
} I2C_HandleTypeDef;

/* 関数プロトタイプ */
ER ini_i2c(I2C_HandleTypeDef*);
void ext_i2c(I2C_HandleTypeDef*);
ER wrt_i2c(I2C_HandleTypeDef*, uint8_t, uint8_t*, uint32_t, TMO);

void i2c_isr(I2C_HandleTypeDef* sci);

#endif /* T_I2C_H */
