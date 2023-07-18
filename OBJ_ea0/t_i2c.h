/*
 * t_i2c.h
 *
 *  Created on: 2023/07/14
 *      Author: ryuta
 */

#ifndef T_I2C_H
#define T_I2C_H

/* �V���A�����o�͐���u���b�N�\���� */
typedef struct t_i2cb
{
    ID txtid;                   /* READY�҂��^�X�N�h�c */
    uint32_t cnt;               /* ���M�o�b�t�@���̕����� */
    uint8_t *buf;               /* �o�b�t�@�̃|�C���^ */
} T_I2CB;

/* I2C �n���h���\���� */
typedef struct _I2C_HandleTypeDef
{
    uint32_t reg;
    uint8_t txbuf[8];           /* ���M�o�b�t�@ */
    T_I2CB i2cb;
} I2C_HandleTypeDef;

/* �֐��v���g�^�C�v */
ER ini_i2c(I2C_HandleTypeDef*);
void ext_i2c(I2C_HandleTypeDef*);
ER wrt_i2c(I2C_HandleTypeDef*, uint8_t, uint8_t*, uint32_t, TMO);

void i2c_isr(I2C_HandleTypeDef* sci);

#endif /* T_I2C_H */
