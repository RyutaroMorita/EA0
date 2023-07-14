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
    uint8_t phase;              /* �t�F�[�Y */

#define TI2CP_UNINIT        0   /* �������� */
#define TI2CP_READY         1   /* �ҋ@ */
#define TI2CP_SEND_ADDR     2   /* ���M�p�A�h���X���M */
#define TI2CP_SEND_DATA     3   /* �f�[�^���M */
#define TI2CP_RECV_ADDR     4   /* ��M�p�A�h���X���M */
#define TI2CP_RECV_READY    5   /* ��M�ҋ@ */
#define TI2CP_RECV_DATA     6   /* �f�[�^��M */
#define TI2CP_STOP          7   /* ��~ */
#define TI2CP_ERROR         8   /* �G���[ */

    uint8_t adr;                /* �A�h���X */
    ID rdytid;                  /* READY�҂��^�X�N�h�c */
    uint32_t cnt;               /* ���M�o�b�t�@���̕����� */
    uint8_t *buf;               /* �o�b�t�@�̃|�C���^ */
} T_I2CB;

/* I2C �n���h���\���� */
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

/* �֐��v���g�^�C�v */
ER ini_i2c(I2C_HandleTypeDef*);
void ext_i2c(I2C_HandleTypeDef*);
ER red_i2c(I2C_HandleTypeDef*, uint8_t, uint8_t*, uint32_t, TMO);
ER wrt_i2c(I2C_HandleTypeDef*, uint8_t, uint8_t*, uint32_t, TMO);

void i2c_rxi_isr(I2C_HandleTypeDef* sci);
void i2c_txi_isr(I2C_HandleTypeDef* sci);
void i2c_tei_isr(I2C_HandleTypeDef* sci);
void i2c_eri_isr(I2C_HandleTypeDef* sci);


#endif /* T_I2C_H */
