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
 *  ��L���쌠�҂́C�ȉ���(1)�`(4)�̏����𖞂����ꍇ�Ɍ���C�{�\�t�g�E�F
 *  �A�i�{�\�t�g�E�F�A�����ς������̂��܂ށD�ȉ������j���g�p�E�����E��
 *  �ρE�Ĕz�z�i�ȉ��C���p�ƌĂԁj���邱�Ƃ𖳏��ŋ�������D
 *  (1) �{�\�t�g�E�F�A���\�[�X�R�[�h�̌`�ŗ��p����ꍇ�ɂ́C��L�̒���
 *      ���\���C���̗��p��������щ��L�̖��ۏ؋K�肪�C���̂܂܂̌`�Ń\�[
 *      �X�R�[�h���Ɋ܂܂�Ă��邱�ƁD
 *  (2) �{�\�t�g�E�F�A���C���C�u�����`���ȂǁC���̃\�t�g�E�F�A�J���Ɏg
 *      �p�ł���`�ōĔz�z����ꍇ�ɂ́C�Ĕz�z�ɔ����h�L�������g�i���p
 *      �҃}�j���A���Ȃǁj�ɁC��L�̒��쌠�\���C���̗��p��������щ��L
 *      �̖��ۏ؋K����f�ڂ��邱�ƁD
 *  (3) �{�\�t�g�E�F�A���C�@��ɑg�ݍ��ނȂǁC���̃\�t�g�E�F�A�J���Ɏg
 *      �p�ł��Ȃ��`�ōĔz�z����ꍇ�ɂ́C���̂����ꂩ�̏����𖞂�����
 *      �ƁD
 *    (a) �Ĕz�z�ɔ����h�L�������g�i���p�҃}�j���A���Ȃǁj�ɁC��L�̒�
 *        �쌠�\���C���̗��p��������щ��L�̖��ۏ؋K����f�ڂ��邱�ƁD
 *    (b) �Ĕz�z�̌`�Ԃ��C�ʂɒ�߂���@�ɂ���āCTOPPERS�v���W�F�N�g��
 *        �񍐂��邱�ƁD
 *  (4) �{�\�t�g�E�F�A�̗��p�ɂ�蒼�ړI�܂��͊ԐړI�ɐ����邢���Ȃ鑹
 *      �Q������C��L���쌠�҂����TOPPERS�v���W�F�N�g��Ɛӂ��邱�ƁD
 *      �܂��C�{�\�t�g�E�F�A�̃��[�U�܂��̓G���h���[�U����̂����Ȃ闝
 *      �R�Ɋ�Â�����������C��L���쌠�҂����TOPPERS�v���W�F�N�g��
 *      �Ɛӂ��邱�ƁD
 *
 *  �{�\�t�g�E�F�A�́C���ۏ؂Œ񋟂���Ă�����̂ł���D��L���쌠�҂�
 *  ���TOPPERS�v���W�F�N�g�́C�{�\�t�g�E�F�A�Ɋւ��āC����̎g�p�ړI
 *  �ɑ΂���K�������܂߂āC�����Ȃ�ۏ؂��s��Ȃ��D�܂��C�{�\�t�g�E�F
 *  �A�̗��p�ɂ�蒼�ړI�܂��͊ԐړI�ɐ����������Ȃ鑹�Q�Ɋւ��Ă��C��
 *  �̐ӔC�𕉂�Ȃ��D
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
#include "t_adc.h"
#include "t_i2c.h"
#include "t_uart.h"
#include "lcd.h"
#include "modbus.h"

#include "main.h"


#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

extern void target_fput_log(char c);

ADC_HandleTypeDef adc;
I2C_HandleTypeDef i2c;
UART_HandleTypeDef uart;

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

static void main_init(void)
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

    uart.reg = (uint32_t)USCI_A1_BASE;
    ini_sio(&uart, "9600 B8 PN S1");
    ctl_sio(&uart, TUART_RXE | TUART_TXE | TUART_DTRON | TUART_RTSON);

    dly_tsk(40);
    i2c.reg = (uint32_t)USCI_B1_BASE;
    ini_i2c(&i2c);

    adc.reg = (uint32_t)ADC12_A_BASE;
    ini_adc(&adc);

    //lcd_init();
}

static uint16_t get_crc(uint8_t *z_p, uint32_t z_message_length)
{
    uint16_t crc= 0xffff;
    uint16_t next;
    uint16_t carry;
    uint16_t n;

    while (z_message_length--) {
        next = (uint16_t)*z_p;
        crc ^= next;
        for (n = 0; n < 8; n++) {
            carry = crc & 1;
            crc >>= 1;
            if (carry)
                crc ^= 0xA001;
        }
        z_p++;
    }

    return crc;
}

/*
 *  printf()��sprintf()�Łu%f�v��u%g�v���g�p����ꍇ��
 *  �����J�̃I�v�V�����Ƃ��āu-u _printf_float�v��ǋL���邱��
 */
void main_task(intptr_t exinf)
{
    uint8_t buf[32];
    float tmp = 1.23456;
    uint8_t* p;
    int i;
    uint8_t c;
    uint16_t val;
    uint8_t sla = 0x01; // �X���[�u�A�h���X
    uint8_t fnc;
    uint16_t sta = 0;
    uint16_t num = 1;
    uint16_t crc;
    int bytes;
    uint16_t reg[4];
    uint8_t sts[4];
    int len;
    ER ret;

    main_init();

    modbus_set_mode(true);

    sprintf((char*)buf, "sprintf() Test = %g\r\n", tmp);
//#if 0
    p = &buf[0];
    for (i = 0; i < strlen((const char*)buf); i++) {
        put_sio(&uart, *p, 10);
        p++;
    }
//#endif
#if 0
    c = 'T';
    while (1) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);   // DE High
        put_sio(&uart, c, 10);
        if (E_OK == get_sio(&uart, &c, 1000)) {
            if (c == 'T')
                lcd_draw_text(0, 0, (uint8_t*)"OK!");
            else
                lcd_draw_text(0, 0, (uint8_t*)"NG!");
        } else {
            lcd_draw_text(0, 0, (uint8_t*)"NG!");
        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);    // DE Low
        dly_tsk(500);
        lcd_clear();
        dly_tsk(500);
    }
#endif

    // Display
    //lcd_draw_text(0, 0, (uint8_t*)"0123456789ABCDEF");
    //lcd_set_cursor(1, 0, false, true);

    //sta_adc(&adc);
    //dly_tsk(10);

    while (1) {
#if 0
        ret = modbus_read_register(HOLDING_REGISTER, 0x0001U, 0x0000U, 2, &reg[0], 1000);
        if (E_OK == ret) {
            sprintf((char*)buf, "%d", reg[0]);
            lcd_draw_text(0, 0, buf);
            sprintf((char*)buf, "%d", reg[1]);
            lcd_draw_text(1, 0, buf);
        } else {
            switch (ret) {
            case E_MODBUS_BFNC:
                sprintf((char*)buf, "Illegal Func.");
                lcd_draw_text(0, 0, buf);
                break;
            case E_MODBUS_BADR:
                sprintf((char*)buf, "Illegal DataAdr.");
                lcd_draw_text(0, 0, buf);
                break;
            case E_MODBUS_BDAT:
                sprintf((char*)buf, "Illegal DataVal.");
                lcd_draw_text(0, 0, buf);
                break;
            }
        }
#endif
#if 0
        if (E_OK == modbus_read_status(COIL_STATUS, 0x0001U, 0x0000U, 10, &sts[0], 1000)) {
            sprintf((char*)buf, "%d", ((sts[0] >> 0) & 0x0001));
            lcd_draw_text(0, 0, buf);
            sprintf((char*)buf, "%d", ((sts[1] >> 1) & 0x0001));
            lcd_draw_text(1, 0, buf);
        }
#endif
#if 0
        reg[0] = 0x1234;
        //reg[1] = 0x5678;
        modbus_write_register(HOLDING_REGISTER, 0x0001U, 0x0000U, 1, &reg[0], 1000);
#endif
#if 0
        sts[0] = 0xAA;
        sts[1] = 0x02;
        ret = modbus_write_status(COIL_STATUS, 0x0001U, 0x0000U, 11, &sts[0], 1000);
        if (E_OK == ret) {
            //
        } else {
            switch (ret) {
            case E_MODBUS_BFNC:
                sprintf((char*)buf, "Illegal Func.");
                lcd_draw_text(0, 0, buf);
                break;
            case E_MODBUS_BADR:
                sprintf((char*)buf, "Illegal DataAdr.");
                lcd_draw_text(0, 0, buf);
                break;
            case E_MODBUS_BDAT:
                sprintf((char*)buf, "Illegal DataVal.");
                lcd_draw_text(0, 0, buf);
                break;
            }
        }
        dly_tsk(100);
#endif
//#if 0
        if (E_OK == get_sio(&uart, &c, 1000)) {
            put_sio(&uart, c, 10);
        }
//#endif
#if 0
//        sprintf((char*)buf, "val = %u\r\n", red_adc(&adc));
        if (E_OK == get_adc(&adc, &val, 10))
            sprintf((char*)buf, "val = %u\r\n", val);
        p = &buf[0];
        for (i = 0; i < strlen((const char*)buf); i++) {
            put_sio(&uart, *p, 10);
            p++;
        }
        dly_tsk(1000);
#endif
    }
}
