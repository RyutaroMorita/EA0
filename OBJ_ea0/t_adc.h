/*
 * t_adc.h
 *
 *  Created on: 2023/07/18
 *      Author: ryuta
 */

#ifndef T_ACD_H
#define T_ACD_H

typedef struct t_adcb
{
    ID tid;                   /* READY待ちタスクＩＤ */
    uint16_t val;
} T_ADCB;

/* I2C ハンドル構造体 */
typedef struct _ADC_HandleTypeDef
{
    uint32_t reg;
    T_ADCB adcb;
} ADC_HandleTypeDef;

/* 関数プロトタイプ */
ER ini_adc(ADC_HandleTypeDef*);
void ext_adc(ADC_HandleTypeDef*);
void stp_adc(ADC_HandleTypeDef*);
void sta_adc(ADC_HandleTypeDef*);
uint16_t red_adc(ADC_HandleTypeDef*);
ER get_adc(ADC_HandleTypeDef*, uint16_t*, TMO);
void adc_isr(ADC_HandleTypeDef* adc);

#endif /* T_ACD_H */
