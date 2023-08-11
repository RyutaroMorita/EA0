/*
 * t_adc.c
 *
 *  Created on: 2023/07/18
 *      Author: ryuta
 */

#include <string.h>

#include <kernel.h>
#include <msp430.h>
#include "driverlib.h"

#include "t_adc.h"


void adc_isr(ADC_HandleTypeDef* adc)
{
    ID tid;

    if (ADC12_A_getInterruptStatus(adc->reg, ADC12_A_IFG0)) {
        //ADC12_A_clearInterrupt(sci->reg, ADC12_A_IFG0);
        adc->adcb.val = ADC12_A_getResults(ADC12_A_BASE, ADC12_A_MEMORY_0);
        if ((tid = adc->adcb.tid) != 0) {
            adc->adcb.tid = 0;
            iwup_tsk((ID)tid);
        }
    }
}

__attribute__ ((section (".subtext"))) ER ini_adc(ADC_HandleTypeDef* adc)
{
    memset(&adc->adcb, 0, sizeof(T_ADCB));

    ADC12_A_configureMemoryParam configureMemoryParam = {0};

    ADC12_A_init(
        adc->reg,
        ADC12_A_SAMPLEHOLDSOURCE_SC,
        ADC12_A_CLOCKSOURCE_ACLK,
        ADC12_A_CLOCKDIVIDER_1
    );

    ADC12_A_setupSamplingTimer(
        adc->reg,
        ADC12_A_CYCLEHOLD_512_CYCLES,
        ADC12_A_CYCLEHOLD_4_CYCLES,
        ADC12_A_MULTIPLESAMPLESENABLE
    );

    ADC12_A_setResolution(adc->reg, ADC12_A_RESOLUTION_12BIT);

    configureMemoryParam.memoryBufferControlIndex = ADC12_A_MEMORY_0;
    configureMemoryParam.inputSourceSelect = ADC12_A_INPUT_A0;
    configureMemoryParam.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    configureMemoryParam.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    configureMemoryParam.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;

    ADC12_A_configureMemory(adc->reg, &configureMemoryParam);

    ADC12_A_clearInterrupt(adc->reg, ADC12_A_IFG0);

    ADC12_A_enableInterrupt(adc->reg, ADC12_A_IE0);

    ADC12_A_enable(adc->reg);

    return E_OK;
}

__attribute__ ((section (".subtext"))) void ext_adc(ADC_HandleTypeDef* adc)
{
    ADC12_A_disableInterrupt(adc->reg, ADC12_A_IE0);

    ADC12_A_disable(adc->reg);
}

__attribute__ ((section (".subtext"))) void stp_adc(ADC_HandleTypeDef* adc)
{
    ADC12_A_disableConversions(adc->reg, ADC12_A_PREEMPTCONVERSION);
}

__attribute__ ((section (".subtext"))) void sta_adc(ADC_HandleTypeDef* adc)
{
    stp_adc(adc);

    ADC12_A_startConversion(
        adc->reg,
        ADC12_A_MEMORY_0,
        ADC12_A_REPEATED_SINGLECHANNEL
    );
}

__attribute__ ((section (".subtext"))) uint16_t red_adc(ADC_HandleTypeDef* adc)
{
    return adc->adcb.val;
}

__attribute__ ((section (".subtext"))) ER get_adc(ADC_HandleTypeDef* adc, uint16_t *val, TMO tmout)
{
    ER ercd;
    ER_UINT wupcnt;

    stp_adc(adc);

    Asm("nop");
    Asm("dint");
    Asm("nop");
    ADC12_A_startConversion(
        adc->reg,
        ADC12_A_MEMORY_0,
        ADC12_A_SINGLECHANNEL
    );
    get_tid(&adc->adcb.tid);
    Asm("nop");
    Asm("eint");
    Asm("nop");
    ercd = tslp_tsk(tmout);
    adc->adcb.tid = 0;
    *val = adc->adcb.val;
    // 複数回 iwup_tsk された場合の対策
    do {
        wupcnt = can_wup(TSK_SELF);
    } while (wupcnt);
    if (ercd)
        return ercd;    /* タイムアウト終了 */

    return E_OK;
}
