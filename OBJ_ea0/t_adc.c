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

ER ini_adc(ADC_HandleTypeDef* adc)
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

void ext_adc(ADC_HandleTypeDef* adc)
{
    ADC12_A_disableInterrupt(adc->reg, ADC12_A_IE0);

    ADC12_A_disable(adc->reg);
}

void stp_adc(ADC_HandleTypeDef* adc)
{
    ADC12_A_disableConversions(adc->reg, ADC12_A_PREEMPTCONVERSION);
}

void sta_adc(ADC_HandleTypeDef* adc)
{
    stp_adc(adc);

    ADC12_A_startConversion(
        adc->reg,
        ADC12_A_MEMORY_0,
        ADC12_A_REPEATED_SINGLECHANNEL
    );
}

uint16_t red_adc(ADC_HandleTypeDef* adc)
{
    return adc->adcb.val;
}

ER get_adc(ADC_HandleTypeDef* adc, uint16_t *val, TMO tmout)
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
    // ������ iwup_tsk ���ꂽ�ꍇ�̑΍�
    do {
        wupcnt = can_wup(TSK_SELF);
    } while (wupcnt);
    if (ercd)
        return ercd;    /* �^�C���A�E�g�I�� */

    return E_OK;
}

#if 0

#include "driverlib.h"
#include "delay.h"
#include "lcd.h"
#include "lcd_print.h"

unsigned long cnt = 0;

void clock_init(void);
void GPIO_init(void);
void ADC12_init(void);

#pragma vector = ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
    switch (__even_in_range(ADC12IV, 34))
    {
        case  0: break;   //Vector  0:  No interrupt
        case  2: break;   //Vector  2:  ADC overflow
        case  4: break;   //Vector  4:  ADC timing overflow
        case  6:          //Vector  6:  ADC12IFG0
        {
            cnt = ADC12_A_getResults(ADC12_A_BASE,
                                     ADC12_A_MEMORY_0);
            break;
        }
        case  8: break;   //Vector  8:  ADC12IFG1
        case 10: break;   //Vector 10:  ADC12IFG2
        case 12: break;   //Vector 12:  ADC12IFG3
        case 14: break;   //Vector 14:  ADC12IFG4
        case 16: break;   //Vector 16:  ADC12IFG5
        case 18: break;   //Vector 18:  ADC12IFG6
        case 20: break;   //Vector 20:  ADC12IFG7
        case 22: break;   //Vector 22:  ADC12IFG8
        case 24: break;   //Vector 24:  ADC12IFG9
        case 26: break;   //Vector 26:  ADC12IFG10
        case 28: break;   //Vector 28:  ADC12IFG11
        case 30: break;   //Vector 30:  ADC12IFG12
        case 32: break;   //Vector 32:  ADC12IFG13
        case 34: break;   //Vector 34:  ADC12IFG14
        default: break;
    }
}

void main(void)
{
    unsigned long volts = 0;

    WDT_A_hold(WDT_A_BASE);

    clock_init();
    GPIO_init();
    ADC12_init();

    LCD_init();
    LCD_clear_home();

    LCD_goto(0, 0);
    LCD_putstr("ADC Count:");

    LCD_goto(0, 1);
    LCD_putstr("Volts/mV :");

    while(1)
    {
        volts = ((cnt * 3300) / 4095);
        print_I(11, 0, cnt);
        print_I(11, 1, volts);
        delay_ms(100);
    };
}

void clock_init(void)
{
    PMM_setVCore(PMM_CORE_LEVEL_3);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
                                               (GPIO_PIN4 | GPIO_PIN2));

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5,
                                                (GPIO_PIN5 | GPIO_PIN3));

    UCS_setExternalClockSource(XT1_FREQ,
                               XT2_FREQ);

    UCS_turnOnXT2(UCS_XT2_DRIVE_4MHZ_8MHZ);

    UCS_turnOnLFXT1(UCS_XT1_DRIVE_0,
                    UCS_XCAP_3);

    UCS_initClockSignal(UCS_FLLREF,
                        UCS_XT2CLK_SELECT,
                        UCS_CLOCK_DIVIDER_4);

    UCS_initFLLSettle(MCLK_KHZ,
                      MCLK_FLLREF_RATIO);

    UCS_initClockSignal(UCS_SMCLK,
                        UCS_XT2CLK_SELECT,
                        UCS_CLOCK_DIVIDER_2);

    UCS_initClockSignal(UCS_ACLK,
                        UCS_XT1CLK_SELECT,
                        UCS_CLOCK_DIVIDER_1);
}

void GPIO_init(void)
{
    GPIO_setAsOutputPin(GPIO_PORT_P4,
                        GPIO_PIN7);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,
                                               GPIO_PIN0);
}


void ADC12_init(void)
{
    ADC12_A_configureMemoryParam configureMemoryParam = {0};

    ADC12_A_init(ADC12_A_BASE,
                 ADC12_A_SAMPLEHOLDSOURCE_SC,
                 ADC12_A_CLOCKSOURCE_ACLK,
                 ADC12_A_CLOCKDIVIDER_1);

    ADC12_A_setupSamplingTimer(ADC12_A_BASE,
                               ADC12_A_CYCLEHOLD_768_CYCLES,
                               ADC12_A_CYCLEHOLD_4_CYCLES,
                               ADC12_A_MULTIPLESAMPLESENABLE);

    ADC12_A_setResolution(ADC12_A_BASE,
                          ADC12_A_RESOLUTION_12BIT);

    configureMemoryParam.memoryBufferControlIndex = ADC12_A_MEMORY_0;
    configureMemoryParam.inputSourceSelect = ADC12_A_INPUT_A0;
    configureMemoryParam.positiveRefVoltageSourceSelect = ADC12_A_VREFPOS_AVCC;
    configureMemoryParam.negativeRefVoltageSourceSelect = ADC12_A_VREFNEG_AVSS;
    configureMemoryParam.endOfSequence = ADC12_A_NOTENDOFSEQUENCE;

    ADC12_A_configureMemory(ADC12_A_BASE,
                            &configureMemoryParam);

    ADC12_A_clearInterrupt(ADC12_A_BASE,
                           ADC12IFG0);

    ADC12_A_enableInterrupt(ADC12_A_BASE,
                            ADC12IE0);

    __enable_interrupt();

    ADC12_A_enable(ADC12_A_BASE);

    ADC12_A_startConversion(ADC12_A_BASE,
                            ADC12_A_MEMORY_0,
                            ADC12_A_REPEATED_SINGLECHANNEL);

#endif
