/*
 * t_i2c.c
 *
 *  Created on: 2023/07/14
 *      Author: ryuta
 */

#include <string.h>

#include <kernel.h>
#include <msp430.h>
#include "driverlib.h"

#include "t_i2c.h"


static void dis_int_i2c(I2C_HandleTypeDef* i2c)
{
    USCI_B_I2C_disableInterrupt(
            i2c->reg,
            USCI_B_I2C_TRANSMIT_INTERRUPT
    );
    USCI_B_I2C_disableInterrupt(
            i2c->reg,
            USCI_B_I2C_NAK_INTERRUPT
    );
}

static void ena_int_i2c(I2C_HandleTypeDef* i2c)
{
    USCI_B_I2C_enableInterrupt(
            i2c->reg,
            USCI_B_I2C_TRANSMIT_INTERRUPT
    );
    USCI_B_I2C_enableInterrupt(
            i2c->reg,
            USCI_B_I2C_NAK_INTERRUPT
    );
}

void i2c_isr(I2C_HandleTypeDef* i2c)
{
    ID tid;

    if (USCI_B_I2C_getInterruptStatus(i2c->reg, USCI_B_I2C_TRANSMIT_INTERRUPT)) {
        USCI_B_I2C_clearInterrupt(i2c->reg, USCI_B_I2C_TRANSMIT_INTERRUPT);
        i2c->i2cb.cnt--;
        if (i2c->i2cb.cnt > 0) {
            i2c->i2cb.buf++;
            USCI_B_I2C_masterSendMultiByteNext(i2c->reg, *i2c->i2cb.buf);
        } else {
            USCI_B_I2C_masterSendMultiByteStop(i2c->reg);
            if ((tid = i2c->i2cb.txtid) != 0) {
                i2c->i2cb.txtid = 0;
                iwup_tsk((ID)tid);
            }
        }
    }

    if (USCI_B_I2C_getInterruptStatus(i2c->reg, USCI_B_I2C_NAK_INTERRUPT)) {
        USCI_B_I2C_clearInterrupt(i2c->reg, USCI_B_I2C_NAK_INTERRUPT);
        if ((tid = i2c->i2cb.txtid) != 0) {
            i2c->i2cb.txtid = 0;
            irel_wai((ID)tid);
        }
    }
}

ER ini_i2c(I2C_HandleTypeDef* i2c)
{
    USCI_B_I2C_initMasterParam I2C_param = {0};

    I2C_param.selectClockSource = USCI_B_I2C_CLOCKSOURCE_SMCLK;
    I2C_param.i2cClk = UCS_getSMCLK();
    I2C_param.dataRate = USCI_B_I2C_SET_DATA_RATE_100KBPS;

    USCI_B_I2C_initMaster(i2c->reg, &I2C_param);

    USCI_B_I2C_enable(i2c->reg);

    memset(&i2c->i2cb, 0, sizeof(T_I2CB));

    /* 割込み許可 */
    ena_int_i2c(i2c);

    return E_OK;
}

void ext_i2c(I2C_HandleTypeDef* i2c)
{
    dis_int_i2c(i2c);

    USCI_B_I2C_disable(i2c->reg);
}


ER wrt_i2c(I2C_HandleTypeDef* i2c, uint8_t adr, uint8_t* data, uint32_t len, TMO tmout)
{
    ER ercd;
    ER_UINT wupcnt;

    while(USCI_B_I2C_isBusBusy(i2c->reg));

    USCI_B_I2C_setSlaveAddress(i2c->reg, (adr >> 1));
    USCI_B_I2C_setMode(i2c->reg, USCI_B_I2C_TRANSMIT_MODE);

    i2c->i2cb.cnt = len;
    memcpy(i2c->txbuf, data, len);
    i2c->i2cb.buf = &i2c->txbuf[0];

    Asm("nop");
    Asm("dint");
    Asm("nop");
    USCI_B_I2C_masterSendMultiByteStart(i2c->reg, *i2c->i2cb.buf);
    get_tid(&i2c->i2cb.txtid);
    Asm("nop");
    Asm("eint");
    Asm("nop");
    ercd = tslp_tsk(tmout);
    i2c->i2cb.txtid = 0;
    // 複数回 iwup_tsk された場合の対策
    do {
        wupcnt = can_wup(TSK_SELF);
    } while (wupcnt);
    if (ercd)
        return ercd;    /* タイムアウト終了 */

    return E_OK;
}
