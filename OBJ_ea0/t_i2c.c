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
            USCI_B_I2C_STOP_INTERRUPT
    );
    USCI_B_I2C_disableInterrupt(
            i2c->reg,
            USCI_B_I2C_START_INTERRUPT
    );
    USCI_B_I2C_disableInterrupt(
            i2c->reg,
            USCI_B_I2C_RECEIVE_INTERRUPT
    );
    USCI_B_I2C_disableInterrupt(
            i2c->reg,
            USCI_B_I2C_TRANSMIT_INTERRUPT
    );
    USCI_B_I2C_disableInterrupt(
            i2c->reg,
            USCI_B_I2C_NAK_INTERRUPT
    );
    USCI_B_I2C_disableInterrupt(
            i2c->reg,
            USCI_B_I2C_ARBITRATIONLOST_INTERRUPT
    );
}

static void ena_int_i2c(I2C_HandleTypeDef* i2c)
{
    USCI_B_I2C_enableInterrupt(
            i2c->reg,
            USCI_B_I2C_STOP_INTERRUPT
    );
    USCI_B_I2C_enableInterrupt(
            i2c->reg,
            USCI_B_I2C_START_INTERRUPT
    );
    USCI_B_I2C_enableInterrupt(
            i2c->reg,
            USCI_B_I2C_RECEIVE_INTERRUPT
    );
    USCI_B_I2C_enableInterrupt(
            i2c->reg,
            USCI_B_I2C_TRANSMIT_INTERRUPT
    );
    USCI_B_I2C_enableInterrupt(
            i2c->reg,
            USCI_B_I2C_NAK_INTERRUPT
    );
    USCI_B_I2C_enableInterrupt(
            i2c->reg,
            USCI_B_I2C_ARBITRATIONLOST_INTERRUPT
    );
}

static bool_t debug;
static uint8_t sts;
void i2c_isr(I2C_HandleTypeDef* i2c)
{
    ID tid;
    debug = !debug;
    if (USCI_B_I2C_getInterruptStatus(i2c->reg, USCI_B_I2C_STOP_INTERRUPT)) {
        sts = USCI_B_I2C_STOP_INTERRUPT;
    }
    if (USCI_B_I2C_getInterruptStatus(i2c->reg, USCI_B_I2C_START_INTERRUPT)) {
        sts = USCI_B_I2C_START_INTERRUPT;
    }
    if (USCI_B_I2C_getInterruptStatus(i2c->reg, USCI_B_I2C_RECEIVE_INTERRUPT)) {
        sts = USCI_B_I2C_RECEIVE_INTERRUPT;
    }
    if (USCI_B_I2C_getInterruptStatus(i2c->reg, USCI_B_I2C_TRANSMIT_INTERRUPT)) {
        USCI_B_I2C_clearInterrupt(i2c->reg, USCI_B_I2C_TRANSMIT_INTERRUPT);
        USCI_B_I2C_masterSendMultiByteStop(i2c->reg);
        if ((tid = i2c->i2cb.txtid) != 0) {
            i2c->i2cb.txtid = 0;
            iwup_tsk((ID)tid);
        }
       sts = USCI_B_I2C_TRANSMIT_INTERRUPT;
    }
    if (USCI_B_I2C_getInterruptStatus(i2c->reg, USCI_B_I2C_NAK_INTERRUPT)) {
        USCI_B_I2C_clearInterrupt(i2c->reg, USCI_B_I2C_NAK_INTERRUPT);
        sts = USCI_B_I2C_NAK_INTERRUPT;
    }
    if (USCI_B_I2C_getInterruptStatus(i2c->reg, USCI_B_I2C_ARBITRATIONLOST_INTERRUPT)) {
        sts = USCI_B_I2C_ARBITRATIONLOST_INTERRUPT;
    }
}

ER ini_i2c(I2C_HandleTypeDef* i2c)
{
    USCI_B_I2C_initMasterParam I2C_param = {0};

    I2C_param.selectClockSource = USCI_B_I2C_CLOCKSOURCE_SMCLK;
    I2C_param.i2cClk = UCS_getSMCLK();
    I2C_param.dataRate = USCI_B_I2C_SET_DATA_RATE_100KBPS;

    USCI_B_I2C_initMaster(USCI_B1_BASE, &I2C_param);

    USCI_B_I2C_enable(USCI_B1_BASE);

    memset(&i2c->i2cb, 0, sizeof(T_I2CB));

    /* 割込み許可 */
    ena_int_i2c(i2c);

    return E_OK;
}

void ext_i2c(I2C_HandleTypeDef* i2c)
{
    dis_int_i2c(i2c);
}

ER red_i2c(I2C_HandleTypeDef* i2c, uint8_t adr, uint8_t* data, uint32_t len, TMO tmout)
{
    return E_OK;
}

ER wrt_i2c(I2C_HandleTypeDef* i2c, uint8_t adr, uint8_t* data, uint32_t len, TMO tmout)
{
    ER ercd;
    ER_UINT wupcnt;

    while(USCI_B_I2C_isBusBusy(i2c->reg));

    USCI_B_I2C_setSlaveAddress(i2c->reg, (adr >> 1));
    USCI_B_I2C_setMode(i2c->reg, USCI_B_I2C_TRANSMIT_MODE);

    Asm("nop");
    Asm("dint");
    Asm("nop");
    USCI_B_I2C_masterSendMultiByteStart(i2c->reg, *data);
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


#if 0

#include "MMA7455L.h"

void I2C_DIO_init(void)
{
    GPIO_setAsPeripheralModuleFunctionOutputPin(I2C_port, (I2C_SDA_pin | I2C_SCL_pin));
}

void USCI_I2C_init(void)
{
    USCI_B_I2C_initMasterParam I2C_param = {0};

    I2C_DIO_init();

    I2C_param.selectClockSource = USCI_B_I2C_CLOCKSOURCE_SMCLK;
    I2C_param.i2cClk = UCS_getSMCLK();
    I2C_param.dataRate = USCI_B_I2C_SET_DATA_RATE_100KBPS;

    USCI_B_I2C_initMaster(USCI_B1_BASE, &I2C_param);

    USCI_B_I2C_enable(USCI_B1_BASE);
}

void MMA7455L_init(void)
{
    USCI_I2C_init();
    MMA7455L_write_byte(MMA7455L_CTL1, 0x80);
    MMA7455L_write_byte(MMA7455L_MCTL, MMA_7455_2G_MODE);
}

void MMA7455L_write_byte(unsigned char address, unsigned char value)
{
    USCI_B_I2C_setSlaveAddress(USCI_B1_BASE, MMA7455L_address);
    USCI_B_I2C_setMode(USCI_B1_BASE, USCI_B_I2C_TRANSMIT_MODE);

    USCI_B_I2C_masterSendMultiByteStart(USCI_B1_BASE, address);
    while(!USCI_B_I2C_masterIsStartSent(USCI_B1_BASE));
    USCI_B_I2C_masterSendMultiByteFinish(USCI_B1_BASE, value);

    while(USCI_B_I2C_isBusBusy(USCI_B1_BASE));
}

void MMA7455L_write_word(unsigned char address, unsigned int value)
{
    unsigned char lb = 0x00;
    unsigned char hb = 0x00;

    lb = ((unsigned char)value);
    value >>= 8;
    hb = ((unsigned char)value);

    MMA7455L_write_byte(address, lb);
    MMA7455L_write_byte((address + 1), hb);
}

unsigned char MMA7455L_read_byte(unsigned char address)
{
    unsigned char value = 0x00;

    USCI_B_I2C_setSlaveAddress(USCI_B1_BASE, MMA7455L_address);
    USCI_B_I2C_setMode(USCI_B1_BASE, USCI_B_I2C_TRANSMIT_MODE);

    USCI_B_I2C_masterSendStart(USCI_B1_BASE);
    USCI_B_I2C_masterSendSingleByte(USCI_B1_BASE, address);

    USCI_B_I2C_setSlaveAddress(USCI_B1_BASE, MMA7455L_address);
    USCI_B_I2C_setMode(USCI_B1_BASE, USCI_B_I2C_RECEIVE_MODE);

    USCI_B_I2C_masterReceiveSingleStart(USCI_B1_BASE);
    value = USCI_B_I2C_masterReceiveSingle(USCI_B1_BASE);

    while(USCI_B_I2C_isBusBusy(USCI_B1_BASE));

    return value;
}

unsigned int MMA7455L_read_word(unsigned char address)
{
    unsigned char lb = 0x00;
    unsigned int hb = 0x0000;

    USCI_B_I2C_setSlaveAddress(USCI_B1_BASE, MMA7455L_address);
    USCI_B_I2C_setMode(USCI_B1_BASE, USCI_B_I2C_TRANSMIT_MODE);

    USCI_B_I2C_masterSendStart(USCI_B1_BASE);
    USCI_B_I2C_masterSendSingleByte(USCI_B1_BASE, address);

    USCI_B_I2C_setSlaveAddress(USCI_B1_BASE, MMA7455L_address);
    USCI_B_I2C_setMode(USCI_B1_BASE, USCI_B_I2C_RECEIVE_MODE);

    USCI_B_I2C_masterReceiveMultiByteStart(USCI_B1_BASE);
    lb = USCI_B_I2C_masterReceiveMultiByteNext(USCI_B1_BASE);
    hb = USCI_B_I2C_masterReceiveMultiByteFinish(USCI_B1_BASE);

    USCI_B_I2C_masterReceiveMultiByteStop(USCI_B1_BASE);

    while(USCI_B_I2C_isBusBusy(USCI_B1_BASE));

    hb <<= 8;
    hb |= lb;

    return hb;
}

signed char MMA7455L_read_axis_8(unsigned char axis)
{
    return ((signed char)MMA7455L_read_byte(axis));
}

signed int MMA7455L_read_axis_10(unsigned char axis)
{
    return ((signed int)MMA7455L_read_word(axis) & 0x03FF);
}

#endif
