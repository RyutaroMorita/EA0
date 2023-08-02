/*
 * modbus.h
 *
 *  Created on: 2023/07/21
 *      Author: ryuta
 */

#ifndef MODBUS_H
#define MODBUS_H

typedef enum {
    COIL_STATUS,
    INPUT_STATUS,
    HOLDING_REGISTER,
    INPUT_REGISTER
} MODBUS_FUNC;

//#define E_MODBUS_BSLA    (-100)
#define E_MODBUS_BFNC    (-101)
#define E_MODBUS_BADR    (-102)
#define E_MODBUS_BDAT    (-103)
#define E_MODBUS_BCRC    (-104)
#define E_MODBUS_BUNK    (-105)


void modbus_set_mode(bool_t f485);
ER modbus_read_status(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, uint8_t* pBuf, TMO tmout);
ER modbus_read_register(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, uint16_t* pBuf, TMO tmout);
ER modbus_write_status(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, uint8_t* pBuf, TMO tmout);
ER modbus_write_register(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, uint16_t* pBuf, TMO tmout);


#endif /* MODBUS_H */
