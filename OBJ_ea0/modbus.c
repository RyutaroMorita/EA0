/*
 * modbus.c
 *
 *  Created on: 2023/07/21
 *      Author: ryuta
 */

#include <kernel.h>
#include "t_uart.h"

#include "modbus.h"

extern UART_HandleTypeDef uart;

uint8_t m_buf[32];


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

ER modbus_read_register(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, uint16_t* pBuf, TMO tmout)
{
    uint16_t crc;
    int len;
    uint8_t* p;
    int i;
    ER ret;
    uint8_t c;
    int bytes;

    switch (fnc) {
    case HOLDING_REGISTER:
        fnc = 0x03;
        break;
    case INPUT_REGISTER:
        fnc = 0x04;
        break;
    default:
        return E_PAR;
        break;
    }

    /*
     *  Read Holding Register（03） - Query
     */
    m_buf[0] = sla;
    m_buf[1] = fnc;
    m_buf[2] = (uint8_t)(sta / 0x100U);
    m_buf[3] = (uint8_t)(sta % 0x100U);
    m_buf[4] = (uint8_t)(num / 0x100U);
    m_buf[5] = (uint8_t)(num % 0x100U);
    crc = get_crc(m_buf, 6);
    m_buf[6] = (uint8_t)(crc % 0x100U); // CRCはリトルエンディアン
    m_buf[7] = (uint8_t)(crc / 0x100U); // CRCはリトルエンディアン
    len = 8;
    p = &m_buf[0];
    for (i = 0; i < len; i++) {
        put_sio(&uart, *p, 10);
        p++;
    }

    /*
     *  Read Holding Register（03） - Response
     */
    len = 0;
    p = &m_buf[0];
    ret = get_sio(&uart, &c, tmout);
    if (E_OK != ret) {
        ctl_sio(&uart, TUART_RXCLR);
        return ret;
    }
    *p = c;
    p++;
    len++;
    if (c != sla) {
        ctl_sio(&uart, TUART_RXCLR);
        return E_MODBUS_BSLA;
    }
    ret = get_sio(&uart, &c, tmout);
    if (E_OK != ret) {
        ctl_sio(&uart, TUART_RXCLR);
        return ret;
    }
    *p = c;
    p++;
    len++;
    if (c != fnc) {
        ctl_sio(&uart, TUART_RXCLR);
        return E_MODBUS_BFNC;
    }
    ret = get_sio(&uart, &c, tmout);
    if (E_OK != ret) {
        ctl_sio(&uart, TUART_RXCLR);
        return ret;
    }
    bytes = (int)c;
    *p = c;
    p++;
    len++;
    for (i = 0; i < bytes; i++) {
        ret = get_sio(&uart, &c, tmout);
        if (E_OK != ret)
            break;
        *p = c;
        p++;
        len++;
    }
    if (i != bytes) {
        ctl_sio(&uart, TUART_RXCLR);
        return ret;
    }
    crc = 0;
    ret = get_sio(&uart, &c, tmout);
    if (E_OK != ret) {
        ctl_sio(&uart, TUART_RXCLR);
        return ret;
    }
    crc += (uint16_t)c;             // CRCはリトルエンディアン
    ret = get_sio(&uart, &c, tmout);
    if (E_OK != ret) {
        ctl_sio(&uart, TUART_RXCLR);
        return ret;
    }
    crc += (uint16_t)(c * 0x100U);  // CRCはリトルエンディアン
    if (crc != get_crc(m_buf, len)) {
        ctl_sio(&uart, TUART_RXCLR);
        return E_MODBUS_BCRC;
    }

    p = &m_buf[0];
    p += 3;
    for (i = 0; i < num; i++) {
        pBuf[i] = 0;
        pBuf[i] += (uint16_t)*p * 0x100U;
        p++;
        pBuf[i] += (uint16_t)*p;
        p++;
    }

    return E_OK;
}
