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

static ER modbus_read(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, void* pBuf, TMO tmout)
{
    uint16_t crc;
    int len;
    uint8_t* p;
    int i;
    ER ret;
    uint8_t c;
    uint8_t err = 0;
    int bytes;
    uint8_t* pByte;
    uint16_t* pWord;

    if (num == 0)
        return E_PAR;

    switch (fnc) {
    case COIL_STATUS:
        fnc = 0x01;
        break;
    case INPUT_STATUS:
        fnc = 0x01;
        break;
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
     *  Read - Query
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
     *  Read - Response
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
    if (c == (fnc | 0x80)) {
        ret = get_sio(&uart, &c, tmout);
        if (E_OK != ret) {
            ctl_sio(&uart, TUART_RXCLR);
            return ret;
        }
        *p = c;
        p++;
        len++;
        err = c;
    } else {
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

    switch (err) {
    case 0x01:
        return E_MODBUS_BFNC;
        break;
    case 0x02:
        return E_MODBUS_BADR;
        break;
    case 0x03:
        return E_MODBUS_BDAT;
        break;
    }

    p = &m_buf[0];
    p += 3;
    if ((fnc == COIL_STATUS) || (fnc == INPUT_STATUS)) {
        if (num % 8) {
            num = (num / 8) + 1;
        } else {
            num = (num / 8);
        }
        pByte = (uint8_t*)pBuf;
        for (i = 0; i < num; i++)
            *pByte++ = *p++;
    } else {
        pWord = (uint16_t*)pBuf;
        for (i = 0; i < num; i++) {
            pWord[i] = 0;
            pWord[i] += (uint16_t)*p * 0x100U;
            p++;
            pWord[i] += (uint16_t)*p;
            p++;
        }
    }

    return E_OK;
}

ER modbus_read_status(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, uint8_t* pBuf, TMO tmout)
{
    if ((fnc != COIL_STATUS) && (fnc != INPUT_STATUS))
        return E_PAR;
    return modbus_read(fnc, sla, sta, num, (void*)pBuf, tmout);
}

ER modbus_read_register(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, uint16_t* pBuf, TMO tmout)
{
    if ((fnc != HOLDING_REGISTER) && (fnc != INPUT_REGISTER))
        return E_PAR;
    return modbus_read(fnc, sla, sta, num, (void*)pBuf, tmout);
}

static ER modbus_write(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, void* pBuf, TMO tmout)
{
    bool_t mul = false;
    uint8_t* pByte;
    uint16_t* pWord;
    uint8_t dat[2];
    uint16_t crc;
    int len;
    int cnt;
    uint8_t* p;
    int i;
    ER ret;
    uint8_t c;
    uint8_t err = 0;

    if (num == 0)
        return E_PAR;

    switch (fnc) {
    case COIL_STATUS:
        if (num == 1) {
            mul = false;
            fnc = 0x05;
        } else {
            mul = true;
            fnc = 0x0F;
        }
        break;
    case HOLDING_REGISTER:
        if (num == 1) {
            mul = false;
            fnc = 0x06;
        } else {
            mul = true;
            fnc = 0x10;
        }
        break;
    default:
        return E_PAR;
        break;
    }

    /*
     *  Write - Query
     */
    m_buf[0] = sla;
    m_buf[1] = fnc;
    m_buf[2] = (uint8_t)(sta / 0x100U);
    m_buf[3] = (uint8_t)(sta % 0x100U);
    if (!mul) {
        if (fnc == 0x05) {
            pByte = (uint8_t*)pBuf;
            m_buf[4] = dat[0] = pByte[0];
            m_buf[5] = dat[1] = pByte[1];
        } else {
            pWord = (uint16_t*)pBuf;
            m_buf[4] = dat[0] = (uint8_t)(pWord[0] / 0x100U);
            m_buf[5] = dat[1] = (uint8_t)(pWord[0] % 0x100U);
        }
        crc = get_crc(m_buf, 6);
        m_buf[6] = (uint8_t)(crc % 0x100U); // CRCはリトルエンディアン
        m_buf[7] = (uint8_t)(crc / 0x100U); // CRCはリトルエンディアン
        len = 8;
    } else {
        m_buf[4] = (uint8_t)(num / 0x100U);
        m_buf[5] = (uint8_t)(num % 0x100U);
        if (fnc == 0x0F) {
            if (num % 8) {
                cnt = (num / 8) + 1;
            } else {
                cnt = (num / 8);
            }
            m_buf[6] = (uint8_t)cnt;
            pByte = (uint8_t*)pBuf;
            for (i = 0; i < cnt; i++)
                m_buf[7 + i] = *pByte++;
        } else {
            m_buf[6] = (uint8_t)(num * 2);
            cnt = 0;
            pWord = (uint16_t*)pBuf;
            for (i = 0; i < num; i++) {
                m_buf[7 + cnt] = (uint8_t)(pWord[i] / 0x100U);
                cnt++;
                m_buf[7 + cnt] = (uint8_t)(pWord[i] % 0x100U);
                cnt++;
            }
        }
        crc = get_crc(m_buf, (7 + cnt));
        m_buf[7 + cnt] = (uint8_t)(crc % 0x100U); // CRCはリトルエンディアン
        cnt++;
        m_buf[7 + cnt] = (uint8_t)(crc / 0x100U); // CRCはリトルエンディアン
        cnt++;
        len = (7 + cnt);
    }
    p = &m_buf[0];
    for (i = 0; i < len; i++) {
        put_sio(&uart, *p, 10);
        p++;
    }

    /*
     *  Write - Response
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
    if (c == (fnc | 0x80)) {
        ret = get_sio(&uart, &c, tmout);
        if (E_OK != ret) {
            ctl_sio(&uart, TUART_RXCLR);
            return ret;
        }
        *p = c;
        p++;
        len++;
        err = c;
    } else {
        if (c != fnc) {
            ctl_sio(&uart, TUART_RXCLR);
            return E_MODBUS_BFNC;
        }
        ret = get_sio(&uart, &c, tmout);
        if (E_OK != ret) {
            ctl_sio(&uart, TUART_RXCLR);
            return ret;
        }
        *p = c;
        p++;
        len++;
        if (c != (uint8_t)(sta / 0x100U)) {
            ctl_sio(&uart, TUART_RXCLR);
            return E_MODBUS_BADR;
        }
        ret = get_sio(&uart, &c, tmout);
        if (E_OK != ret) {
            ctl_sio(&uart, TUART_RXCLR);
            return ret;
        }
        *p = c;
        p++;
        len++;
        if (c != (uint8_t)(sta % 0x100U)) {
            ctl_sio(&uart, TUART_RXCLR);
            return E_MODBUS_BADR;
        }
        if (!mul) {
            ret = get_sio(&uart, &c, tmout);
            if (E_OK != ret) {
                ctl_sio(&uart, TUART_RXCLR);
                return ret;
            }
            *p = c;
            p++;
            len++;
            if (c != dat[0]) {
                ctl_sio(&uart, TUART_RXCLR);
                return E_MODBUS_BDAT;
            }
            ret = get_sio(&uart, &c, tmout);
            if (E_OK != ret) {
                ctl_sio(&uart, TUART_RXCLR);
                return ret;
            }
            *p = c;
            p++;
            len++;
            if (c != dat[1]) {
                ctl_sio(&uart, TUART_RXCLR);
                return E_MODBUS_BDAT;
            }
        } else {
            ret = get_sio(&uart, &c, tmout);
            if (E_OK != ret) {
                ctl_sio(&uart, TUART_RXCLR);
                return ret;
            }
            *p = c;
            p++;
            len++;
            if (c != (uint8_t)(num / 0x100U)) {
                ctl_sio(&uart, TUART_RXCLR);
                return E_MODBUS_BDAT;
            }
            ret = get_sio(&uart, &c, tmout);
            if (E_OK != ret) {
                ctl_sio(&uart, TUART_RXCLR);
                return ret;
            }
            *p = c;
            p++;
            len++;
            if (c != (uint8_t)(num % 0x100U)) {
                ctl_sio(&uart, TUART_RXCLR);
                return E_MODBUS_BDAT;
            }
        }
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

    switch (err) {
    case 0x01:
        return E_MODBUS_BFNC;
        break;
    case 0x02:
        return E_MODBUS_BADR;
        break;
    case 0x03:
        return E_MODBUS_BDAT;
        break;
    }

    return E_OK;
}

ER modbus_write_status(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, uint8_t* pBuf, TMO tmout)
{
    if (fnc != COIL_STATUS)
        return E_PAR;
    return modbus_write(fnc, sla, sta, num, (void*)pBuf, tmout);
}

ER modbus_write_register(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, uint16_t* pBuf, TMO tmout)
{
    if (fnc != HOLDING_REGISTER)
        return E_PAR;
    return modbus_write(fnc, sla, sta, num, (void*)pBuf, tmout);
}
