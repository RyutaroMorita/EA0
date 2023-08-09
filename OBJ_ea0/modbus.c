/*
 * modbus.c
 *
 *  Created on: 2023/07/21
 *      Author: ryuta
 */

#include <kernel.h>
//#include <msp430.h>
#include "driverlib.h"
#include "t_uart.h"

#include "modbus.h"

#define MAX3485__RE_LOW     GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2)
#define MAX3485__RE_HIGH    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2)
#define MAX3485_DE_LOW      GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7)
#define MAX3485_DE_HIGH     GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7)


extern UART_HandleTypeDef uart;

uint8_t m_buf[32];
bool_t m_f485 = false;


__attribute__ ((section (".subtext"))) void modbus_set_mode(bool_t f485)
{
    m_f485 = f485;
    if (!m_f485) {
        MAX3485__RE_HIGH;
        MAX3485_DE_LOW;
    } else {
        MAX3485__RE_LOW;
        MAX3485_DE_HIGH;
    }
}

__attribute__ ((section (".subtext"))) static uint16_t get_crc(uint8_t *z_p, uint32_t z_message_length)
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

__attribute__ ((section (".subtext"))) static ER modbus_read(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, void* pBuf, TMO tmout)
{
    uint16_t crc;
    int len;
    uint8_t* p;
    int i;
    ER ret;
    uint8_t c;
    SYSTIM tim;
    SYSTIM limit;
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
        fnc = 0x02;
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
    if (!m_f485) {
        for (i = 0; i < len; i++) {
            put_sio(&uart, *p, 10);
            p++;
        }
    } else {
        MAX3485_DE_HIGH;
        for (i = 0; i < len; i++) {
            put_sio(&uart, *p, 10);
            ret = get_sio(&uart, &c, 10);
            if (E_OK != ret) {
                ctl_sio(&uart, TUART_RXCLR);
                MAX3485_DE_LOW;
                return ret;
            }
            if (*p != c) {
                ctl_sio(&uart, TUART_RXCLR);
                MAX3485_DE_LOW;
                return E_SYS;
            }
            p++;
        }
        MAX3485_DE_LOW;
    }

    /*
     *  Read - Response
     */
    get_tim(&tim);
    limit = tim + (SYSTIM)tmout;
    while (1) {
        get_tim(&tim);
        if (tim >= limit)
            return E_TMOUT;
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
            //return E_MODBUS_BSLA;
            continue;
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
                //return E_MODBUS_BFNC;
                continue;
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
        break;
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
    case 0x00:
        break;
    case 0x01:
        return E_MODBUS_BFNC;
        break;
    case 0x02:
        return E_MODBUS_BADR;
        break;
    case 0x03:
        return E_MODBUS_BDAT;
        break;
    default:
        return E_MODBUS_BUNK;
        break;
    }

    p = &m_buf[0];
    p += 3;
    if ((fnc == 0x01) || (fnc == 0x02)) {
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

__attribute__ ((section (".subtext"))) ER modbus_read_status(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, uint8_t* pBuf, TMO tmout)
{
    if ((fnc != COIL_STATUS) && (fnc != INPUT_STATUS))
        return E_PAR;
    return modbus_read(fnc, sla, sta, num, (void*)pBuf, tmout);
}

__attribute__ ((section (".subtext"))) ER modbus_read_register(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, uint16_t* pBuf, TMO tmout)
{
    if ((fnc != HOLDING_REGISTER) && (fnc != INPUT_REGISTER))
        return E_PAR;
    return modbus_read(fnc, sla, sta, num, (void*)pBuf, tmout);
}

__attribute__ ((section (".subtext"))) static ER modbus_write(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, void* pBuf, TMO tmout)
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
    SYSTIM tim;
    SYSTIM limit;
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
    if (!m_f485) {
        for (i = 0; i < len; i++) {
            put_sio(&uart, *p, 10);
            p++;
        }
    } else {
        MAX3485_DE_HIGH;
        for (i = 0; i < len; i++) {
            put_sio(&uart, *p, 10);
            ret = get_sio(&uart, &c, 10);
            if (E_OK != ret) {
                ctl_sio(&uart, TUART_RXCLR);
                MAX3485_DE_LOW;
                return ret;
            }
            if (*p != c) {
                ctl_sio(&uart, TUART_RXCLR);
                MAX3485_DE_LOW;
                return E_SYS;
            }
            p++;
        }
        MAX3485_DE_LOW;
    }

    /*
     *  Write - Response
     */
    get_tim(&tim);
    limit = tim + (SYSTIM)tmout;
    while (1) {
        get_tim(&tim);
        if (tim >= limit)
            return E_TMOUT;
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
            //return E_MODBUS_BSLA;
            continue;
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
                //return E_MODBUS_BFNC;
                continue;
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
                //return E_MODBUS_BADR;
                continue;
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
                //return E_MODBUS_BADR;
                continue;
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
                    //return E_MODBUS_BDAT;
                    continue;
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
                    //return E_MODBUS_BDAT;
                    continue;
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
                    //return E_MODBUS_BDAT;
                    continue;
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
                    //return E_MODBUS_BDAT;
                    continue;
                }
            }
        }
        break;
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
    case 0x00:
        break;
    case 0x01:
        return E_MODBUS_BFNC;
        break;
    case 0x02:
        return E_MODBUS_BADR;
        break;
    case 0x03:
        return E_MODBUS_BDAT;
        break;
    default:
        return E_MODBUS_BUNK;
        break;
    }

    return E_OK;
}

__attribute__ ((section (".subtext"))) ER modbus_write_status(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, uint8_t* pBuf, TMO tmout)
{
    if (fnc != COIL_STATUS)
        return E_PAR;
    return modbus_write(fnc, sla, sta, num, (void*)pBuf, tmout);
}

__attribute__ ((section (".subtext"))) ER modbus_write_register(MODBUS_FUNC fnc, uint16_t sla, uint16_t sta, int num, uint16_t* pBuf, TMO tmout)
{
    if (fnc != HOLDING_REGISTER)
        return E_PAR;
    return modbus_write(fnc, sla, sta, num, (void*)pBuf, tmout);
}
