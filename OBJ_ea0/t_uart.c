//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//
//		MSP430�p�V���A���ʐM�h���C�o
//		t_uart.c (Ver.00.00.00)
//
//		2023/07/13	Ver.00.00.00	����
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#include <string.h>

#include <kernel.h>
#include <msp430.h>
#include "driverlib.h"

#include "t_uart.h"

/* ����M�o�b�t�@�T�C�Y�̒�` */

#if 0
#ifndef BUFSZ
#define BUFSZ       1024        /* ��M�o�b�t�@�� */
#endif
#ifndef TXBUFSZ
#define TXBUFSZ     BUFSZ       /* ���M�o�b�t�@�� */
#endif
#endif

/* �����ϐ� */


/*****************************************************************************
* �V���A�����o�͐���u���b�N�������i�����֐��j
*
******************************************************************************/

static void init_buf(UART_HandleTypeDef* uart)
{
	int init;

	/* ����u���b�N�N���A */

	init = uart->uartb.flag & TSF_INIT;
	memset(&uart->uartb, 0, sizeof(T_UARTB));
	uart->uartb.flag = (uint8_t)init;

	/* �|�C���^�ރZ�b�g�A�b�v */

	uart->uartb.rxbuf = uart->rxbuf;		/* ��M�o�b�t�@ */
	uart->uartb.rxputp = uart->uartb.rxbuf;	/* ��M�o�b�t�@�i�[�|�C���^�����l */
	uart->uartb.rxgetp = uart->uartb.rxbuf;	/* ��M�o�b�t�@�擾�|�C���^�����l */
}

/*****************************************************************************
* �X�e�[�^�X���� & �ҏW�i�����֐��j
*
* DSR, CD �� �����̂ŏ펞 1 �Ƃ���
******************************************************************************/

static int get_stat(UART_HandleTypeDef* uart)
{
	int stat;
	int sr;

	/* �X�e�[�^�X���� */

	sr = USCI_A_UART_queryStatusFlags(uart->reg, (UCBRK|UCOE|UCFE|UCPE));
	sr |= (uart->uartb.rxsts & (UCPE|UCFE));	/* ��M���̃X�e�[�^�X��OR */

	/* �X�e�[�^�X�r�b�g�ҏW */

	stat = TUART_DSR|TUART_CD;  /* DSR, CD �펞ON */

	if (sr & UCPE)			    /* �p���e�B�G���[ */
		stat |= TUART_PE;
	if (sr & UCOE)			    /* �I�[�o�[�����G���[ */
		stat |= TUART_OE;
	if (sr & UCFE)			    /* �t���[�~���O�G���[ */
		stat |= TUART_FE;
	if (sr & UCBRK)
		stat |= TUART_BD;

	return stat;
}

/*****************************************************************************
* �r�h�n�f�o�C�X�������i�����֐��j
*
******************************************************************************/
static int init_sio(UART_HandleTypeDef* uart, uint8_t parity, uint8_t numberofStopBits, uint32_t baud)
{
    // From Table 36-4 in the family user's manual where UCOS16 = 0 and
    //            baudrate = 9600
    //            clock freq = 4MHz
    // UCBRx = 416, UCBRFx = 6, UCBRSx = 0, UCOS16 = 0
    // http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
    USCI_A_UART_initParam param = {0};

    param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
    switch (baud) {
    case 19200:
        param.clockPrescalar = 208; // UCBRx
        param.firstModReg = 3;      // UCBRFx
        param.secondModReg = 0;     // UCBRSx
        break;
    case 38400:
        param.clockPrescalar = 104; // UCBRx
        param.firstModReg = 1;      // UCBRFx
        param.secondModReg = 0;     // UCBRSx
        break;
    case 57600:
        param.clockPrescalar = 69;  // UCBRx
        param.firstModReg = 4;      // UCBRFx
        param.secondModReg = 0;     // UCBRSx
        break;
    case 115200:
        param.clockPrescalar = 34;  // UCBRx
        param.firstModReg = 6;      // UCBRFx
        param.secondModReg = 0;     // UCBRSx
        break;
    default:
        param.clockPrescalar = 416; // UCBRx
        param.firstModReg = 6;      // UCBRFx
        param.secondModReg = 0;     // UCBRSx
        break;
    }

    param.parity = parity;
    param.msborLsbFirst = USCI_A_UART_LSB_FIRST;
    param.numberofStopBits = numberofStopBits;
    param.uartMode = USCI_A_UART_MODE;
    param.overSampling = USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;     // UCOS16 = 0

    USCI_A_UART_init(uart->reg, &param);

    //Enable UART module for operation
    USCI_A_UART_enable(uart->reg);

	return true;
}

/*****************************************************************************
* �V���A�������݋֎~�i�����֐��j
*
******************************************************************************/

static void dis_int_sio(UART_HandleTypeDef* uart)
{
    USCI_A_UART_disableInterrupt(
            uart->reg,
            USCI_A_UART_TRANSMIT_INTERRUPT
    );
    USCI_A_UART_disableInterrupt(
            uart->reg,
            USCI_A_UART_RECEIVE_INTERRUPT
    );
    USCI_A_UART_disableInterrupt(
            uart->reg,
            USCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT
    );
}

/*****************************************************************************
* �V���A�������݋��i�����֐��j
*
******************************************************************************/

static void ena_int_sio(UART_HandleTypeDef* uart)
{
    USCI_A_UART_enableInterrupt(
            uart->reg,
            USCI_A_UART_TRANSMIT_INTERRUPT
    );
    USCI_A_UART_enableInterrupt(
            uart->reg,
            USCI_A_UART_RECEIVE_INTERRUPT
    );
    USCI_A_UART_enableInterrupt(
            uart->reg,
            USCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT
    );
}

/*****************************************************************************
* ��M�o�b�t�@�N���A�i�����֐��j
*
******************************************************************************/
static void clr_rxbuf(UART_HandleTypeDef* uart)
{
	/* ��M�o�b�t�@�N���A */

	uart->uartb.rxsts = 0;
	uart->uartb.oldsts = 0;
	uart->uartb.eotcnt = 0;
	uart->uartb.rxcnt = 0;
	uart->uartb.rxgetp = uart->uartb.rxputp;
}

/*****************************************************************************
* ��M�o�b�t�@�֎�M����/�X�e�[�^�X�i�[�i�����֐��j
*
******************************************************************************/

static void put_rxbuf(UART_HandleTypeDef* uart)
{
	int cnt;
	uint8_t *p;

	/* �A���u���[�N�Ȃ�i�[���Ȃ� */

	if ((uart->uartb.rxsts & UCBRK) && (uart->uartb.oldsts & UCBRK))
		return;

	/* �o�b�t�@���t�`�F�b�N */

	cnt = uart->uartb.rxcnt;
	if (cnt == BUFSZ)
		return;
	if (++cnt == BUFSZ)
		uart->uartb.rxsts |= UCBUSY;	/* �I�[�o�t���[�� UCBUSY �Ɋ��蓖�� */

	/* �o�b�t�@�������� + 1 */

	uart->uartb.rxcnt = (uint16_t)cnt;

	/* �o�b�t�@�֊i�[ */
	/* �I�[�������o+1 */

	p = uart->uartb.rxputp;
	if ((*p = uart->uartb.rxchr) == uart->uartb.eot)
		uart->uartb.eotcnt++;
	*(p + BUFSZ) = uart->uartb.rxsts;

	/* �i�[�|�C���^���P�i�߂� */

	if (++p >= uart->uartb.rxbuf + BUFSZ)
		p = uart->uartb.rxbuf;
	uart->uartb.rxputp = p;
}

/*****************************************************************************
* ��M�o�b�t�@����P�����擾�i�����֐��j
*
* �o�b�t�@��Ŏ擾�ł��Ȃ������ꍇ�́A-1 ��Ԃ��B
******************************************************************************/

static int get_rxbuf(UART_HandleTypeDef* uart, uint8_t *c)
{
	int cnt;
	int sts;
	uint8_t *p;

	/* ��M�o�b�t�@��`�F�b�N */

	cnt = uart->uartb.rxcnt;
	if (--cnt == -1)
		return cnt;

	/* ��M�o�b�t�@�������� - 1 */

	uart->uartb.rxcnt = (uint16_t)cnt;


	/* ��M�o�b�t�@����擾 */
	/* �I�[�������o��-1 */

	p = uart->uartb.rxgetp;
	if ((*c = *p) == uart->uartb.eot)
		uart->uartb.eotcnt--;
	sts = *(p + BUFSZ);

	/* �擾�|�C���^���P�i�߂� */

	if (++p >= uart->uartb.rxbuf + BUFSZ)
		p = uart->uartb.rxbuf;
	uart->uartb.rxgetp = p;

	return sts;
}

/*****************************************************************************
* ��M�����݃n���h���{�́i�����֐��j
*
******************************************************************************/

static void rx_int(UART_HandleTypeDef* uart)
{
	int sts;
	int chr;
	int tid;

	/* ��M�X�e�[�^�X�Ǝ�M��������� */

	sts = USCI_A_UART_queryStatusFlags(uart->reg, (UCBRK|UCOE|UCFE|UCPE));
	chr = USCI_A_UART_receiveData(uart->reg);

	uart->uartb.oldsts = uart->uartb.rxsts;		/* �O��̎�M�X�e�[�^�X�L�� */
	uart->uartb.rxsts = (uint8_t)sts;
	uart->uartb.rxchr = (uint8_t)chr;

	/* ��M�o�b�t�@�֊i�[ */

	put_rxbuf(uart);

	/* ��M�҂����� */

	if ((tid = uart->uartb.rxtid) != 0)
	{
		uart->uartb.rxtid = 0;
		iwup_tsk((ID)tid);
	}
}

/*****************************************************************************
* �����݃n���h���{��
*
******************************************************************************/

void uart_isr(UART_HandleTypeDef* uart)
{
    ID tid;

    if (USCI_A_UART_getInterruptStatus(uart->reg, USCI_A_UART_RECEIVE_INTERRUPT_FLAG)) {
        rx_int(uart);
    } else
    if (USCI_A_UART_getInterruptStatus(uart->reg, USCI_A_UART_TRANSMIT_INTERRUPT_FLAG)) {
        USCI_A_UART_clearInterrupt(uart->reg, USCI_A_UART_TRANSMIT_INTERRUPT_FLAG);
        if ((tid = uart->uartb.txtid) != 0) {
            uart->uartb.txtid = 0;
            iwup_tsk((ID)tid);
        }
    }
}

/*****************************************************************************
* �������p�����[�^��́i�����֐��j
*
******************************************************************************/

static int set_param(const char *s, uint8_t* pParity, uint8_t* pNumberofStopBits, uint32_t *baud)
{
	char c;
	uint32_t b;

	/* ���[�h���� */

	*pParity = USCI_A_UART_NO_PARITY;
	*pNumberofStopBits = USCI_A_UART_ONE_STOP_BIT;
	if (strstr(s, (char *)"PE") != NULL) {
		*pParity = USCI_A_UART_EVEN_PARITY;
	} else
	if (strstr(s, (char *)"PO") != NULL) {
        *pParity = USCI_A_UART_ODD_PARITY;
	}
	if (strstr(s, (char *)"S2") != NULL) {
		*pNumberofStopBits = USCI_A_UART_TWO_STOP_BITS;
	}

	/* �{�[���[�g���� */
	/* 3���ȏ�̐��l�������ĕϊ� */

	for (;;)
	{
		c = *s++;
		if (c == '\0')
		{
			*baud = 9600L;
			break;
		}
		if (c < '0' || c > '9')
			continue;
		c = *s++;
		if (c < '0' || c > '9')
			continue;
		c = *s++;
		if (c < '0' || c > '9')
			continue;

		s -= 3;
		b = 0L;
		for (;;)
		{
			c = *s++;
			if (c < '0' || c > '9')
				break;
			b = b * 10 + (c - '0');
		}
		*baud = b;
		break;
	}
	return true;
}

/*****************************************************************************
* �V���A�����o�͏�����
*
******************************************************************************/

ER ini_sio(UART_HandleTypeDef* uart, const char *param)
{
	uint32_t baud;
	uint8_t parity;
	uint8_t numberofStopBits;

	dis_int_sio(uart);

	/* ����u���b�N������ */

	init_buf(uart);

	/* �p�����[�^��� */

	if (!set_param(param, &parity, &numberofStopBits, &baud))
		return E_PAR;

	/* �f�o�C�X������ */

	if (!init_sio(uart, parity, numberofStopBits, baud))
		return E_PAR;

	uart->uartb.flag |= TSF_INIT;

	/* �����݋��� */
	ena_int_sio(uart);

	return E_OK;
}

/*****************************************************************************
* �V���A�����o�͏I��
*
******************************************************************************/

void ext_sio(UART_HandleTypeDef* uart)
{
	if (!(uart->uartb.flag & TSF_INIT))	/* ���������Ȃ牽�����Ȃ� */
		return;
	dis_int_sio(uart);					/* �V���A�������݋֎~ */

	uart->uartb.flag &= ~TSF_INIT;		/* �������ς݃t���O�N���A */
}

/*****************************************************************************
* �V���A���P��������
*
******************************************************************************/

ER get_sio(UART_HandleTypeDef* uart, uint8_t *c, TMO tmout)
{
	ER ercd;
	int sts;
	ER_UINT wupcnt;

	for (;;)
	{
		/* ��M�o�b�t�@����P�������� */

		sts = get_rxbuf(uart, c);

		if (sts != -1)              /* ��M�����������ꍇ */
		{
		    Asm("nop");
		    Asm("dint");
		    Asm("nop");
			/* ��M�G���[���� */

			if (sts & (UCBUSY|UCBRK|UCOE|UCFE|UCPE))
			{
				if (sts & UCBUSY)			    /* �I�[�o�t���[��UCBUSY�Ɋ��蓖�� */
				ercd = EV_SIOOVF;
				else if (sts & UCOE)
				ercd = EV_SIOORN;
				else if (sts & UCBRK)
				ercd = EV_SIOBRK;
				else if (sts & UCFE)
				ercd = EV_SIOFRM;
				else /* FSR_PER */
				ercd = EV_SIOPTY;
			}
			else
				ercd = E_OK;
			return ercd;
		}

		/* ��M�����ݑ҂� */
		get_tid(&uart->uartb.rxtid);
		Asm("nop");
		Asm("eint");
		Asm("nop");
		ercd = tslp_tsk(tmout);
		uart->uartb.rxtid = 0;
		// ������ iwup_tsk ���ꂽ�ꍇ�̑΍�
		do {
			wupcnt = can_wup(TSK_SELF);
		} while (wupcnt);
		if (ercd)
			return ercd;    /* �^�C���A�E�g�I�� */
	}
}

/*****************************************************************************
* �V���A���P�����o��
*
******************************************************************************/

ER put_sio(UART_HandleTypeDef* uart, uint8_t c, TMO tmout)
{
	ER ercd;
	ER_UINT wupcnt;

    Asm("nop");
    Asm("dint");
    Asm("nop");
	USCI_A_UART_transmitData(uart->reg, c);
    get_tid(&uart->uartb.txtid);
    Asm("nop");
    Asm("eint");
    Asm("nop");
    ercd = tslp_tsk(tmout);
    uart->uartb.txtid = 0;
    // ������ iwup_tsk ���ꂽ�ꍇ�̑΍�
    do {
        wupcnt = can_wup(TSK_SELF);
    } while (wupcnt);
    if (ercd)
        return ercd;    /* �^�C���A�E�g�I�� */

    return E_OK;
}

/*****************************************************************************
* �V���A�����o�͐���
*
******************************************************************************/

ER ctl_sio(UART_HandleTypeDef* uart, uint16_t fncd)
{
    Asm("nop");
    Asm("dint");
    Asm("nop");

	/* �o�b�t�@�N���A */

	if (fncd & TUART_RXCLR)
		clr_rxbuf(uart);

    Asm("nop");
    Asm("eint");
    Asm("nop");

    return E_OK;
}

/*****************************************************************************
* �V���A�����o�͏�ԎQ��
*
******************************************************************************/

ER ref_sio(UART_HandleTypeDef* uart, T_UARTS *pk_sios)
{
	int stat;

    Asm("nop");
    Asm("dint");
    Asm("nop");

	stat = get_stat(uart);

	pk_sios->siostat = (uint8_t)stat;
	pk_sios->rxlen   = uart->uartb.rxcnt;
	pk_sios->eotcnt  = uart->uartb.eotcnt;

    Asm("nop");
    Asm("eint");
    Asm("nop");

    return E_OK;
}
