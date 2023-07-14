//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//
//		MSP430�p�V���A���ʐM�h���C�o
//		t_usci.c (Ver.00.00.00)
//
//		2023/07/13	Ver.00.00.00	����
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#include <string.h>

#include <kernel.h>
#include <msp430.h>
#include "driverlib.h"

#include "t_usci.h"

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

static void init_buf(USCI_HandleTypeDef* usci)
{
	int init;

	/* ����u���b�N�N���A */

	init = usci->uscib.flag & TSF_INIT;
	memset(&usci->uscib, 0, sizeof(T_USCIB));
	usci->uscib.flag = (uint8_t)init;

	/* �|�C���^�ރZ�b�g�A�b�v */

	usci->uscib.rxbuf = usci->rxbuf;		/* ��M�o�b�t�@ */
	usci->uscib.rxputp = usci->uscib.rxbuf;	/* ��M�o�b�t�@�i�[�|�C���^�����l */
	usci->uscib.rxgetp = usci->uscib.rxbuf;	/* ��M�o�b�t�@�擾�|�C���^�����l */
}

/*****************************************************************************
* �X�e�[�^�X���� & �ҏW�i�����֐��j
*
* DSR, CD �� �����̂ŏ펞 1 �Ƃ���
******************************************************************************/

static int get_stat(USCI_HandleTypeDef* usci)
{
	int stat;
	int sr;

	/* �X�e�[�^�X���� */

	sr = USCI_A_UART_queryStatusFlags(usci->reg, (UCBRK|UCOE|UCFE|UCPE));
	sr |= (usci->uscib.rxsts & (UCPE|UCFE));	/* ��M���̃X�e�[�^�X��OR */

	/* �X�e�[�^�X�r�b�g�ҏW */

	stat = TUSCI_DSR|TUSCI_CD;  /* DSR, CD �펞ON */

	if (sr & UCPE)			    /* �p���e�B�G���[ */
		stat |= TUSCI_PE;
	if (sr & UCOE)			    /* �I�[�o�[�����G���[ */
		stat |= TUSCI_OE;
	if (sr & UCFE)			    /* �t���[�~���O�G���[ */
		stat |= TUSCI_FE;
	if (sr & UCBRK)
		stat |= TUSCI_BD;

	return stat;
}

/*****************************************************************************
* �r�h�n�f�o�C�X�������i�����֐��j
*
******************************************************************************/
static int init_sio(USCI_HandleTypeDef* usci, uint8_t parity, uint8_t numberofStopBits, uint32_t baud)
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

    USCI_A_UART_init(usci->reg, &param);

    //Enable UART module for operation
    USCI_A_UART_enable(usci->reg);

	return true;
}

/*****************************************************************************
* �V���A�������݋֎~�i�����֐��j
*
******************************************************************************/

static void dis_int_sio(USCI_HandleTypeDef* usci)
{
    USCI_A_UART_disableInterrupt(
            usci->reg,
            USCI_A_UART_TRANSMIT_INTERRUPT
    );
    USCI_A_UART_disableInterrupt(
            usci->reg,
            USCI_A_UART_RECEIVE_INTERRUPT
    );
    USCI_A_UART_disableInterrupt(
            usci->reg,
            USCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT
    );
}

/*****************************************************************************
* �V���A�������݋��i�����֐��j
*
******************************************************************************/

static void ena_int_sio(USCI_HandleTypeDef* usci)
{
    USCI_A_UART_enableInterrupt(
            usci->reg,
            USCI_A_UART_TRANSMIT_INTERRUPT
    );
    USCI_A_UART_enableInterrupt(
            usci->reg,
            USCI_A_UART_RECEIVE_INTERRUPT
    );
    USCI_A_UART_enableInterrupt(
            usci->reg,
            USCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT
    );
}

/*****************************************************************************
* ��M�o�b�t�@�N���A�i�����֐��j
*
******************************************************************************/
static void clr_rxbuf(USCI_HandleTypeDef* usci)
{
	/* ��M�o�b�t�@�N���A */

	usci->uscib.rxsts = 0;
	usci->uscib.oldsts = 0;
	usci->uscib.eotcnt = 0;
	usci->uscib.rxcnt = 0;
	usci->uscib.rxgetp = usci->uscib.rxputp;
}

/*****************************************************************************
* ��M�o�b�t�@�֎�M����/�X�e�[�^�X�i�[�i�����֐��j
*
******************************************************************************/

static void put_rxbuf(USCI_HandleTypeDef* usci)
{
	int cnt;
	uint8_t *p;

	/* �A���u���[�N�Ȃ�i�[���Ȃ� */

	if ((usci->uscib.rxsts & UCBRK) && (usci->uscib.oldsts & UCBRK))
		return;

	/* �o�b�t�@���t�`�F�b�N */

	cnt = usci->uscib.rxcnt;
	if (cnt == BUFSZ)
		return;
	if (++cnt == BUFSZ)
		usci->uscib.rxsts |= UCBUSY;	/* �I�[�o�t���[�� UCBUSY �Ɋ��蓖�� */

	/* �o�b�t�@�������� + 1 */

	usci->uscib.rxcnt = (uint16_t)cnt;

	/* �o�b�t�@�֊i�[ */
	/* �I�[�������o+1 */

	p = usci->uscib.rxputp;
	if ((*p = usci->uscib.rxchr) == usci->uscib.eot)
		usci->uscib.eotcnt++;
	*(p + BUFSZ) = usci->uscib.rxsts;

	/* �i�[�|�C���^���P�i�߂� */

	if (++p >= usci->uscib.rxbuf + BUFSZ)
		p = usci->uscib.rxbuf;
	usci->uscib.rxputp = p;
}

/*****************************************************************************
* ��M�o�b�t�@����P�����擾�i�����֐��j
*
* �o�b�t�@��Ŏ擾�ł��Ȃ������ꍇ�́A-1 ��Ԃ��B
******************************************************************************/

static int get_rxbuf(USCI_HandleTypeDef* usci, uint8_t *c)
{
	int cnt;
	int sts;
	uint8_t *p;

	/* ��M�o�b�t�@��`�F�b�N */

	cnt = usci->uscib.rxcnt;
	if (--cnt == -1)
		return cnt;

	/* ��M�o�b�t�@�������� - 1 */

	usci->uscib.rxcnt = (uint16_t)cnt;


	/* ��M�o�b�t�@����擾 */
	/* �I�[�������o��-1 */

	p = usci->uscib.rxgetp;
	if ((*c = *p) == usci->uscib.eot)
		usci->uscib.eotcnt--;
	sts = *(p + BUFSZ);

	/* �擾�|�C���^���P�i�߂� */

	if (++p >= usci->uscib.rxbuf + BUFSZ)
		p = usci->uscib.rxbuf;
	usci->uscib.rxgetp = p;

	return sts;
}

/*****************************************************************************
* ��M�����݃n���h���{�́i�����֐��j
*
******************************************************************************/

static void rx_int(USCI_HandleTypeDef* usci)
{
	int sts;
	int chr;
	int tid;

	/* ��M�X�e�[�^�X�Ǝ�M��������� */

	sts = USCI_A_UART_queryStatusFlags(usci->reg, (UCBRK|UCOE|UCFE|UCPE));
	chr = USCI_A_UART_receiveData(usci->reg);

	usci->uscib.oldsts = usci->uscib.rxsts;		/* �O��̎�M�X�e�[�^�X�L�� */
	usci->uscib.rxsts = (uint8_t)sts;
	usci->uscib.rxchr = (uint8_t)chr;

	/* ��M�o�b�t�@�֊i�[ */

	put_rxbuf(usci);

	/* ��M�҂����� */

	if ((tid = usci->uscib.rxtid) != 0)
	{
		usci->uscib.rxtid = 0;
		iwup_tsk((ID)tid);
	}
}

/*****************************************************************************
* �����݃n���h���{��
*
******************************************************************************/

void usci_isr(USCI_HandleTypeDef* usci)
{
    ID tid;

    if (USCI_A_UART_getInterruptStatus(usci->reg, USCI_A_UART_RECEIVE_INTERRUPT_FLAG)) {
        rx_int(usci);
    } else
    if (USCI_A_UART_getInterruptStatus(usci->reg, USCI_A_UART_TRANSMIT_INTERRUPT_FLAG)) {
        USCI_A_UART_clearInterrupt(usci->reg, USCI_A_UART_TRANSMIT_INTERRUPT_FLAG);
        if ((tid = usci->uscib.txtid) != 0) {
            usci->uscib.txtid = 0;
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

ER ini_sio(USCI_HandleTypeDef* usci, const char *param)
{
	uint32_t baud;
	uint8_t parity;
	uint8_t numberofStopBits;

	dis_int_sio(usci);

	/* ����u���b�N������ */

	init_buf(usci);

	/* �p�����[�^��� */

	if (!set_param(param, &parity, &numberofStopBits, &baud))
		return E_PAR;

	/* �f�o�C�X������ */

	if (!init_sio(usci, parity, numberofStopBits, baud))
		return E_PAR;

	/* �����݃n���h���̒�` */
	usci->uscib.flag |= TSF_INIT;

	/* �����݋��� */
	ena_int_sio(usci);

	return E_OK;
}

/*****************************************************************************
* �V���A�����o�͏I��
*
******************************************************************************/

void ext_sio(USCI_HandleTypeDef* usci)
{
	if (!(usci->uscib.flag & TSF_INIT))	/* ���������Ȃ牽�����Ȃ� */
		return;
	dis_int_sio(usci);					/* �V���A�������݋֎~ */

	usci->uscib.flag &= ~TSF_INIT;		/* �������ς݃t���O�N���A */
}

/*****************************************************************************
* �V���A���P��������
*
******************************************************************************/

ER get_sio(USCI_HandleTypeDef* usci, uint8_t *c, TMO tmout)
{
	ER ercd;
	int sts;
	ER_UINT wupcnt;

	for (;;)
	{
		/* ��M�o�b�t�@����P�������� */

		sts = get_rxbuf(usci, c);

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
		get_tid(&usci->uscib.rxtid);
		Asm("nop");
		Asm("eint");
		Asm("nop");
		ercd = tslp_tsk(tmout);
		usci->uscib.rxtid = 0;
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

ER put_sio(USCI_HandleTypeDef* usci, uint8_t c, TMO tmout)
{
	ER ercd;
	ER_UINT wupcnt;

    Asm("nop");
    Asm("dint");
    Asm("nop");
	USCI_A_UART_transmitData(usci->reg, c);
    get_tid(&usci->uscib.txtid);
    Asm("nop");
    Asm("eint");
    Asm("nop");
    ercd = tslp_tsk(tmout);
    usci->uscib.txtid = 0;
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

ER ctl_sio(USCI_HandleTypeDef* usci, uint16_t fncd)
{
    Asm("nop");
    Asm("dint");
    Asm("nop");

	/* �o�b�t�@�N���A */

	if (fncd & TUSCI_RXCLR)
		clr_rxbuf(usci);

    Asm("nop");
    Asm("eint");
    Asm("nop");

    return E_OK;
}

/*****************************************************************************
* �V���A�����o�͏�ԎQ��
*
******************************************************************************/

ER ref_sio(USCI_HandleTypeDef* usci, T_USCIS *pk_sios)
{
	int stat;

    Asm("nop");
    Asm("dint");
    Asm("nop");

	stat = get_stat(usci);

	pk_sios->siostat = (uint8_t)stat;
	pk_sios->rxlen   = usci->uscib.rxcnt;
	pk_sios->eotcnt  = usci->uscib.eotcnt;

    Asm("nop");
    Asm("eint");
    Asm("nop");

    return E_OK;
}
