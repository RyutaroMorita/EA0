//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//
//		MSP430�p�V���A���ʐM�h���C�o
//		nosio.c (Ver.00.00.00)
//
//		2023/07/13	Ver.00.00.00	����
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#include <string.h>

#include <kernel.h>
#include <msp430.h>
#include <sil.h>

 #ifdef TOPPERS_ASP
  #define PSW         PRI
 #else
  #define PSW         UINT
 #endif

#include "driverlib.h"
#include "nosio.h"

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

static bool_t tx_ready = false;

/*****************************************************************************
* �V���A�����o�͐���u���b�N�������i�����֐��j
*
******************************************************************************/

static void init_buf(NOSIO_HandleTypeDef* nosio)
{
	int init;

	/* ����u���b�N�N���A */

	init = nosio->sio.flag & TSF_INIT;
	memset(&nosio->sio, 0, sizeof(T_SIO));
	nosio->sio.flag = (uint8_t)init;

	/* �|�C���^�ރZ�b�g�A�b�v */

	nosio->sio.txbuf = nosio->txbuf;		/* ���M�o�b�t�@ */
	nosio->sio.rxbuf = nosio->rxbuf;		/* ��M�o�b�t�@ */
	nosio->sio.txputp = nosio->sio.txbuf;	/* ���M�o�b�t�@�i�[�|�C���^�����l */
	nosio->sio.txgetp = nosio->sio.txbuf;	/* ���M�o�b�t�@�擾�|�C���^�����l */
	nosio->sio.rxputp = nosio->sio.rxbuf;	/* ��M�o�b�t�@�i�[�|�C���^�����l */
	nosio->sio.rxgetp = nosio->sio.rxbuf;	/* ��M�o�b�t�@�擾�|�C���^�����l */
}

/*****************************************************************************
* �X�e�[�^�X���� & �ҏW�i�����֐��j
*
* DSR, CD �� �����̂ŏ펞 1 �Ƃ���
******************************************************************************/

static int get_stat(NOSIO_HandleTypeDef* nosio)
{
	int stat;
	int sr;

	/* �X�e�[�^�X���� */

	sr = USCI_A_UART_queryStatusFlags(nosio->reg, (UCBRK|UCOE|UCFE|UCPE));
	sr |= (nosio->sio.rxsts & (UCPE|UCFE));	/* ��M���̃X�e�[�^�X��OR */

	/* �X�e�[�^�X�r�b�g�ҏW */

	stat = TSIO_DSR|TSIO_CD;		/* DSR, CD �펞ON */

	if (sr & UCPE)			        /* �p���e�B�G���[ */
		stat |= TSIO_PE;
	if (sr & UCOE)			        /* �I�[�o�[�����G���[ */
		stat |= TSIO_OE;
	if (sr & UCFE)			        /* �t���[�~���O�G���[ */
		stat |= TSIO_FE;
	if (sr & UCBRK)
		stat |= TSIO_BD;

	return stat;
}

/*****************************************************************************
* �r�h�n�f�o�C�X�������i�����֐��j
*
******************************************************************************/
static int init_sio(NOSIO_HandleTypeDef* nosio, uint8_t parity, uint8_t numberofStopBits, uint32_t baud)
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

    USCI_A_UART_init(nosio->reg, &param);

    //Enable UART module for operation
    USCI_A_UART_enable(nosio->reg);

	return true;
}

/*****************************************************************************
* �V���A�������݋֎~�i�����֐��j
*
******************************************************************************/

static void dis_int_sio(NOSIO_HandleTypeDef* nosio)
{
    USCI_A_UART_disableInterrupt(
            nosio->reg,
            USCI_A_UART_TRANSMIT_INTERRUPT
    );
    USCI_A_UART_disableInterrupt(
            nosio->reg,
            USCI_A_UART_RECEIVE_INTERRUPT
    );
}

/*****************************************************************************
* �V���A�������݋��i�����֐��j
*
******************************************************************************/

static void ena_int_sio(NOSIO_HandleTypeDef* nosio)
{
    USCI_A_UART_enableInterrupt(
            nosio->reg,
            USCI_A_UART_TRANSMIT_INTERRUPT
    );
    USCI_A_UART_enableInterrupt(
            nosio->reg,
            USCI_A_UART_RECEIVE_INTERRUPT
    );
}

/*****************************************************************************
* ���M�o�b�t�@�N���A�i�����֐��j
*
******************************************************************************/

static void clr_txbuf(NOSIO_HandleTypeDef* nosio)
{
	/* ���M�o�b�t�@�N���A */

	nosio->sio.txcnt = 0;
	nosio->sio.txgetp = nosio->sio.txputp;
}

/*****************************************************************************
* ��M�o�b�t�@�N���A�i�����֐��j
*
******************************************************************************/
static void clr_rxbuf(NOSIO_HandleTypeDef* nosio)
{
	/* ��M�o�b�t�@�N���A */

	nosio->sio.rxsts = 0;
	nosio->sio.oldsts = 0;
	nosio->sio.eotcnt = 0;
	nosio->sio.rxcnt = 0;
	nosio->sio.rxgetp = nosio->sio.rxputp;
}

/*****************************************************************************
* ���M�o�b�t�@�ւP�����i�[�i�����֐��j
*
* �o�b�t�@���t�Ŋi�[�ł��Ȃ������ꍇ�́AFALSE ��Ԃ��B
******************************************************************************/
static void tx_ctx(NOSIO_HandleTypeDef* nosio);

static int put_txbuf(NOSIO_HandleTypeDef* nosio, uint8_t c)
{
	uint8_t *p;

	/* �o�b�t�@�������� + 1 */
	/* �o�b�t�@���t�`�F�b�N */

	if (++nosio->sio.txcnt > TXBUFSZ)
	{
		nosio->sio.txcnt = TXBUFSZ;
		return false;
	}

	/* �o�b�t�@�֊i�[ */

	p = nosio->sio.txputp;
	*p = c;

	/* �i�[�|�C���^���P�i�߂� */

	if (++p >= nosio->sio.txbuf + TXBUFSZ)
		p = nosio->sio.txbuf;
	nosio->sio.txputp = p;

	/* ���M�����݋��� */
    USCI_A_UART_enableInterrupt(
            nosio->reg,
            USCI_A_UART_TRANSMIT_INTERRUPT
    );

    if (tx_ready) {
        tx_ready = false;
        tx_ctx(nosio);
    }

	return true;
}

/*****************************************************************************
* ��M�o�b�t�@�֎�M����/�X�e�[�^�X�i�[�i�����֐��j
*
******************************************************************************/

static void put_rxbuf(NOSIO_HandleTypeDef* nosio)
{
	int cnt;
	uint8_t *p;

	/* �A���u���[�N�Ȃ�i�[���Ȃ� */

	if ((nosio->sio.rxsts & UCBRK) && (nosio->sio.oldsts & UCBRK))
		return;

	/* �o�b�t�@���t�`�F�b�N */

	cnt = nosio->sio.rxcnt;
	if (cnt == BUFSZ)
		return;
	if (++cnt == BUFSZ)
		nosio->sio.rxsts |= UCBUSY;	/* �I�[�o�t���[�� UCBUSY �Ɋ��蓖�� */

	/* �o�b�t�@�������� + 1 */

	nosio->sio.rxcnt = (uint16_t)cnt;

	/* �o�b�t�@�֊i�[ */
	/* �I�[�������o+1 */

	p = nosio->sio.rxputp;
	if ((*p = nosio->sio.rxchr) == nosio->sio.eot)
		nosio->sio.eotcnt++;
	*(p + BUFSZ) = nosio->sio.rxsts;

	/* �i�[�|�C���^���P�i�߂� */

	if (++p >= nosio->sio.rxbuf + BUFSZ)
		p = nosio->sio.rxbuf;
	nosio->sio.rxputp = p;
}

/*****************************************************************************
* ��M�o�b�t�@����P�����擾�i�����֐��j
*
* �o�b�t�@��Ŏ擾�ł��Ȃ������ꍇ�́A-1 ��Ԃ��B
******************************************************************************/

static int get_rxbuf(NOSIO_HandleTypeDef* nosio, uint8_t *c)
{
	int cnt;
	int sts;
	uint8_t *p;

	/* ��M�o�b�t�@��`�F�b�N */

	cnt = nosio->sio.rxcnt;
	if (--cnt == -1)
		return cnt;

	/* ��M�o�b�t�@�������� - 1 */

	nosio->sio.rxcnt = (uint16_t)cnt;


	/* ��M�o�b�t�@����擾 */
	/* �I�[�������o��-1 */

	p = nosio->sio.rxgetp;
	if ((*c = *p) == nosio->sio.eot)
		nosio->sio.eotcnt--;
	sts = *(p + BUFSZ);

	/* �擾�|�C���^���P�i�߂� */

	if (++p >= nosio->sio.rxbuf + BUFSZ)
		p = nosio->sio.rxbuf;
	nosio->sio.rxgetp = p;

	return sts;
}

/*****************************************************************************
* ��M�����݃n���h���{�́i�����֐��j
*
******************************************************************************/

static void rx_int(NOSIO_HandleTypeDef* nosio)
{
	int sts;
	int chr;
	int tid;

	/* ��M�X�e�[�^�X�Ǝ�M��������� */

	sts = USCI_A_UART_queryStatusFlags(nosio->reg, (UCBRK|UCOE|UCFE|UCPE));
	chr = USCI_A_UART_receiveData(nosio->reg);

	nosio->sio.oldsts = nosio->sio.rxsts;		/* �O��̎�M�X�e�[�^�X�L�� */
	nosio->sio.rxsts = (uint8_t)sts;
	nosio->sio.rxchr = (uint8_t)chr;

	/* ��M�o�b�t�@�֊i�[ */

	put_rxbuf(nosio);

	/* ��M�҂����� */

	if ((tid = nosio->sio.rxtid) != 0)
	{
		nosio->sio.rxtid = 0;
		iwup_tsk((ID)tid);
	}
}

/*****************************************************************************
* ���M�����݃n���h���{�́i�����֐��j
*
******************************************************************************/

static void tx_int(NOSIO_HandleTypeDef* nosio)
{
	uint8_t *p;
	int tid;

	/* ���M�o�b�t�@�������� - 1 */
	/* ���M�o�b�t�@��Ȃ瑗�M�s�� */

	if (--nosio->sio.txcnt == (uint16_t)-1)
	{
		nosio->sio.txcnt = 0;
	    /* ���M�I���҂����� */
	    if ( nosio->sio.tetid != 0 )
	        iwup_tsk(nosio->sio.tetid);
		/* ���M�����݋֎~ */
        USCI_A_UART_disableInterrupt(
                nosio->reg,
                USCI_A_UART_TRANSMIT_INTERRUPT
        );
        tx_ready = true;
		return;
	}

	/* �P�������M */

	p = nosio->sio.txgetp;
	USCI_A_UART_transmitData(nosio->reg, *p);

	/* �擾�|�C���^���P�i�߂� */

	if (++p >= nosio->sio.txbuf + TXBUFSZ)
		p = nosio->sio.txbuf;
	nosio->sio.txgetp = p;

	/* ���M�҂����� */

	if ((tid = nosio->sio.txtid) != 0)
	{
		nosio->sio.txtid = 0;
		iwup_tsk((ID)tid);
	}
}

static void tx_ctx(NOSIO_HandleTypeDef* nosio)
{
    uint8_t *p;

    /* �P�������M */

    p = nosio->sio.txgetp;
    USCI_A_UART_transmitData(nosio->reg, *p);

    /* �擾�|�C���^���P�i�߂� */

    if (++p >= nosio->sio.txbuf + TXBUFSZ)
        p = nosio->sio.txbuf;
    nosio->sio.txgetp = p;
}

/*****************************************************************************
* �����݃n���h���{��
*
******************************************************************************/

//void nosio_isr(intptr_t exinf)
void NOSIO_IRQHandler(NOSIO_HandleTypeDef* nosio)
{
    ID tid;

    if (USCI_A_UART_getInterruptStatus(nosio->reg, USCI_A_UART_RECEIVE_INTERRUPT_FLAG)) {
        rx_int(nosio);
    } else {
        USCI_A_UART_clearInterrupt(nosio->reg, USCI_A_UART_TRANSMIT_INTERRUPT_FLAG);
        //tx_int(nosio);
        if ((tid = nosio->sio.txtid) != 0)
        {
            nosio->sio.txtid = 0;
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

ER ini_sio(NOSIO_HandleTypeDef* nosio, const char *param)
{
	uint32_t baud;
	uint8_t parity;
	uint8_t numberofStopBits;

	dis_int_sio(nosio);

	/* ����u���b�N������ */

	init_buf(nosio);

	/* �p�����[�^��� */

	if (!set_param(param, &parity, &numberofStopBits, &baud))
		return E_PAR;

	/* �f�o�C�X������ */

	if (!init_sio(nosio, parity, numberofStopBits, baud))
		return E_PAR;

	/* �����݃n���h���̒�` */
	nosio->sio.flag |= TSF_INIT;

	/* ��M�����݋��� */
/*
    USCI_A_UART_enableInterrupt(
            nosio->reg,
            USCI_A_UART_RECEIVE_INTERRUPT
    );
*/
	ena_int_sio(nosio);

	return E_OK;
}

/*****************************************************************************
* �V���A�����o�͏I��
*
******************************************************************************/

void ext_sio(NOSIO_HandleTypeDef* nosio)
{
	if (!(nosio->sio.flag & TSF_INIT))	/* ���������Ȃ牽�����Ȃ� */
		return;
	dis_int_sio(nosio);					/* �V���A�������݋֎~ */

	nosio->sio.flag &= ~TSF_INIT;		/* �������ς݃t���O�N���A */
}

/*****************************************************************************
* �V���A���P��������
*
******************************************************************************/

ER get_sio(NOSIO_HandleTypeDef* nosio, uint8_t *c, TMO tmout)
{
	ER ercd;
	int sts;
	ER_UINT wupcnt;

	for (;;)
	{
		/* ��M�o�b�t�@����P�������� */

		sts = get_rxbuf(nosio, c);

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
		get_tid(&nosio->sio.rxtid);
		Asm("nop");
		Asm("eint");
		Asm("nop");
		ercd = tslp_tsk(tmout);
		nosio->sio.rxtid = 0;
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

ER put_sio(NOSIO_HandleTypeDef* nosio, uint8_t c, TMO tmout)
{
	ER ercd;
	ER_UINT wupcnt;

    Asm("nop");
    Asm("dint");
    Asm("nop");
	USCI_A_UART_transmitData(nosio->reg, c);
    get_tid(&nosio->sio.txtid);
    Asm("nop");
    Asm("eint");
    Asm("nop");
    ercd = tslp_tsk(tmout);
    nosio->sio.txtid = 0;
    // ������ iwup_tsk ���ꂽ�ꍇ�̑΍�
    do {
        wupcnt = can_wup(TSK_SELF);
    } while (wupcnt);
    if (ercd)
        return ercd;    /* �^�C���A�E�g�I�� */

#if 0
	for (;;)
	{
	    Asm("nop");
	    Asm("dint");
	    Asm("nop");

		/* ���M�o�b�t�@�ւP�����i�[ */

		if (put_txbuf(nosio, c))	/* �i�[�ł����ꍇ */
		{
            Asm("nop");
            Asm("eint");
            Asm("nop");
			return E_OK;	/* ����I�� */
		}

		/* ���M�����ݑ҂� */

        get_tid(&nosio->sio.txtid);
        Asm("nop");
        Asm("eint");
        Asm("nop");
        ercd = tslp_tsk(tmout);
		nosio->sio.txtid = 0;
		// ������ iwup_tsk ���ꂽ�ꍇ�̑΍�
		do {
			wupcnt = can_wup(TSK_SELF);
		} while (wupcnt);
		if (ercd)
			return ercd;	/* �^�C���A�E�g�I�� */
	}
#endif
}

/*****************************************************************************
* �V���A�����o�͐���
*
******************************************************************************/

ER ctl_sio(NOSIO_HandleTypeDef* nosio, uint16_t fncd)
{
    Asm("nop");
    Asm("dint");
    Asm("nop");

	/* �o�b�t�@�N���A */

	if (fncd & TSIO_RXCLR)
		clr_rxbuf(nosio);
	if (fncd & TSIO_TXCLR)
		clr_txbuf(nosio);

    Asm("nop");
    Asm("eint");
    Asm("nop");

    return E_OK;
}

/*****************************************************************************
* �V���A�����o�͏�ԎQ��
*
******************************************************************************/

ER ref_sio(NOSIO_HandleTypeDef* nosio, T_SIOS *pk_sios)
{
	int stat;

    Asm("nop");
    Asm("dint");
    Asm("nop");

	stat = get_stat(nosio);
	if (nosio->sio.txcnt != 0)
		stat &= ~TSIO_TXEMP;

	pk_sios->siostat = (uint8_t)stat;
	pk_sios->rxlen   = nosio->sio.rxcnt;
	pk_sios->frbufsz = (uint16_t)(TXBUFSZ - nosio->sio.txcnt);
	pk_sios->eotcnt  = nosio->sio.eotcnt;

    Asm("nop");
    Asm("eint");
    Asm("nop");

    return E_OK;
}

/*****************************************************************************
* �V���A�����M�o�b�t�@�t���b�V��
*
******************************************************************************/

ER fls_sio(NOSIO_HandleTypeDef* nosio, TMO tmout)
{
	ER ercd;
	ER_UINT wupcnt;

	for (;;)
	{
		if (nosio->sio.txcnt == 0)
		{
	        Asm("nop");
	        Asm("eint");
	        Asm("nop");
			break;	/* ����I�� */
		}

		/* ���M�I�������ݑ҂� */

		get_tid(&nosio->sio.tetid);
        Asm("nop");
        Asm("eint");
        Asm("nop");
		ercd = tslp_tsk(tmout);
		nosio->sio.tetid = 0;
		// ������ iwup_tsk ���ꂽ�ꍇ�̑΍�
		do {
			wupcnt = can_wup(TSK_SELF);
		} while (wupcnt);
		if (ercd)
			return ercd;	/* �^�C���A�E�g�I�� */
	}

	return E_OK;
}
