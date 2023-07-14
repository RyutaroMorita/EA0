//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//
//		MSP430�p�V���A���ʐM�h���C�o
//		t_usci.h (Ver.00.00.00)
//
//		2023/07/13	Ver.00.00.00	����
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#ifndef T_USCI_H
#define T_USCI_H


#define NORTiAPI    /**/
#define cdecl       /**/

/* �G���[�R�[�h */

#define EV_SIOINI   1           /* �������� */
#define EV_SIOOVF   2           /* ��M�o�b�t�@�I�[�o�t���[ */
#define EV_SIOPTY   3           /* �p���e�B�G���[ */
#define EV_SIOORN   4           /* �I�[�o�[�����G���[ */
#define EV_SIOFRM   5           /* �t���[�~���O�G���[ */
#define EV_SIOBRK   6           /* �u���[�N���o */

/* �@�\�R�[�h */

#define TUSCI_RXE    0x01       /* ��M�C�l�[�u�� */
#define TUSCI_RXD    0x02       /* ��M�f�B�Z�[�u�� */
#define TUSCI_TXE    0x04       /* ���M�C�l�[�u�� */
#define TUSCI_TXD    0x08       /* ���M�f�B�Z�[�u�� */
#define TUSCI_RTSON  0x10       /* RTS�M��ON */
#define TUSCI_RTSOFF 0x20       /* RTS�M��OFF */
#define TUSCI_DTRON  0x40       /* DTR�M��ON */
#define TUSCI_DTROFF 0x80       /* DTR�M��OFF */
#define TUSCI_RXCLR  0x0100     /* ��M�o�b�t�@�N���A */
#define TUSCI_TXCLR  0x0200     /* ���M�o�b�t�@�N���A */
#define TUSCI_SBON   0x0400     /* �u���[�N���MON */
#define TUSCI_SBOFF  0x0800     /* �u���[�N���MOFF */

typedef struct t_uscis
{
    uint8_t siostat;            /* �V���A�����o�̓X�e�[�^�X */

#define TUSCI_CD     0x01       /* ��M�L�����A���o */
#define TUSCI_CTS    0x02       /* CTS�M��ON(1)/OFF(0) */
#define TUSCI_TXEMP  0x04       /* ���M�o�b�t�@�� */
#define TUSCI_PE     0x08       /* �p���e�B�G���[ */
#define TUSCI_OE     0x10       /* �I�[�o�����G���[ */
#define TUSCI_FE     0x20       /* �t���[�~���O�G���[ */
#define TUSCI_BD     0x40       /* �u���[�N��Ԍ��o */
#define TUSCI_DSR    0x80       /* DSR�M��ON(1)/OFF(0) */

    uint8_t rxchr;              /* ��M�o�b�t�@�̐擪�̕����i���g�p�j*/
    uint16_t rxlen;             /* ��M�o�b�t�@�̃f�[�^�� */
    uint16_t frbufsz;           /* ���M�o�b�t�@�̋󂫃T�C�Y */
    uint16_t eotcnt;            /* ��M�o�b�t�@�̏I�[������ */

} T_USCIS;

/* �V���A�����o�͐���u���b�N�\���� */

typedef struct t_uscib
{
	uint8_t ch;                 /* �`���l���ԍ� */
	uint8_t flag;               /* ����t���O */

#define TSF_INIT    0x01        /* �������ς� */
#define TSF_TXREQ   0x02        /* XON/XOFF���M�v�� */
#define TSF_RXOFF   0x04        /* XOFF��M���� */
#define TSF_TXOFF   0x08        /* XOFF���M���� */
#define TSF_XON     0x10        /* XON/OFF�ɂ��t���[����L�� */
#define TSF_DTR     0x20        /* DTR�ɂ��t���[����L�� */
#define TSF_RTS     0x40        /* RTS�ɂ��t���[����L�� */

	uint8_t txchr;              /* ���M���� */
	uint8_t rxchr;              /* ��M���� */
	uint8_t rxsts;              /* ��M�X�e�[�^�X */
	uint8_t oldchr;             /* �O��̎�M���� */
	uint8_t oldsts;             /* �O��̎�M�X�e�[�^�X */

	uint8_t eot;                /* �I�[���� */
	uint16_t eotcnt;            /* �I�[�������o�J�E���^ */

	ID txtid;                   /* ���M�҂��^�X�N�h�c */
	ID rxtid;                   /* ��M�҂��^�X�N�h�c */
	ID tetid;                   /* ���M�I���҂��^�X�N�h�c */

	uint8_t cmd[6];             /* SIO�R�}���h�o�b�t�@ */
	uint8_t rsv[2];             /* �\�� */

	uint16_t rxcnt;             /* ��M�o�b�t�@���̕����� */

	uint8_t *rxbuf;             /* ��M�o�b�t�@�i+BUFSZ:��M�X�e�[�^�X�o�b�t�@�j*/
	uint8_t *rxputp;            /* ��M�o�b�t�@�i�[�|�C���^ */
	uint8_t *rxgetp;            /* ��M�o�b�t�@�擾�|�C���^ */

  #ifdef NORTi86
	int16_t far *vram;          /* ���j�^�\���p�e�L�X�gVRAM�A�h���X */
  #endif

} T_USCIB;

/* ����M�o�b�t�@�T�C�Y�̒�` */

#ifndef BUFSZ
#define BUFSZ       256         /* ��M�o�b�t�@�� */
#endif

/*  */
typedef struct _USCI_HandleTypeDef
{
	uint32_t reg;
	uint8_t rxbuf[BUFSZ*2];		/* ��M�o�b�t�@, ��M�X�e�[�^�X�o�b�t�@ */
	T_USCIB uscib;
} USCI_HandleTypeDef;

/* �֐��v���g�^�C�v */

ER cdecl ini_sio(USCI_HandleTypeDef*, const char *);
void cdecl ext_sio(USCI_HandleTypeDef*);
ER cdecl get_sio(USCI_HandleTypeDef*, uint8_t *, TMO);
ER cdecl put_sio(USCI_HandleTypeDef*, uint8_t, TMO);
ER cdecl ctl_sio(USCI_HandleTypeDef*, uint16_t);
ER cdecl ref_sio(USCI_HandleTypeDef*, T_USCIS *);

void usci_isr(USCI_HandleTypeDef* nosio);

#endif /* T_USCI_H */
