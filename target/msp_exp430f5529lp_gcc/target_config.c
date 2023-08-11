/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2011 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 *
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *	2022/10/12 Ryutaro Morita
 */

/*
 *		ターゲット依存モジュール（msp-exp430f5529lp用）
 */

#include "kernel_impl.h"
#include <sil.h>
#include <msp430.h>
#include "driverlib.h"

/*
 *  プロセッサ識別のための変数（マルチプロセッサ対応）
 */
uint_t	board_id;			/* ボードID */
void	*board_addr;		/* ローカルメモリの先頭アドレス */

/*
 *  起動時のハードウェア初期化処理
 */
void
hardware_init_hook(void) {
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);
}

void
usart_early_init()
{
	GPIO_setAsPeripheralModuleFunctionInputPin(
			GPIO_PORT_P4,
			GPIO_PIN4+GPIO_PIN5
	);

	// From Table 36-4 in the family user's manual where UCOS16 = 0 and
	//            baudrate = 9600
	//            clock freq = 4MHz
	// UCBRx = 416, UCBRFx = 6, UCBRSx = 0, UCOS16 = 0
	// http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
	USCI_A_UART_initParam param = {0};
	param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
	param.clockPrescalar = 416;                                             // UCBRx
	param.firstModReg = 6;                                                  // UCBRFx
	param.secondModReg = 0;                                                 // UCBRSx
	param.parity = USCI_A_UART_NO_PARITY;
	param.msborLsbFirst = USCI_A_UART_LSB_FIRST;
	param.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
	param.uartMode = USCI_A_UART_MODE;
	param.overSampling = USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;     // UCOS16 = 0

	USCI_A_UART_init(USCI_A1_BASE, &param);

	//Enable UART module for operation
	USCI_A_UART_enable(USCI_A1_BASE);
}

/*
 *  ターゲット依存の初期化
 */
// for Debug
uint32_t mclk = 0;
uint32_t smclk = 0;
uint32_t aclk = 0;
void
target_initialize(void)
{
	/*
	 *  プロセッサ依存の初期化
	 */
	prc_initialize();

	/*
	 *  コア電力モードの初期化
	 */
	// up to  8 MHz => PMM_CORE_LEVEL_0
	// up to 12 MHz => PMM_CORE_LEVEL_1
	// up to 20 MHz => PMM_CORE_LEVEL_2
	// up to 25 MHz => PMM_CORE_LEVEL_3
	PMM_setVCore(PMM_CORE_LEVEL_3);

    /*
     *  GPIOの初期化
     */
	//P1.0 output ACLK
    // Important First Steps: Configure Pins for Crystals!
    // All to port P5
    // PIN5 -> XT1 OUT
    // PIN4 -> XT1 IN
    // PIN3 -> XT2 OUT
    // PIN2 -> XT2 IN
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P5,
        GPIO_PIN4+GPIO_PIN2
    );
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        GPIO_PORT_P5,
        GPIO_PIN5+GPIO_PIN3
    );

    /*
     *  クロックの初期化
     */
	// Inform the system of the crystal frequencie
    UCS_setExternalClockSource(
       32768,               // Frequency of XT1 in Hz.
       4000000              // Frequency of XT2 in Hz.
    );

    // Initialize the crystals
    UCS_turnOnXT2(
       UCS_XT2_DRIVE_4MHZ_8MHZ
    );
    UCS_turnOnLFXT1(
        UCS_XT1_DRIVE_0,
        UCS_XCAP_3
    );

    UCS_initClockSignal(
        UCS_FLLREF,         // The reference for Frequency Locked Loop
        UCS_XT2CLK_SELECT,  // Select XT2
//        UCS_XT1CLK_SELECT,  // Select XT1
//        UCS_DCOCLK_SELECT,
        UCS_CLOCK_DIVIDER_4 // FLL ref. will be 1MHz (4MHz XT2/4)
    );
    UCS_initFLLSettle(
        24000000/1000,
        24000000/(4000000/4)
//        25000000/(32000)
    );

    // MCLK = 24MHz
    UCS_initClockSignal(
        UCS_MCLK,
        UCS_DCOCLK_SELECT,
        UCS_CLOCK_DIVIDER_1
    );

    // SMCLK = 4MHz
    UCS_initClockSignal(
        UCS_SMCLK,
        UCS_XT2CLK_SELECT,
        UCS_CLOCK_DIVIDER_1
    );

    // ACLK = 32.768kHz
    UCS_initClockSignal(
        UCS_ACLK,
        UCS_XT1CLK_SELECT,
//        UCS_XT2CLK_SELECT,
        UCS_CLOCK_DIVIDER_1
    );

    /*
     *  クロックリクエスト機能の無効化
     */
    UCS_disableClockRequest(UCS_ACLK);
    UCS_disableClockRequest(UCS_SMCLK);
    UCS_disableClockRequest(UCS_MCLK);
    UCS_disableClockRequest(UCS_MODOSC);

    // for Debug
    aclk=UCS_getACLK();
    mclk=UCS_getMCLK();
    smclk=UCS_getSMCLK();

    usart_early_init();
}

/*
 *  ターゲット依存の終了処理
 */
void
target_exit(void)
{
	/*
	 *  プロセッサ依存の終了処理
	 */
	prc_terminate();
	while(1);
}

/*
 *  システムログの低レベル出力のための文字出力
 */
void
target_fput_log(char c)
{
    if (c == '\n') {
        USCI_A_UART_transmitData(USCI_A1_BASE, '\r');
    }
    USCI_A_UART_transmitData(USCI_A1_BASE, c);
    while (USCI_A_UART_queryStatusFlags(USCI_A1_BASE, USCI_A_UART_BUSY));
}

/*
 *  割込み要求ラインの属性の設定
 *
 *  ASPカーネルでの利用を想定して，パラメータエラーはアサーションでチェッ
 *  クしている．cfg_intサービスコールを設ける場合には，エラーを返すよう
 *  にすべきであろう．
 */
void
x_config_int(INTNO intno, ATR intatr, PRI intpri)
{
	assert(VALID_INTNO_CFGINT(intno));
	assert(TMIN_INTPRI <= intpri && intpri <= TMAX_INTPRI);
}
