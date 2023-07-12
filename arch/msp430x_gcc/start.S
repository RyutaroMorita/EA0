/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005,2006 by Embedded and Real-Time Systems Laboratory
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
 *		カーネル対応のスタートアップモジュール（msp430x用）
 */
#define	TOPPERS_MACRO_ONLY
#include "kernel_impl.h"

	.section .handlers,"a"
	.global	start
start:
	/*
	 *  プロセッサモードとスタックポインタの初期化
	 */
;	mova	#0x4400,r1
	mova	&_kernel_istkpt,sp

	/*
	 *  hardware_init_hookの呼出し（0でない場合）
	 *
	 *  ターゲットハードウェアに依存して必要な初期化処理がある場合は，
	 *  hardware_init_hookという関数を用意する．
	 */
	mova	#hardware_init_hook,r12
	tsta	r12
	jeq		0f
	calla	r12
0:

	/*
	 *  dataセクションの初期化（ROM化対応）
	 */
	mova	#__idata_start, r12
	mov.w	#__data_start, r13
	mov.w	#__data_end, r14
	jmp		2f
1:
	mov.w	@r12+,r15
	mov.w	r15,@r13
	add.w	#2,r13
2:
	cmp		r13,r14
	jne		1b

	/*
	 *  bssセクションのクリア
	 */
	mova	#__bss_start, r13
	mov.w	#__bss_end, r14
	clr.w	r12
	jmp		4f
3:
	mov.w	r12,@r13
	add.w	#2,r13
4:
	cmp		r13,r14
	jne		3b

	/*
	 *  software_init_hookの呼出し（0でない場合）
	 *
	 *  ソフトウェア環境（特にライブラリ）に依存して必要な初期化処理が
	 *  ある場合は，software_init_hookという関数を用意すればよい．
	 */
	mova	#software_init_hook,r12
	tsta	r12
	jeq		5f
	calla	r12
5:

	bra		#sta_ker
