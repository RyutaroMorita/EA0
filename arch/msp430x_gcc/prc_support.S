/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2010 by Embedded and Real-Time Systems Laboratory
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
 *		プロセッサ依存モジュール アセンブリ言語部（msp430x用）
 */

#define	TOPPERS_MACRO_ONLY
#define UINT_C(val)			(val)		/* uint_t型の定数を作るマクロ */
#define ULONG_C(val)		(val)		/* ulong_t型の定数を作るマクロ */
#define CAST(type, val)		(val)		/* 型キャストを行うマクロ */
#include "kernel_impl.h"
#include "offset.h"

/*
 *  タスクディスパッチャ
 */
	.text
	.globl dispatch
dispatch:
	/*
	 *  このルーチンは，タスクコンテキスト・CPUロック状態・ディスパッチ
	 *  許可状態・（モデル上の）割込み優先度マスク全解除状態で呼び出さ
	 *  れる．
	 */
	pushm.a	#8,r11						/* レジスタを保存(r11,r10,r9,r8,r7,r6,r5,r4) */
	mova	&p_runtsk,r12				/* p_runtskをr12に */
	mova	sp,TCB_msp(r12)				/* タスクスタックを保存 */
	mova	#dispatch_r,r13
	mova	r13,TCB_pc(r12)				/* 実行再開番地を保存 */
	jmp		dispatcher

dispatch_r:
	popm.a	#8,r11						/* レジスタを復帰(r4,r5,r6,r7,r8,r9,r10,r11) */
	bit.w	#TCB_enatex_bit, TCB_enatex(r12)
	jeq		dispatch_r_1				/* enatexがfalseならリターン */
	tst.w	TCB_texptn(r12)				/* texptnが0ならリターン */
	jeq		dispatch_r_1
	tst.w	&ipmflg						/* ipmflgがtrueであれば            */
	jeq		dispatch_r_1
	bra		#call_texrtn				/*  タスク例外処理ルーチンの呼出し */

dispatch_r_1:
	reta

;	rts

/*
 *  ディスパッチャの動作開始（prc_support.S）
 */
	.globl start_dispatch
start_dispatch:
	/*
	 *  このルーチンは，カーネル起動時に，すべての割込みを禁止した状態
	 * （割込みロック状態と同等）で呼び出される．また，割込みモード（非
	 *  タスクコンテキストと同等）で呼び出されることを想定している．
	 *
	 *  prc_initializeで，lock_flagをtrueに，saved_iipmをIIPM_ENAALLに
	 *  初期化しているため，カーネル管理外の割込みを許可することで，
	 *  CPUロック状態・（モデル上の）割込み優先度マスク全解除状態になる．
	 *  また，initialize_taskでdisdspをfalseに初期化しているため，ディ
	 *  スパッチ許可状態になっている．
	 */
	clr.w	r12							/* マスタモードに */
	mov.w	r12,&int_cnt
	jmp		dispatcher_0

/*
 *  現在のコンテキストを捨ててディスパッチ
 */
	.globl exit_and_dispatch
exit_and_dispatch:
	/* ディスパッチャ本体（dispatcher）へ */

/*
 *  ディスパッチャ本体
 */
dispatcher:
	/*
	 *  このルーチンは，タスクコンテキスト・CPUロック状態・ディスパッチ
	 *  許可状態・（モデル上の）割込み優先度マスク全解除状態で呼び出さ
	 *  れる．
	 *
	 *  すなわち，マスタモード・lock_flagがtrue・disdspがfalse・dspflg
	 *  がtrue・saved_iipmがIIPM_ENAALLとなっている．実行再開番地へもこ
	 *  の状態のまま分岐する．
	 */
#ifdef LOG_DSP_ENTER
	mova	&p_runtsk,r12				/* p_runtskをパラメータに */
	calla	#log_dsp_enter

#endif /* LOG_DSP_ENTER */
dispatcher_0:
	mova	&p_schedtsk,r12				/* p_schedtskをp_runtskに */
	mova	r12,&p_runtsk
	tst.a	r12							/* p_runtskがNULLならdispatcher_1へ */
	jeq		dispatcher_1
	mova	TCB_msp(r12),sp				/* タスクスタックを復帰 */
#ifdef LOG_DSP_LEAVE
	mova	&p_runtsk,r12				/* p_runtskをパラメータに */
	calla	#log_dsp_leave
	mova	&p_runtsk,r12
#endif /* LOG_DSP_LEAVE */
	mova	TCB_pc(r12), r13
	bra		r13

dispatcher_1:
	/*
	 *  CPUロック状態を解除する準備をする．
	 */
	mov.w	#1,&int_cnt					/* 割込みモードに */					/* 7/13 修正 */
	mova	&_kernel_istkpt,sp			/* 割込み用のスタックへ切替え */		/* 7/13 修正 */
	mov.w	sr,r12
	clr.w	&lock_flag

dispatcher_2:
	/*
	 *  割込みを許可し，割込みモードに切り換えて，割込みを待つ．
	 *
	 *  ここで割込みモードに切り換えるのは，ここで発生する割込み処理に
	 *  どのスタックを使うかという問題の解決と，割込みハンドラ内でのタ
	 *  スクディスパッチの防止という2つの意味がある．
	 *
	 *  プロセッサを割込み待ちに移行させる処理と，割込み許可とは，不可
	 *  分に行なう必要がある．これを不可分に行なわない場合，割込みを許
	 *  可した直後に割込みが入り，その中でタスクが実行可能状態になると
	 *  実行すべきタスクがあるにもかかわらずプロセッサが割込み待ちにな
	 *  ってしまう．
	 *
	 *  割込み待ちの間は，p_runtskをNULL（＝0）に設定しなければならな
	 *  い．このように設定しないと，割込みハンドラからiget_tidを呼び出
	 *  した際の動作が仕様に合致しなくなる．
	 */
	nop									/* 割込み待ち */
//	eint
	bis.w	#0x0058,sr					/* 割り込み許可とLPM1への移行を不可分に行う */
										/* タイマにSMCLKを使用しているためLPM1までしか下げられない */
	nop
	dint
	nop
	mov.w	r12,sr						/* 元の状態に戻す */
	nop
	tst.w	&reqflg						/* reqflgがfalseならdispatcher_2へ */
	jeq		dispatcher_2
	clr.w	&reqflg						/* reqflgをfalseに */

	/*
	 *  CPUロック状態に戻す．割込み待ちの間に実行した割込みハンドラによ
	 *  り，saved_iipmが書き換えられる可能性があるため，元の値に戻す必
	 *  要がある．dispatcherが実行される時は，saved_iipmがIIPM_ENAALL
	 *  となっているため，ここではsaved_iipmをIIPM_ENAALL（＝0）に戻せ
	 *  ばよい．
	 */
	clr.w	r12							/* マスタモードに */					/* 7/13 修正 */
	mov.w	r12,&int_cnt														/* 7/13 修正 */
	clr.w	&saved_iipm					/* saved_iipmを0にする */
	mov.w	#1,&lock_flag				/* CPUロック状態に */
	jmp		dispatcher_0

/*
 *  カーネルの終了処理の呼出し
 */
	.globl call_exit_kernel
call_exit_kernel:
	mov.w	#1,&int_cnt					/* 割込みモードに */
;	mova	#0x4400,sp
	mova	&_kernel_istkpt,sp
	bra		#exit_kernel				/* カーネルの終了処理を呼ぶ */

/*
 *  タスク開始時処理
 *
 *  dispatcherから呼び出されるため，TCBのアドレスはr12に入っている
 */
	.text
	.globl start_r
start_r:
	clr.w	&lock_flag					/* CPUロック解除状態に */
	nop
	eint
	nop
	mova	TCB_p_tinib(r12),r13		/* p_runtsk->p_tinibをr13に */
	mov.w	TINIB_exinf(r13),r12
	mova	#ext_tsk,r14
	pushm.a	#1,r14
	mova	TINIB_task(r13), r14		/* タスクの起動番地をr14に */
	bra		r14

/*
 *  割込みエントリ（r15にエントリ番号が入る）
 */
	.global inthdr_entry
inthdr_entry:
	pushm.a	#3,r14						/* スクラッチレジスタを保存(r14,r13,r12) */

	mov.w	&int_cnt,r12
	inc.w	r12							/* 割込みモードに */
	mov.w	r12,&int_cnt
	cmp.w	#1,r12						/* 加算前が0でなければ多重割込み */
	jne		inthdr_entry_0_d

	mova	sp,&int_sp
;	mova	#0x4400,sp
	mova	&_kernel_istkpt,sp
	jmp		inthdr_entry_0

inthdr_entry_0_d:
	jmp		inthdr_entry_0

inthdr_entry_0:
;	nop
;	eint
;	nop
	mova	#_kernel_exc_tbl,r12
	mov.w	r15,r13
0:
	tst.w	r13
	jeq		1f

	add.w	#4,r12
	dec.w	r13
	jmp		0b

1:
#ifdef LOG_INH_LEAVE
	pushm.a	#1,r15						/* for Debug */
#endif

#ifdef LOG_INH_ENTER
	pushm.a	#4,r15						/* スクラッチレジスタを保存(r15,r14,r13,r12) */
	mov,w	r15,r12
	call	log_inh_enter				/* log_exc_enterを呼び出す */

	popm.a	#4,r15						/* スクラッチレジスタを復帰(r12,r13,r14,r15) */
#endif /* LOG_EXC_ENTER */

	calla	@r12						/* Cルーチン呼び出し */

#ifdef LOG_INH_LEAVE
	popm.a	#1,r15						/* スクラッチレジスタを復帰(r15) */
	mov,w	r15,r12
	call	log_inh_leave				/* log_exc_leaveを呼び出す */

#endif /* LOG_INH_LEAVE */

	nop
	eint
	nop

/*
 *  割込みハンドラ出口処理
 *
 *  ret_intは，割込みハンドラから戻った直後に実行するルーチンで，
 *  INTHDR_ENTRYマクロで展開する割込みハンドラの入口処理で，割込みハン
 *  ドラからの戻り番地に設定される．
 */
	.text
	.globl ret_int
ret_int:
	dint
	nop
//	popm.a	#1,r15						/* for Debug */
	mov.w	&int_cnt,r12				/* 戻り先が割込みモードなら */
	dec.w	r12
	mov.w	r12,&int_cnt
	tst.w	r12
	jne		ret_int_1_d					/*           すぐにリターン */

	/*
	 *  カーネル管理の割込みを禁止する．この時点では，CPUロック状態に
	 *  はならない（lock_flagとsaved_iipmは更新しない）．
	 *
	 *  reqflgをチェックする前に割込みを禁止するのは，reqflgをチェック
	 *  した直後に割込みハンドラが起動され，その中でディスパッチが要求
	 *  された場合に，すぐにディスパッチされないという問題が生じるため
	 *  である．
	 */

	/*
	 *  このルーチンはカーネル管理の割込みから起動されるため，ここでは，
	 *  割込み優先度マスクがTIPM_LOCKよりも低いと想定できる．
	 */
	tst.w	&reqflg						/* reqflgがtrueであればret_int_2へ */
	jne		ret_int_2

	mova	&int_sp,sp

ret_int_1:
	/*
	 *  割込み処理からのリターンにより，CPUロック解除状態に移行するよう
	 *  準備する．割込み優先度マスクは，RTE命令により元の値に戻るため，
	 *  lock_flagをfalseにしておけばよい（lock_flagをfalseにするのは，
	 *  CPUロック状態のまま割込みハンドラから戻った時の対策）．
	 */
	clr.w	&lock_flag					/* CPUロック解除の準備 */
	popm.a	#4,r15						/* スクラッチレジスタを復帰(r12,r13,r14,r15) */
	bic.w	#0x00F0,0(SP)				/* 割り込み復帰後はLPM1→Activeに事前に設定 */
	reti

ret_int_1_d:
	jmp		ret_int_1

ret_int_2:
	mova	&int_sp,sp

ret_int_3:
	/*
	 *  ここへは，CPU例外ハンドラの出口処理からも分岐してくる．
	 *
	 *  ここでは，戻り先がタスクであり，スタックは，例外スタックフレー
	 *  ムの上にスクラッチレジスタのみが保存された状態になっている．ま
	 *  た，プロセッサは，マスタモード・カーネル管理の割込みを禁止した
	 *  状態となっている．
	 */
	clr.w	&reqflg						/* reqflgをfalseに */

	/*
	 *  CPUロック状態に移行し，割込み優先度マスクを割込み処理前の値に設
	 *	定する．カーネル管理の割込みはすでに禁止しているので，lock_flag
	 *  とsaved_iipmを更新する．saved_iipmは，戻り先の割込み優先度マス
	 *  ク（の内部表現）に設定する．
	 *
	 *  この時点でCPUロック状態とするのは，dispatcherへ分岐する時と，
	 *  call_texrtnを呼び出す時に，CPUロック状態になっている必要がある
	 *  ためである．
	 */
	mov.w	#1,&lock_flag				/* lock_flagをtrueに */

	/*
	 *  dspflgがfalseである場合と，p_runtskとp_schedtskが同じ場合には，
	 *  ディスパッチを行わない．このチェックが必要なのは，タスク例外処
	 *  理ルーチンの呼出しが必要な場合に，ディスパッチが必要なくても，
	 *  reqflgをtrueにするためである．
	 */
	mova	&p_runtsk,r12				/* p_runtskをr12に */
	tst.w	&dspflg						/* dspflgがfalseならret_int_4へ */
	jeq		ret_int_4
	mova	&p_schedtsk,r13
	cmpa	r12,r13						/* p_runtskとp_schedtskが同じなら */
	jeq		ret_int_4					/*                    ret_int_4へ */

	pushm.a	#8,r11						/* 残りのレジスタを保存(r11,r10,r9,r8,r7,r6,r5,r4) */
	mova	sp,TCB_msp(r12)				/* タスクスタックを保存 */
	mova	#ret_int_r,r13
	mova	r13,TCB_pc(r12)				/* 実行再開番地を保存 */
	jmp		dispatcher

ret_int_r:
	popm.a	#8,r11						/* レジスタを復帰(r4,r5,r6,r7,r8,r9,r10,r11) */

ret_int_4:
	/*
	 *  enatexがtrueで，texptnが0でなく，ipmflgがtrueであれば，タスク
	 *  例外処理ルーチンを呼び出す．
	 */
	bit.w	#TCB_enatex_bit, TCB_enatex(r12)
	jeq		ret_int_5					/* enatexがfalseならret_int_5へ */
	tst.w	TCB_texptn(r12)				/* texptnが0ならret_int_5へ */
	jeq		ret_int_5
	tst.w	&ipmflg						/* ipmflgがfalseならret_int_5へ */
	jeq		ret_int_5
	calla	#call_texrtn				/*  タスク例外処理ルーチンの呼出し */

ret_int_5:
	/*
	 *  割込み処理からのリターンにより，CPUロック解除状態に移行するよ
	 *  う準備する．割込み優先度マスクは，RTE命令により元の値に戻るた
	 *  め，lock_flagをfalseにしておけばよい．
	 */
	clr.w	&lock_flag					/* CPUロック解除の準備 */
	popm.a	#4,r15						/* スクラッチレジスタを復帰(r12,r13,r14,r15) */
	bic.w	#0x00F0,0(SP)				/* 割り込み復帰後はLPM1→Activeに事前に設定 */
	reti
