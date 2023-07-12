$
$ 		パス2のターゲット依存テンプレート（msp-exp430f5529lp用）
$

$
$  有効な割込み番号，割込みハンドラ番号
$
$INTNO_VALID = { 0,1,...,62 }$
$INHNO_VALID = INTNO_VALID$

$
$  ATT_ISRで使用できる割込み番号とそれに対応する割込みハンドラ番号
$
$INTNO_ATTISR_VALID = INTNO_VALID$
$INHNO_ATTISR_VALID = INHNO_VALID$

$
$  DEF_INT／DEF_EXCで使用できる割込みハンドラ番号／CPU例外ハンドラ番号
$
$INHNO_DEFINH_VALID = INHNO_VALID$

$
$  CFG_INTで使用できる割込み番号と割込み優先度
$
$INTNO_CFGINT_VALID = INTNO_VALID$
$INTPRI_CFGINT_VALID = { -7,-6,...,-1 }$

$
$  標準テンプレートファイルのインクルード
$
$INCLUDE "kernel/kernel.tf"$

/*$NL$
$SPC$*  Target-dependent Definitions (msp-exp430f5529lp)$NL$
$SPC$*/$NL$
$NL$

$
$   CFG_INTのターゲット依存のエラーチェックと_kernel_bitpat_cfgintの生成
$
$bitpat_cfgint = 0$
const uint32_t	_kernel_bitpat_cfgint = UINT32_C($FORMAT("0x%08x", bitpat_cfgint)$);
$NL$

$NL$
$FOREACH inhno INTNO_VALID$
	extern void $FORMAT("_kernel_int_handler_%d(void);", +inhno)$$NL$
	asm(".section .handlers,\"a\" \n\t"$NL$
	$FORMAT("\"_kernel_int_handler_%d: \\n\\t\"", +inhno)$$NL$
	"	pushm.a #1,r15 \n\t"$NL$
	$FORMAT("\"	mov.w #%d,r15 \\n\\t\"", +inhno)$$NL$
	"	bra #inthdr_entry \n\t");$NL$
	$NL$
$END$

const FP _kernel_exc_tbl[] = $NL$
{$NL$
$FOREACH inhno INTNO_VALID$
	$IF LENGTH(INH.INHNO[inhno])$
		$TAB$(FP)($INH.INTHDR[inhno]$),
	$ELSE$
		$TAB$(FP)(_kernel_default_int_handler),
	$END$
	$SPC$$FORMAT("/* %d */", +inhno)$$NL$
$END$
};$NL$

$NL$
asm(".section .vectors,\"a\" \n\t"$NL$
$FOREACH inhno INTNO_VALID$
	$FORMAT("\"	.word _kernel_int_handler_%d \\n\\t\"", +inhno)$$NL$
$END$
"	.word start \n\t"$NL$
".section .text \n\t"$NL$
);$NL$
