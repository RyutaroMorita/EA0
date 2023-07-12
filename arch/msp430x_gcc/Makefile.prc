#
#		Makefileのプロセッサ依存部（msp430x用）
#

#
#  GNU開発環境のターゲットアーキテクチャの定義
#
GCC_TARGET = msp430-elf

#
#  プロセッサ依存部ディレクトリ名の定義
#
PRCDIR = $(SRCDIR)/arch/$(PRC)_$(TOOL)

#
#  コンパイルオプション
#
INCLUDES := $(INCLUDES) \
		    -I$(SRCDIR)/arch/$(PRC)_$(TOOL)/include_gcc \
			-I$(SRCDIR)/arch/$(PRC)_$(TOOL)/driverlib/MSP430F5xx_6xx
COPTS := $(COPTS) -mhwmult=f5series -mmcu=msp430f5529 -mlarge -mcode-region=none -mdata-region=lower \
			-Wno-int-to-pointer-cast \
			-Wno-maybe-uninitialized \
			-Wno-parentheses
LDFLAGS := $(LDFLAGS) -nostartfiles

#
#  カーネルに関する定義
#
KERNEL_DIR := $(KERNEL_DIR) $(PRCDIR) \
			  $(SRCDIR)/arch/$(PRC)_$(TOOL)/driverlib/MSP430F5xx_6xx
KERNEL_ASMOBJS := $(KERNEL_ASMOBJS) prc_support.o
KERNEL_COBJS := $(KERNEL_COBJS) prc_config.o

#
#  コンフィギュレータ関係の変数の定義
#
CFG_TABS := $(CFG_TABS) --cfg1-def-table $(PRCDIR)/prc_def.csv

#
#  オフセットファイル生成のための定義
#
OFFSET_TF = $(PRCDIR)/prc_offset.tf
