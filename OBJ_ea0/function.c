/*
 * function.c
 *
 *  Created on: 2022/04/05
 *      Author: ryuta
 */

#include <kernel.h>
#include "kernel_cfg.h"
#include "driverlib.h"
#include "ea0.h"

#include "function.h"

#define TOGGLE_WIDTH	10
#define KEYS			5

typedef struct {
	uint16_t bit;
	uint8_t count;
	FLGPTN event_off;
	FLGPTN event_on;
} KEY_STATE;

static KEY_STATE m_key[] = {
		{ GPIO_PIN6,    0x00,   EVENT_ESC_OFF,  EVENT_ESC_ON    },	// F1[ESC]
		{ GPIO_PIN2,    0x00,   EVENT_SHF_OFF,  EVENT_SHF_ON    },	// F2[SHF]
		{ GPIO_PIN4,    0x00,   EVENT_UP__OFF,  EVENT_UP__ON    },	// F3[UP]
		{ GPIO_PIN0,    0x00,   EVENT_DWN_OFF,  EVENT_DWN_ON    },	// F4[DWN]
		{ GPIO_PIN5,    0x00,   EVENT_ENT_OFF,  EVENT_ENT_ON    },	// F5[ENT]
};


/* ファンクションキータスク */
__attribute__ ((section (".subtext"))) void function_task(intptr_t exinf)
{
	uint16_t sample[3];
	uint16_t flg;
    uint16_t mnu;
    uint16_t state;
    int i;
	uint8_t toggle = TOGGLE_WIDTH;
	uint16_t push;
	uint16_t release;
	int timer;

	sample[0] = 0;
	sample[1] = 0;
	sample[2] = 0;

	while (1) {
		flg = 0;
		mnu = 0;
		state = 0;
        for (i = 0; i < KEYS; i++) {
            if (GPIO_getInputPinValue(GPIO_PORT_P2, m_key[i].bit) == GPIO_INPUT_PIN_HIGH)
                state |= m_key[i].bit;
        }
		for (i = 0; i < KEYS; i++) {
			if (~(state) & m_key[i].bit) {
                if ((i != 2) && (i != 3)) {
                    flg |= (m_key[i].bit);
                } else {
                    if (m_key[i].count >= toggle) {
                        flg &= ~(m_key[i].bit);
                        m_key[i].count = 0;
                        toggle = TOGGLE_WIDTH;
                    } else {
                        flg |= m_key[i].bit;
                        m_key[i].count++;
                    }
                }
                mnu |= (1 << i);
			} else {
				m_key[i].count = 0;
			}
		}
		sample[2] = sample[1];
		sample[1] = sample[0];
		sample[0] = ~flg ^ 0xFFFF;
		// 変化が検出されたbitが1になる
		push = (sample[2] ^ 0xFFFF) & sample[1] & sample[0];
		release = sample[2] & (sample[1] ^ 0xFFFF) & (sample[0] ^ 0xFFFF);
		if ((mnu & 0x0001) && (mnu & 0x0010)) {
		    push = release = 0;
		    timer++;
		    if (timer >= 40) {
		        timer = 0;
                set_flg(FLG_INP, EVENT_MNU);
		    }
		} else {
            timer = 0;
		}
		if (push) {
			for (i = 0; i < KEYS; i++) {
				if (push & m_key[i].bit)
					set_flg(FLG_INP, m_key[i].event_on);
			}
		}
		if (release) {
			for (i = 0; i < KEYS; i++) {
				if (release & m_key[i].bit) {
					set_flg(FLG_INP, m_key[i].event_off);
					toggle = TOGGLE_WIDTH;
				}
			}
		}
		dly_tsk(10);	// 反応を調整できる
	}
}

/* ファンクションキーの初期化 */
__attribute__ ((section (".subtext"))) void function_init(void)
{
	act_tsk(TSK_FNC);
}
