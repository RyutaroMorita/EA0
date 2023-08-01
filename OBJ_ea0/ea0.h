/*
 * ea0.h
 *
 *  Created on: 2023/07/31
 *      Author: ryuta
 */

#ifndef EA0_H
#define EA0_H


/*
 *  レジスタテーブル
 */
typedef struct {
    bool_t registered;
    uint16_t address;
    uint16_t data[4];
} REG_RECORD;

/*
 *  表示モード
 */
typedef enum {
    Signed,
    Unsigned,
    Hex,
    Binary,
    Long,
    Long_Inverse,
    Float,
    Float_Inverse,
    Double,
    Double_Inverse,
} DSP_MODE;

/*
 *  操作モード
 */
typedef enum {
    Address,
    Value,
    View,
} OPT_MODE;

/*
 *  イベントフラグ
 */
#define EVENT_ESC_OFF       0x0001UL
#define EVENT_ESC_ON        0x0002UL
#define EVENT_SHF_OFF       0x0004UL
#define EVENT_SHF_ON        0x0008UL
#define EVENT_UP__OFF       0x0010UL
#define EVENT_UP__ON        0x0020UL
#define EVENT_DWN_OFF       0x0040UL
#define EVENT_DWN_ON        0x0080UL
#define EVENT_ENT_OFF       0x0100UL
#define EVENT_ENT_ON        0x0200UL
#define EVENT_MNU           0x8000UL


#endif /* EA0_H */
