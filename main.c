// BLDC
#define FCY 69784687UL
#include <libpic30.h>
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <string.h>
#include "soft_i2c.h"
#include "lcd_i2c.h"
#include "pwm_table.inc"
#include "motor_table.inc"

#define VAL05 32766

#define ZEROD 2075 /* 角度センサーのゼロ点規正 1080 2075 */
// TMR1 秒間１万回割り込み
// TMR2 サーボパルス幅の測定に使う
// TMR3 実行時間測定用


typedef union tagHL32 {
    signed long SL;
    unsigned long HL;
    struct {
        uint16_t L;
        uint16_t H;
    };
} HL32;


typedef union tagSU16 {
    signed short S;
    uint16_t U;
} SU16;


#define MIN_VOL 1500
#define MAX_VOL 8191


signed long target_spd = 0; // ターゲット速度

uint8_t cnt_pwm = 0; // サーボパルス幅
uint8_t pwm_in = 0;


void int_pwm(void) {
    if (PULSE_GetValue()) {
        TMR2 = 0;
        pwm_in = 1;
    }
    else {
        if (pwm_in) {
            uint16_t cnt_pwm1 = TMR2;
            if (cnt_pwm1 < 500) return;
            if (cnt_pwm1 > 2800) return;
            if (cnt_pwm1 < 610) {
                cnt_pwm = 0;
            }
            else {
                cnt_pwm1 -= 610;
                cnt_pwm1 >>= 3;
                if (cnt_pwm1 > 255) {
                    cnt_pwm = 255;
                }
                else {
                    cnt_pwm = (uint8_t)(cnt_pwm1);
                }
            }
            pwm_in = 0;
        }
    }
}


char buf[32];


//////////////////////////////////
// 角度センサー取得
//////////////////////////////////
uint16_t as5048a(void) {
    SPI_CS_SetLow();
    uint16_t v = (uint16_t)SPI2_Exchange16bit(0xffff);
    SPI_CS_SetHigh();
    return (v & 0x3fff); 
}


//////////////////////////////////
// 正弦波出力値取得
// arc0: 角 0 to 16383
// adv: 進角 4096=90度
// vol: 振幅 0 to 8191
//////////////////////////////////
uint16_t get_pwm1(uint16_t arc0, signed long adv, uint16_t vol) {
    HL32 n1;
    n1.SL = (signed long)(arc0 * 5);
    n1.SL += adv;
    n1.L &= 0x3fff;
    n1.H = 0;
    uint16_t arc = n1.L;
    if (arc <= 4096) {
        n1.L = VAL05 + table_pwm[arc];
    }
    else if (arc <= 8192) {
        n1.L = VAL05 + table_pwm[8192 - arc];
    }
    else if (arc <= 12288) {
        n1.L = VAL05 - table_pwm[arc - 8192];
    }
    else {
        n1.L = VAL05 - table_pwm[16384 - arc];
    }
    n1.HL *= vol;
    return n1.H;
}


//////////////////////////////////
// 超低速クローズドループ専用
// 正弦波出力値取得
// arc0: 角 0 to 16383
// vol: 振幅 0 to 8191
//////////////////////////////////
uint16_t get_pwm2(uint16_t arc0, uint16_t vol) {
    HL32 n1;
    n1.SL = (signed long)arc0;
    n1.L &= 0x3fff;
    n1.H = 0;
    uint16_t arc = n1.L;
    if (arc <= 4096) {
        n1.L = VAL05 + table_pwm[arc];
    }
    else if (arc <= 8192) {
        n1.L = VAL05 + table_pwm[8192 - arc];
    }
    else if (arc <= 12288) {
        n1.L = VAL05 - table_pwm[arc - 8192];
    }
    else {
        n1.L = VAL05 - table_pwm[16384 - arc];
    }
    n1.HL *= vol;
    return n1.H;
}


int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    CN_SetInterruptHandler(int_pwm);
    TMR1_Start();
    TMR2_Start();
    PDC1 = 0;                
    PDC2 = 0;                
    PDC3 = 0;                

    __delay_ms(100);    
    WATCHDOG_TimerClear();
    LCD_i2c_init(8);

    HL32 su;
    uint16_t n = 0;
    uint16_t pwm1 = 0;
    uint16_t pwm2 = 0;
    uint16_t pwm3 = 0;
    signed long target_slow;
    signed short target_pwm;
    signed short target_pwm0 = 0;
    while (1) {
        WATCHDOG_TimerClear();
        target_pwm = table_pwmh[cnt_pwm];
        if (target_pwm) {
            __delay_us(100);    
            signed long adv;
            n = (as5048a() - ZEROD) & 0x3fff;
            if (target_pwm > 0) {
                adv = 4096 + (target_pwm >> 4);
            }
            else if (target_pwm < 0) {
                adv = (-4096) + (target_pwm >> 4);;
                target_pwm = (-target_pwm);
            }
            n &= 0x3fff;
            pwm1 = get_pwm1(n, adv, (uint16_t)target_pwm);
            pwm2 = get_pwm1(n + 5461, adv, (uint16_t)target_pwm);
            pwm3 = get_pwm1(n + 10923, adv, (uint16_t)target_pwm);
            PDC1 = pwm1;
            PDC2 = pwm2;
            PDC3 = pwm3;
        }
        else { // open loop
            target_slow = table_slowh[cnt_pwm];
            if (target_pwm0) {
                su.H = (as5048a() - ZEROD) & 0x3fff;
                su.H *= 5;
                su.L = 0;
            }
            su.SL += target_slow;
            pwm1 = get_pwm2(su.H, MIN_VOL);
            pwm2 = get_pwm2(su.H + 5461, MIN_VOL);
            pwm3 = get_pwm2(su.H + 10923, MIN_VOL);
        }
        PDC1 = pwm1;
        PDC2 = pwm2;
        PDC3 = pwm3;
        target_pwm0 = target_pwm;
    }    

//        LCD_i2C_cmd(0x80);
//        sprintf(buf, "%6d", (signed short)fut_spd);
//        LCD_i2C_data(buf);
//        __delay_us(100);    
}
