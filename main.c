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


#define MIN_VOL 1024
#define MAX_VOL 8191


#define NUM_BUF 64 /* 角度センサー値の過去バッファー数 */
uint16_t BUFD[NUM_BUF]; // 角度センサーのナマ値履歴
signed short BUFS[NUM_BUF]; // 速度履歴
signed long sum_deg = 0; // 過去NUM_BUF回のBUFS合計
signed long  SUMD[NUM_BUF]; // sum_deg の履歴
uint8_t pBufd = 0; // 次に値を保存する添字

signed long target_spd = 0; // ターゲット速度
signed long now_advd = 0; // 現在の進角 360度=16384 90度=4096
signed long now_vol = MIN_VOL; // 現在のボリューム


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
// vol: 振幅 0 to 8191
//////////////////////////////////
uint16_t get_pwm1(uint16_t arc0, uint16_t vol) {
    HL32 n1;
    n1.SL = (signed long)(arc0 * 5);
    n1.SL += now_advd;
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


// 加減量 add に応じて now_advd と now_vol を更新する
void add_d_and_vol0(signed long add) {
    if (add > 0) {
        add += 4096;

        if (now_advd <= (-4096)) {
            now_vol -= add;
            if (now_vol >= MIN_VOL) return;
            add = MIN_VOL - now_vol;
            now_vol = MIN_VOL;
        }
        if (now_advd < 4096) {
            now_advd += add;
            if (now_advd < 4096) return;
            add = now_advd - 4096;
            now_advd = 4096;
        }
        now_vol += add;
        if (now_vol > MAX_VOL) {
            now_vol = MAX_VOL;
        }
    }
    if (add < 0) {
        add -= 4096;
        
        if (now_advd >= 4096) {
            now_vol += add;
            if (now_vol >= MIN_VOL) return;
            add = now_vol - MIN_VOL;
            now_vol = MIN_VOL;
        }
        if (now_advd > (-4096)) {
            now_advd += add;
            if (now_advd > (-4096)) return;
            add = now_advd + 4096;
            now_advd = (-4096);
        }
        now_vol -= add;
        if (now_vol > MAX_VOL) {
            now_vol = MAX_VOL;
        }
    }
}


void add_d_and_vol(signed long add) {
    if (add > 0) {
        now_advd = 4096;
        now_vol = add;
    }
    else {
        now_advd = (-4096);
        now_vol = (-add);
    }
    if (now_vol > MAX_VOL) {
        now_vol = MAX_VOL;
    }
}


signed short get_speed(uint16_t now, uint16_t pre) {
    uint16_t s;
    signed short r;
    if (now >= pre) {
        s = now - pre;
        if (s < 8192) return (signed short)s;
        r = (signed short)(16384 - now + pre);
        return (-r);
    }
    s = pre - now;
    if (s < 8192) {
        r = (signed short)s;
        return (-r);
    }
    return (signed short)(16384 - pre + now);
}


int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    uint8_t i;
    uint16_t n = as5048a();
    for (i=0; i<NUM_BUF; i++) {
        BUFD[i] = n;
        BUFS[i] = 0;
        SUMD[i] = 0;
    }
    CN_SetInterruptHandler(int_pwm);
    TMR1_Start();
    TMR2_Start();
    PDC1 = 0;                
    PDC2 = 0;                
    PDC3 = 0;                

    __delay_ms(100);    
    WATCHDOG_TimerClear();
    LCD_i2c_init(8);

/*
    while (1) {
        WATCHDOG_TimerClear();
        now_advd = 0;
        uint16_t pwm1 = get_pwm1(0, 1024);
        uint16_t pwm2 = get_pwm1(0 + 5461, 1024);
        uint16_t pwm3 = get_pwm1(0 + 10923, 1024);
        PDC1 = pwm1;
        PDC2 = pwm2;
        PDC3 = pwm3;
        uint16_t n = as5048a();
        LCD_i2C_cmd(0x80);
        sprintf(buf, "%6u", n);
        LCD_i2C_data(buf);
        __delay_ms(100);    
        __delay_ms(100);    
    }    
*/
    
    while (1) {
        WATCHDOG_TimerClear();
        target_spd = table_motor[cnt_pwm];
        uint8_t pBufd0;
        if (pBufd) {
            pBufd0 = pBufd - 1;
        }
        else {
            pBufd0 = NUM_BUF - 1;
        }
        uint16_t n0 = BUFD[pBufd0];

        signed short s = get_speed(n, n0); 

        
        now_vol = 32 * cnt_pwm;
        now_advd = 4096;
        uint16_t n = as5048a();
        BUFD[pBufd] = n;
        pBufd ++;
        if (pBufd >= NUM_BUF) pBufd = 0;

        uint16_t n1 = (n - ZEROD) & 0x3fff;
        uint16_t pwm1 = get_pwm1(n1, now_vol);
        uint16_t pwm2 = get_pwm1(n1 + 5461, now_vol);
        uint16_t pwm3 = get_pwm1(n1 + 10923, now_vol);
        PDC1 = pwm1;
        PDC2 = pwm2;
        PDC3 = pwm3;
        __delay_us(100);    
    }    
    
    
    
  
    
    
    
    
    
    while (1) {
        WATCHDOG_TimerClear();
        target_spd = table_motor[cnt_pwm];
        now_vol = 32 * cnt_pwm;
now_advd = 4096;
        uint16_t n = as5048a();
        uint16_t n1 = (n - ZEROD) & 0x3fff;
        uint16_t pwm1 = get_pwm1(n1, now_vol);
        uint16_t pwm2 = get_pwm1(n1 + 5461, now_vol);
        uint16_t pwm3 = get_pwm1(n1 + 10923, now_vol);
        PDC1 = pwm1;
        PDC2 = pwm2;
        PDC3 = pwm3;
        __delay_us(100);    
    }    
    
    
    while (1) {
        WATCHDOG_TimerClear();
        target_spd = table_motor[cnt_pwm];
        uint8_t pBufd0;
        if (pBufd) {
            pBufd0 = pBufd - 1;
        }
        else {
            pBufd0 = NUM_BUF - 1;
        }
        uint16_t n0 = BUFD[pBufd0];

//        while (TMR1 < 12000) ; // 6977
        uint16_t n = as5048a();
//        TMR1 = 0;

        //n - n0 が最新速度
        signed short s = get_speed(n, n0); 
        if ((s < (-600)) || (s > 600)) { // 異常速度
            s = BUFS[pBufd0]; // 直前の速度を使用
            SU16 su;
            su.S = s + (signed short)n0;
            n = su.U;
            n &= 0x3fff;
        }
        BUFD[pBufd] = n;
        signed long las = (signed long)BUFS[pBufd];
        BUFS[pBufd] = s;

        sum_deg += (signed long)s;
        sum_deg -= las; // 平均速度・合計速度でもある
        
//        uint16_t n1 = (n + n - n0 - ZEROD) & 0x3fff;
        uint16_t n1 = (n - ZEROD) & 0x3fff;

        signed long las_spd = SUMD[pBufd]; // 以前の速度
        SUMD[pBufd ++] = sum_deg; // 最新の速度
        if (pBufd >= NUM_BUF) pBufd = 0;
        signed long fut_spd = sum_deg + sum_deg - las_spd; // 未来の速度

        //add_d_and_vol(target_spd  - fut_spd);
//        add_d_and_vol(target_spd / 2);
now_vol = 32 * cnt_pwm;
now_advd = 4096;

        uint16_t pwm1 = get_pwm1(n1, now_vol);
        uint16_t pwm2 = get_pwm1(n1 + 5461, now_vol);
        uint16_t pwm3 = get_pwm1(n1 + 10923, now_vol);
        PDC1 = pwm1;
        PDC2 = pwm2;
        PDC3 = pwm3;

//        HL32 v;
//        v.SL = fut_spd;

//        LCD_i2C_cmd(0x80);
//        sprintf(buf, "%6d", (signed short)fut_spd);
//        LCD_i2C_data(buf);
        __delay_us(100);    
    }
}
