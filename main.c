// BLDC
#define FCY 69784687UL
#include <libpic30.h>
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <string.h>
#include "soft_i2c.h"
#include "lcd_i2c.h"
#include "pwm_table.inc"

#define VAL05 32766


typedef union tagHL32 {
    unsigned long HL;
    struct {
        uint16_t L;
        uint16_t H;
    };
} HL32;


// TMR2 サーボパルス幅の測定に使う


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
    SPI2_Exchange8bit(0xff);
    uint16_t hh = (uint16_t)SPI2_Exchange8bit(0xff);
    SPI2_Exchange8bit(0xff);
    uint16_t ll = (uint16_t)SPI2_Exchange8bit(0xff);
    SPI_CS_SetHigh();
    return (((hh << 8) | ll) & 0x3fff); 
}


//////////////////////////////////
// 正弦波出力値取得
// arc: 角 0 to 16383
// vol: 振幅 0 to 8191
//////////////////////////////////
uint16_t get_pwm1(uint16_t arc, uint16_t vol) {
    HL32 n1;
    arc &= 0x3fff;
    n1.H = 0;
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
// 正弦波出力値取得
// arc: 角 0 to 16383
// vol: 振幅 0 to 8191
//////////////////////////////////
uint16_t get_pwm2(uint16_t arc, uint16_t rev) {
    arc &= 0x3fff;
    uint16_t n = table_pwm[arc >> 2];
    return (n >> 6);
}



int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    CN_SetInterruptHandler(int_pwm);
    //TMR2_SetInterruptHandler(int_timer);
    //DMA_ChannelEnable(DMA_CHANNEL_0);
    //DMA_PeripheralAddressSet(DMA_CHANNEL_0, (volatile unsigned int) &ADC1BUF0);
    //DMA_StartAddressASet(DMA_CHANNEL_0, (uint16_t)(&temp));        
    TMR2_Start();


    PDC1 = 0;                
    PDC2 = 0;                
    PDC3 = 0;                

    __delay_ms(100);    
    WATCHDOG_TimerClear();
    LCD_i2c_init(8);

    uint16_t arc = 0;
    
    
    while (1) {
        WATCHDOG_TimerClear();
        
        arc ++;
        arc &= 0x3fff;
        uint16_t vol = (uint16_t)cnt_pwm;
        vol <<= 5;
        
vol = 1500;
        uint16_t pwm1 = get_pwm1(arc, vol);
        uint16_t pwm2 = get_pwm1(arc + 5461, vol);
        uint16_t pwm3 = get_pwm1(arc + 10923, vol);
        PDC1 = pwm1;
        PDC2 = pwm2;
        PDC3 = pwm3;

//        uint16_t n = as5048a();

//        LCD_i2C_cmd(0x80);
//        sprintf(buf, "%5u", t1);
//        LCD_i2C_data(buf);
        __delay_us(100);    

    }
}
