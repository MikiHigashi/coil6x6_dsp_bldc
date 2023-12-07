// コンデンサー充電器制御 dsPIC
#define FCY 69784687UL
#include <libpic30.h>
#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <string.h>
#include "soft_i2c.h"
#include "lcd_i2c.h"

#define V_BASE 8 /* ゼロ電圧時の MAX186 取得値 */
#define V_MAGA 100400 /* 分圧倍率分子 */
#define V_MAGB 999 /* 分圧倍率分母 */
#define PWM_LOW 32 /* 突入電流最小化PWM値 */
#define PWM_MAX 6400 /* PWM最大値 */


typedef union tagHL16 {
    uint16_t HL;
    struct {
        uint8_t L;
        uint8_t H;
    };
    struct {
        unsigned voltage:12;
        unsigned rsv1:1;
        unsigned rsv2:1;
        unsigned rsv3:1;
        unsigned charged:1;
    };
} HL16;



typedef union tagPWM4 {
    uint16_t pwm[4];
    uint8_t buf[8];
  
} PWM4;

#define SPI_BYTES 8 /* SPI送受信するデーターのバイト数 */
#define MAX_CNT_ERR 5 /* 連続エラーがこれだけ続くと強制停止 */
#define MIN_V 50 /* OGBT破損チェックに使う電圧閾値 0.1V単位 */
#define CHARGE_COUNT 10 /* 短時間充電時間（10ミリ秒単位）*/
#define TRIGGER_COUNT 30 /* トリガーON送出時間（10ミリ秒単位）*/
#define BROKEN_COUNT 1000 /* 破損検出後の最低限無条件停止時間（10ミリ秒単位）*/

uint16_t temp = 0; // 温度センサー


PWM4 data; // 受け取った情報
uint16_t cnt_err = 0; // ERROR 連続回数
uint16_t data_ok = 0; // 正常に受信できた最後のデーター 0:強制停止
// MSB15 1:充電ON 0:充電OFF
 // bit14 1:トリガー押された 0:トリガー押されていない
 // bit13 予備
 // bit12 予備
 // bit11-0 充電ターゲット電圧
PWM4 send; // 送り返す情報 send.pwm[0] だけに書き込めば良い
 // MSB15 1:充電完了 0:充電未完
 // bit14 1:IGBT破損の可能性あり 0:正常
 // bit13 1:装填されている 0:装填されていない
 // bit12 予備
 // bit11-0 充電現在電圧

uint8_t mode = 0; // 0:
 // 0: 処理待ち

#define N_V_PAST 16
signed short v_past[N_V_PAST]; // 電圧値
uint8_t v_past_p = 0; // 格納位置

signed short v_target; // 目標電圧
signed short v_target_over; // 超過電圧
signed short v_raw; // 電圧ナマ値
#define OVER_DOWN 320 /* v_target_over 超過時のDUTY急減速値 */
#define MAX_CHANGE1 20 /* pwm2最大増量 */


char buf[32];

uint16_t countdown = 0; // カウントダウンタイマー
uint16_t countbroken = 0; // 破損時カウントダウンタイマー
 // TMR2 割り込みごとにカウントダウンする。１カウント10ミリ秒


#define MAX186_V   0b11111111

//////////////////////////////////
// MAX186 からAD変換値を取得
//////////////////////////////////
uint16_t max186(uint8_t ch) {
    SPI_CS_SetLow();
    SPI2_Exchange8bit(ch);
    uint16_t hh = (uint16_t)SPI2_Exchange8bit(0);
    uint16_t ll = (uint16_t)SPI2_Exchange8bit(0);
    SPI_CS_SetHigh();
    return (((hh << 5) | (ll >> 3)) & 0xfff); 
}


// SPI受信
void int_strb(void) {
    if (SPI3_STRB_GetValue()) return;
    send.pwm[3] = send.pwm[2] = send.pwm[1] = send.pwm[0];

    uint8_t idx, b, d; //, *dp = data;
    uint8_t m, d2; //, *dp2 = send;
    for (idx=0; idx<SPI_BYTES; idx++) {
        d = 0;
        d2 = send.buf[idx];
        m = 0x80;
        for (b=0; b<8; b++) {
            while (SPI3_CLOCK_GetValue() == 0) {} // CLOCK立ち上がりをソフトループで待つ
            if (d2 & m) {
                SPI3_OUT_SetHigh();
            }
            else {
                SPI3_OUT_SetLow();
            }
            m >>= 1;
            d <<= 1;
            while (SPI3_CLOCK_GetValue() == 1) {} // CLOCK立ち下がりをソフトループで待つ
            d |= SPI3_IN_GetValue();
        }
        data.buf[idx] = d;
    }

    // データー化けチェック
    if (data.pwm[0] == data.pwm[3]) {
        if (data.pwm[0] == data.pwm[2]) {
            if (data.pwm[0] == data.pwm[1]) {
                data_ok = data.pwm[0];
                cnt_err = 0;
            }
        }
    }
    cnt_err ++;
    if (cnt_err >= MAX_CNT_ERR) {
        cnt_err = MAX_CNT_ERR;
        data_ok = 0;
    }
}


//////////////////////////////////
// MAX186 取得値補正
//////////////////////////////////
// MAX186値を0.1V単位に変換
uint16_t max186_to_volt(uint16_t vin) {
    if (vin < V_BASE) return 0;
    unsigned long v = vin - V_BASE;
    v *= V_MAGA;
    v /= V_MAGB;
    v += 50;
    v /= 100;
    return (uint16_t)(v & 0xfff); // 0.1V単位
}
// 0.1V単位のターゲット値をMAX186値に変換
short volt2max186(uint16_t vtg) {
    unsigned long v = (unsigned long)vtg * (unsigned long)100 * V_MAGB;
    v /= V_MAGA;
    return (short)v + V_BASE;
}


void int_timer(void) {
    if (countdown) {
        countdown --;
    }
    if (countbroken) {
        countbroken --;
    }
}


uint8_t mode_charger = 0; // コンデンサー充電モード
 // 0: コンデンサー充電せず待機
 // 1: 射撃後
 // 2: コンデンサー短時間充電
 // 3: コンデンサー充電状態再確認
 // 4: スイッチング素子破損
 // 5: コンデンサー通常充電中
 // 6: コンデンサー充電完了し電圧保持状態


int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    CN_SetInterruptHandler(int_strb);
    TMR2_SetInterruptHandler(int_timer);
    DMA_ChannelEnable(DMA_CHANNEL_0);
    DMA_PeripheralAddressSet(DMA_CHANNEL_0, (volatile unsigned int) &ADC1BUF0);
    DMA_StartAddressASet(DMA_CHANNEL_0, (uint16_t)(&temp));        
    PDC1 = 0;                
    PDC2 = 0;                
    TRIGGER_SetLow();

    __delay_ms(100);    
    WATCHDOG_TimerClear();
    LCD_i2c_init(8);
    
    short pwm = 0; // 全体電源 PWM 設定値 0 to 6400
    short pwm2 = 0; // チョッパー PWM 設定値 0 to 6400
    uint8_t charge = 0; // 充電スイッチ 1:ON 0:OFF
    uint8_t trig = 0; // トリガー入力 1:ON 0:OFF
    uint8_t charge0 = 0; // 直前のcharge
    uint8_t charged = 0; // 充電完了（電圧保持動作中）なら1
    uint8_t broken = 0; // IGBT破損なら1
    uint8_t set_trigeer = 0; // トリガーONにするなら1

    // 突入電流緩和１秒
    uint16_t cnt;
    for (cnt=0; cnt<300; cnt++) {
        PDC1 = PWM_LOW;
        __delay_ms(10);    
        WATCHDOG_TimerClear();
    }
    v_raw = max186_to_volt(max186(MAX186_V));
    if (v_raw < MIN_V) { 
        pwm = 0;
        mode_charger = 4; // スイッチング素子破損に移動
        countbroken = BROKEN_COUNT;
    }
    else {
        pwm = PWM_MAX;
    }

    while (1) {
        WATCHDOG_TimerClear();

        v_raw = max186_to_volt(max186(MAX186_V));
        uint16_t data_back = 1234; //v_raw; // SPI送り返し値
        uint16_t t = temp;
        if (t < 2900) t = 2900; // 最小電圧約7.2V
        if (t > 3380) t = 3380; // 最小電圧約8.4V
        unsigned long u = 16722712;
        u /= t;
        uint16_t max_pwm2 = (uint16_t)u; // PWM2 最大値 

        if (BULLET_GetValue() == 0) { // パチンコ玉が装填されている
            data_back |= 0x2000;  
        }
        if (broken) { // IGBT破損
            data_back |= 0x4000;  
        }
        if (charged) { // 充電完了
            data_back |= 0x8000;  
        }
        send.pwm[0] = data_back;

        v_target = volt2max186(3000);
        v_target_over = v_target + 32;

        // 電圧履歴
        signed short v_past1 = v_past[v_past_p]; // 過去値取り出し
        v_past[v_past_p++] = v_raw; // 最新値格納
        // 格納位置更新
        if (v_past_p >= N_V_PAST) {
            v_past_p = 0;
        }
        
        // 未来値予想
        signed short v_future = (v_raw + v_raw - v_past1);;
        if (v_future < 0) {
            v_future = 0;
        }

                    if (v_target > v_future) { // 電圧不足
                        pwm2 += MAX_CHANGE1;
                        if (pwm2 > max_pwm2) {
                            pwm2 = max_pwm2;
                        }
                    }
                    if (v_target < v_future) { // 電圧超過
                        if (v_target_over < v_future) {
                            if (pwm2 >= OVER_DOWN) {
                                pwm2 -= OVER_DOWN;
                            }
                            else {
                                pwm2 = 0;
                            }
                        }
                        else if (pwm2 > 0) {
                            pwm2 -= MAX_CHANGE1;
                        }
                    }
        PDC1 = (uint16_t)6400;
        PDC2 = (uint16_t)pwm2;

    }

    
    while (1) {
        WATCHDOG_TimerClear();

        v_raw = max186_to_volt(max186(MAX186_V));
        uint16_t data_back = v_raw; // SPI送り返し値
        // 10158 * temp >>12 でバッテリーのミリボルト値になる
        uint16_t t = temp;
        if (t < 2900) t = 2900; // 最小電圧約7.2V
        if (t > 3380) t = 3380; // 最小電圧約8.4V
        unsigned long u = 16722712;
        u /= t;
        uint16_t max_pwm2 = (uint16_t)u; // PWM2 最大値 

        if (BULLET_GetValue() == 0) { // パチンコ玉が装填されている
            data_back |= 0x2000;  
        }
        if (broken) { // IGBT破損
            data_back |= 0x4000;  
        }
        if (charged) { // 充電完了
            data_back |= 0x8000;  
        }
        send.pwm[0] = data_back;

        v_target = volt2max186(data_ok & 4095);
        v_target_over = v_target + 32;
        charge = (data_ok & 0x8000) ? 1 : 0;
        trig = (data_ok & 0x4000) ? 1 : 0;

        // 電圧履歴
        signed short v_past1 = v_past[v_past_p]; // 過去値取り出し
        v_past[v_past_p++] = v_raw; // 最新値格納
        // 格納位置更新
        if (v_past_p >= N_V_PAST) {
            v_past_p = 0;
        }
        
        // 未来値予想
        signed short v_future = (v_raw + v_raw - v_past1);;
        if (v_future < 0) {
            v_future = 0;
        }
        
        set_trigeer = 0;

        if (countbroken) {
            // 素子破損扱い中
            pwm = 0;
            pwm2 = 0;
            broken = 1;
            charge = 1; // 充電ON状態とし続けるが充電は停止
            charged = 0;
        }
        else if (charge) { // 充電ON
            pwm = PWM_MAX;
            if (charge0 == 0) { // OFF→ONになったばかり
                pwm2 = 0;
                if (v_raw > MIN_V) { // 十分な残存電圧あり
                    mode_charger = 5; // コンデンサー通常充電中に移動
                }
                else { // 十分な残存電圧なし
                    // コンデンサー短時間充電に移行
                    countdown = CHARGE_COUNT;
                    mode_charger = 2;
                }
            }
            else {
                if (mode_charger == 2) { // コンデンサー短時間充電
                    pwm2 = PWM_MAX;
                    if (countdown == 0) { // 動作待ち終了している
                        // コンデンサー充電状態再確認に移行
                        mode_charger = 3;
                    }
                }
                else if (mode_charger == 3) { // コンデンサー充電状態再確認
                    pwm2 = 0;
                    if (v_raw > MIN_V) { // 十分な残存電圧あり
                        mode_charger = 5; // コンデンサー通常充電中に移動
                    }
                    else { // 十分な残存電圧なし
                        mode_charger = 4; // スイッチング素子破損に移動
                        countbroken = BROKEN_COUNT;
                    }
                }
                else if (mode_charger == 4) { // スイッチング素子破損
                    pwm2 = 0;
                    if (countbroken) {
                        broken = 1;
                    }
                }
                else if (mode_charger == 5) { // コンデンサー通常充電中
                    if (v_target > v_future) { // 電圧不足
                        pwm2 += MAX_CHANGE1;
                        if (pwm2 > max_pwm2) {
                            pwm2 = max_pwm2;
                        }
                    }
                    if (v_target < v_future) { // 電圧超過
                        if (v_target_over < v_future) {
                            if (pwm2 >= OVER_DOWN) {
                                pwm2 -= OVER_DOWN;
                            }
                            else {
                                pwm2 = 0;
                            }
                        }
                        else if (pwm2 > 0) {
                            pwm2 -= MAX_CHANGE1;
                        }
                    }
                    if (v_target <= v_raw) {
                        mode_charger = 6; // コンデンサー充電完了し電圧保持状態に移動
                    }
                }
                else if (mode_charger == 6) { // コンデンサー充電完了し電圧保持状態
                    charged = 1;
                    if (v_target > v_future) { // 電圧不足
                        pwm2 ++;
                        if (pwm2 > max_pwm2) {
                            pwm2 = max_pwm2;
                        }
                    }
                    if (v_target < v_future) { // 電圧超過
                        if (v_target_over < v_future) {
                            if (pwm2 >= OVER_DOWN) {
                                pwm2 -= OVER_DOWN;
                            }
                            else {
                                pwm2 = 0;
                            }
                        }
                        else if (pwm2 > 0) {
                            pwm2 --;
                        }
                    }
                    if (trig) {
                        countdown = TRIGGER_COUNT;
                        mode_charger = 1; // 射撃後
                    }
                }
                else { // 射撃後
                    if (countdown) {
                        set_trigeer = 1;
                    }
                }
            }
        }
        else { // 充電OFF
            pwm = 0;
            pwm2 = 0;
            broken = 0;
            charged = 0;
        }        
        
        if (set_trigeer) {
            TRIGGER_SetHigh();
        }
        else {
            TRIGGER_SetLow();
        }
        PDC1 = (uint16_t)pwm;
        PDC2 = (uint16_t)pwm2;
        charge0 = charge;
        
        //LCD_i2C_cmd(0x80);
        //sprintf(buf, "%4d%5d%5d", v_target, power, trig);
        //LCD_i2C_data(buf);
    }
    return 1; 
}
