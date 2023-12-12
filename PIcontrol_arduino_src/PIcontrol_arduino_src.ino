/*
 * ボード設定：Arduino DUE（Programming Port）
 * 
 * タイマ割り込み参考サイト
 * ↓20msの割り込みには使えない
 * http://jtakao.web.fc2.com/elec/due_timerinterrupt/index.html
 * ↓使える
 * https://qiita.com/GANTZ/items/5ae148baca9072f62016
 * 
 * 外部割り込み参考サイト
 * https://garretlab.web.fc2.com/arduino_reference/language/functions/external_interrupts/attachInterrupt.html
 * 
 * Arduino DUE のPWM出力法方
 * https://www.shujima.work/entry/2018/07/29/170810
 */
 
#include <DueTimer.h>

// _/-_/-_/-_/- 定数定義 _/-_/-_/-_/-
volatile const int getEncoderPin = 10; // エンコーダーにつながるピン(外部割り込み設定)
const int speedReqPin = 13; // モータ電圧制御ピン
volatile const int testPin = 5;
volatile int testSta;
volatile int speedReqSta;


// _/-_/-_/-_/- 変数定義 _/-_/-_/-_/-
volatile int encoderPulseCnt; // エンコーダパルスカウンタ
volatile bool isCalculationRpmReq; // rpmの計算要求フラグ

int rpm;

void setup() {

  encoderPulseCnt = 0;
  isCalculationRpmReq = false;
  rpm = 0;

  pinMode( speedReqPin, OUTPUT);
  analogWriteResolution(12);  // PWMの分解能を12bitにする(？)

  pinMode( testPin, OUTPUT);
  testSta = speedReqSta = HIGH;

  pinMode( getEncoderPin, INPUT);
  

  Serial.begin(115200);

  // 割り込み設定
  attachInterrupt(digitalPinToInterrupt(getEncoderPin), encoderInterrupt, CHANGE);
  // タイマ割り込みの設定
  Timer3.attachInterrupt(Timer3_handler);
  Timer3.start(20000); // 10msec (10,000usec)
//  startTimer(TC1, 0, TC3_IRQn,  1000);

  delay(1000);
}

void loop() {

  // 20msecに一度実行
  // 現在の回転数を計算
  // 結果をもとに回転速度要求を計算
  // 現在の回転数をPCに送信
//  if( isCalculationRpmReq ){
//    isCalculationRpmReq = false;
//    rpm = encoderPulseCnt*15; // [cnt/20ms] -> [rpm]
//    encoderPulseCnt = 0;
//    
//    // PI制御のプログラム
//
//    // ここまで
//    
//    Serial.println( rpm );
//  }
}

//// タイマ割り込み設定
//void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t mSec) {
//  pmc_enable_periph_clk((uint32_t)irq);
//  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
//  uint32_t rc = (VARIANT_MCK/2/1000)*mSec;
//  TC_SetRC(tc, channel, rc);
//  TC_Start(tc, channel);
//  tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
//  tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
//  NVIC_EnableIRQ(irq);
//}

//// 20msecのタイマ割り込み
//void TC3_Handler() {
//  TC_GetStatus(TC1, 0);
//  isCalculationRpmReq = true;
//
//  digitalWrite( testPin, testSta);
//  testSta = !testSta;
//}

// タイマハンドラ
void Timer3_handler(void){
  digitalWrite( testPin, testSta);
  testSta = !testSta;
//  if(flag == true){
//    digitalWrite(7,HIGH);
//    flag = false;
//  }else{
//    digitalWrite(7,LOW);
//    flag = true;
//  }
}

// エンコーダーの立ち上がり/下がりエッジをカウント
void encoderInterrupt(){
  ++encoderPulseCnt;

  digitalWrite( speedReqPin, speedReqSta);
  speedReqSta = !speedReqSta;
}
