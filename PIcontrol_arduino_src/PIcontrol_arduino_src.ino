/*
 * ボード設定：Arduino DUE（Programming Port）
 */

// _/-_/-_/-_/- 定数定義 _/-_/-_/-_/-
const int getEncoderPin = 10; // エンコーダーにつながるピン(外部割り込み設定)

// _/-_/-_/-_/- 変数定義 _/-_/-_/-_/-
volatile int encoderPulseCnt; // エンコーダパルスカウンタ
volatile bool isCalculationRpmReq; // rpmの計算要求フラグ

int rpm;

void setup() {

  encoderPulseCnt = 0;
  isCalculationRpmReq = false;
  rpm = 0;

  Serial.begin(115200);

  // 割り込み設定
  attachInterrupt(digitalPinToInterrupt(getEncoderPin), encoderInterrupt, CHANGE);
  startTimer(TC1, 0, TC3_IRQn, 20);

  delay(1000);
}

void loop() {

  // 20msecに一度実行
  // 現在の回転数を計算
  // 結果をもとに回転速度要求を計算
  // 現在の回転数をPCに送信
  if( isCalculationRpmReq ){
    isCalculationRpmReq = false;
    rpm = encoderPulseCnt*15; // [cnt/20ms] -> [rpm]
    encoderPulseCnt = 0;
    
    // PI制御のプログラム

    // ここまで
    
    Serial.println( rpm );
  }
}

// タイマ割り込み設定
void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t mSec) {
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
  uint32_t rc = (VARIANT_MCK/2/1000)*mSec;
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}

// 20msecのタイマ割り込み
void TC3_Handler() {
  TC_GetStatus(TC1, 0);
  isCalculationRpmReq = true;
}

// エンコーダーの立ち上がり/下がりエッジをカウント
void encoderInterrupt(){
  ++encoderPulseCnt;
}
