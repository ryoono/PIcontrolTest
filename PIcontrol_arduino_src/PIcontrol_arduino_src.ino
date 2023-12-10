/*
 * ボード設定：Arduino DUE（Programming Port）
 */

// エンコーダーにつながるピン(外部割り込み設定)
const int getEncoderPin = 10;

void setup() {

  attachInterrupt(digitalPinToInterrupt(getEncoderPin), encoderInterrupt, CHANGE);
  startTimer(TC1, 0, TC3_IRQn, 20);
}

void loop() {
  // put your main code here, to run repeatedly:

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


void TC3_Handler() {
  TC_GetStatus(TC1, 0);
  // 割り込み発生時に実行する部分
}

void encoderInterrupt(){
  
}
