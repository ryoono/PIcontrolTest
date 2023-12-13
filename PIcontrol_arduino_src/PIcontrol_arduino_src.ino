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

#define Kp      5
#define Ki      10
#define target  4000
#define dt      0.02


// _/-_/-_/-_/- 定数定義 _/-_/-_/-_/-
const int getEncoderPin = 10; // エンコーダーにつながるピン(外部割り込み設定)
const int speedReqPin = 13; // モータ電圧制御ピン


// _/-_/-_/-_/- 変数定義 _/-_/-_/-_/-
volatile int encoderPulseCnt; // エンコーダパルスカウンタ
volatile bool isCalculationRpmReq; // rpmの計算要求フラグ

int rpm;
int U, duty;
float P, I;

void setup() {

  encoderPulseCnt = 0;
  isCalculationRpmReq = false;
  rpm = 0;
  P = I = 0.0;
  U = duty = 0;

  pinMode( speedReqPin, OUTPUT);
  analogWriteResolution(12);  // PWMの分解能を12bitにする(？)
  analogWrite( speedReqPin, 4095);

  pinMode( getEncoderPin, INPUT);
  

  Serial.begin(115200);

  // 割り込み設定 立ち上がり/立ち下がりで割り込み
  attachInterrupt(digitalPinToInterrupt(getEncoderPin), encoderInterrupt, CHANGE);
  // タイマ割り込みの設定
  Timer3.attachInterrupt(Timer3_handler);
  Timer3.start(20000); // 20msec (20,000usec)

  delay(2000);
}

void loop() {

  // 20msecに一度実行
  // 現在の回転数を計算
  // 結果をもとに回転速度要求を計算
  // 現在の回転数をPCに送信
  if( isCalculationRpmReq ){
    isCalculationRpmReq = false;
    rpm = encoderPulseCnt * 15; // [cnt/20ms] -> [rpm]
    encoderPulseCnt = 0;
    
    // PI制御のプログラム
    P  = target - rpm;
    I += P * dt;
    U = (int)(Kp * P) + (int)(Ki * I);
    
    duty += (U - duty);
    if( duty > 4095 ) duty = 4095;
    if( duty < 0 ) duty = 0;

    analogWrite( speedReqPin, 4095-duty);
    
    if( rpm ){
      Serial.print( rpm );
      Serial.print( ',' );
      Serial.println( 4095-duty );
    }
  }
}

// タイマハンドラ
void Timer3_handler(void){
  isCalculationRpmReq = true;
}

// エンコーダーの立ち上がり/下がりエッジをカウント
void encoderInterrupt(){
  ++encoderPulseCnt;
}
