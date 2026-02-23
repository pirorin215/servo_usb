// Pro Micro + SG90 サーボモーター制御
// オルタネートスイッチで0°/180°を制御（動作中の切り替えに即座に反応）

#include <Servo.h>

// ピン設定
const int SERVO_PIN = 5;
const int SWITCH_PIN = 4;

// 定義値
const int TARGET_ANGLE = 180;
const int STEP_DELAY = 50;

// グローバル変数
Servo myServo;
int currentAngle = 0;

void setup() {
  // スイッチの設定（プルアップ抵抗方式）
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // サーボの設定
  myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);
}

void loop() {
  // スイッチ状態を読み取り（プルアップなのでLOW=ON, HIGH=OFF）
  int switchState = digitalRead(SWITCH_PIN);

  // スイッチOFFなら0°、ONなら180°を目標に設定
  int targetAngle = (switchState == LOW) ? TARGET_ANGLE : 0;

  // 現在の角度が目標と違うなら移動
  if (currentAngle != targetAngle) {
    // 目標角度に向かって1度ずつ移動
    if (currentAngle < targetAngle) {
      currentAngle++;
    } else {
      currentAngle--;
    }
    myServo.write(currentAngle);
    delay(STEP_DELAY);
  }
}
