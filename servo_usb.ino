// Pro Micro + SG90 サーボモーター制御
// オルタネートスイッチで2つの角度を制御（経由地を通る移動）

#include <Servo.h>
#include <stdarg.h>

// ピン設定
const int SERVO_PIN = 5;
const int SWITCH_PIN = 4;

// 定義値
const int ANGLE_ON = 130;                      // スイッチ開放時の角度（ON状態）
const int ANGLE_OFF = 167;                     // スイッチGND接触時の角度（OFF状態）
const int WAYPOINT_ANGLE_ON_TO_OFF = 180;      // ON→OFF時の経由地角度
const int WAYPOINT_ANGLE_OFF_TO_ON = 120;      // OFF→ON時の経由地角度
const int WAYPOINT_DELAY = 800;                // 経由地での遅延(ms)
const int STEP_DELAY = 0;                      // 1ステップの遅延(ms)
const unsigned long DEBOUNCE_DELAY = 50;       // デバウンス遅延(ms)

// 移動状態
enum MoveState {
  IDLE = 0,              // 移動なし
  MOVING_TO_WAYPOINT = 1,  // 経由地へ移動中
  WAITING_AT_WAYPOINT = 2, // 経由地で遅延中
  MOVING_TO_TARGET = 3     // 目標地へ移動中
};

// グローバル変数
Servo myServo;
int currentAngle = 0;             // 初期位置（setupで設定）
MoveState moveState = IDLE;
int lastTargetAngle = -1;
int currentWaypoint = 0;          // 現在の経由地
unsigned long delayStartTime = 0;

// デバウンス用グローバル変数
unsigned long lastDebounceTime = 0;
int lastDebouncedState = -1;      // デバウンス後のスイッチ状態（グローバル化）
int lastRawSwitchState = -1;      // 前回の生スイッチ状態

// ログ出力関数（millis()を自動的に先頭に付加）
void log(const char* format, ...) {
  char buf[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  Serial.print(millis());
  Serial.print(" ");
  Serial.println(buf);
}

void setup() {
  Serial.begin(9600);
  //while (!Serial);  // シリアルモニタを待つ

  // スイッチの設定（プルアップ抵抗方式）
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // 初期スイッチ状態を読み取り
  int switchState = digitalRead(SWITCH_PIN);
  lastTargetAngle = (switchState == HIGH) ? ANGLE_ON : ANGLE_OFF;
  currentAngle = lastTargetAngle;  // 初期位置をスイッチ状態に合わせる

  // デバウンス状態の初期化（setupで行う！）
  lastDebouncedState = switchState;
  lastRawSwitchState = switchState;

  // サーボの設定
  myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);

  Serial.println("=== Startup ===");
  log("Initial currentAngle: %d", currentAngle);
  log("Initial lastTargetAngle: %d", lastTargetAngle);
  log("Initial switchState: %s", switchState == HIGH ? "HIGH" : "LOW");
}

void loop() {
  static MoveState lastState = IDLE;  // 前回の状態

  // スイッチ状態を読み取り
  int switchState = digitalRead(SWITCH_PIN);

  // スイッチ開放時（HIGH）→ ON、GND接触時（LOW）→ OFF
  int targetAngle = (switchState == HIGH) ? ANGLE_ON : ANGLE_OFF;

  // デバッグ: スイッチ状態を表示（1秒おき）
  static unsigned long lastPrint = 0;
  static int lastLoggedState = -1;
  if (millis() - lastPrint > 1000 || switchState != lastLoggedState) {
    log("Switch: %d (%s) Target: %d CurrentAngle: %d", switchState, switchState == HIGH ? "HIGH" : "LOW", targetAngle, currentAngle);
    lastPrint = millis();
    lastLoggedState = switchState;
  }

  // デバウンス処理
  // 生のスイッチ状態が変わったらタイマーをリセット
  if (switchState != lastRawSwitchState) {
    lastDebounceTime = millis();
    lastRawSwitchState = switchState;
  }

  // デバウンス期間を経過し、かつ安定状態が変わった場合のみ処理
  // ※ IDLE状態の時のみスイッチ変化を受け付ける（重要！）
  if (moveState == IDLE &&
      (millis() - lastDebounceTime) >= DEBOUNCE_DELAY &&
      switchState != lastDebouncedState) {

    // 方向を判断して経由地を設定
    if (lastDebouncedState == HIGH && switchState == LOW) {
      // ON → OFF
      currentWaypoint = WAYPOINT_ANGLE_ON_TO_OFF;
      log("Direction: ON -> OFF");
    } else if (lastDebouncedState == LOW && switchState == HIGH) {
      // OFF → ON
      currentWaypoint = WAYPOINT_ANGLE_OFF_TO_ON;
      log("Direction: OFF -> ON");
    } else {
      // 不明な場合は現在位置を使用（経由地なし）
      currentWaypoint = targetAngle;
      log("Direction: Unknown (no waypoint)");
    }

    log("Target: %d Waypoint: %d", targetAngle, currentWaypoint);

    moveState = MOVING_TO_WAYPOINT;
    lastTargetAngle = targetAngle;
    lastDebouncedState = switchState;  // デバウンス後の状態を更新
  }

  // 移動状態マシン
  switch (moveState) {
    case IDLE:
      // 何もしない
      break;

    case MOVING_TO_WAYPOINT:
      // 経由地へ移動
      if (currentAngle != currentWaypoint) {
        if (currentAngle < currentWaypoint) {
          currentAngle++;
        } else {
          currentAngle--;
        }
        myServo.write(currentAngle);
        delay(STEP_DELAY);
      } else {
        // 経由地に到達、遅延開始
        log("Reached waypoint: %d", currentAngle);
        moveState = WAITING_AT_WAYPOINT;
        delayStartTime = millis();
      }
      break;

    case WAITING_AT_WAYPOINT:
      // 経由地で遅延
      {
        unsigned long elapsed = millis() - delayStartTime;
        if (elapsed >= WAYPOINT_DELAY) {
          // 遅延完了、目標地への移動開始
          log("Wait complete (%lums)", elapsed);
          log("Moving to target: %d", lastTargetAngle);
          moveState = MOVING_TO_TARGET;
        }
      }
      break;

    case MOVING_TO_TARGET:
      // 目標地へ移動
      if (currentAngle != lastTargetAngle) {
        if (currentAngle < lastTargetAngle) {
          currentAngle++;
        } else {
          currentAngle--;
        }
        myServo.write(currentAngle);
        delay(STEP_DELAY);
      } else {
        // 目標地に到達、完了
        log("Reached target: %d", currentAngle);
        moveState = IDLE;
      }
      break;
  }

  // 状態が変わったらログ出力（状態マシン実行後）
  if (moveState != lastState) {
    log("State: %d -> %d", lastState, moveState);
    lastState = moveState;
  }
}
