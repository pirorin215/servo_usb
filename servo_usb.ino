// Pro Micro + SG90 サーボモーター制御
// オルタネートスイッチで2つの角度を制御（経由地を通る移動）
// 赤外線送受信機能でスマートリモコンと連携
// IRremote 3.9.0対応

#include <Servo.h>
#include <IRremote.hpp>  // 赤外線送受信ライブラリ 3.9.0

// ピン設定
const int SERVO_PIN = 5;
const int SWITCH_PIN = 4;
const int IR_LED_PIN = 3;     // 赤外線LED（送信）
const int IR_RECV_PIN = 2;    // 赤外線受信モジュール

// 定義値
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long IR_COOLDOWN = 500;

// 独自IRパターン定義（NEC形式を採用）
const uint16_t ON_PATTERN[] PROGMEM = {
  9000, 4500,  // NECリーダー
  560, 560, 560, 560, 560, 560, 560, 560, 560, 560, 560, 560, 560,  // アドレス(0x00)
  560, 560, 560, 560, 560, 560, 560, 560, 560, 560, 560, 560, 560, 560,  // コマンド(0x01)
  560  // ストップビット
};
const int ON_PATTERN_LEN = sizeof(ON_PATTERN) / sizeof(ON_PATTERN[0]);

const uint16_t OFF_PATTERN[] PROGMEM = {
  9000, 4500,  // NECリーダー
  560, 560, 560, 560, 560, 560, 560, 560, 560, 560, 560, 560, 560, 560,  // アドレス(0x00)
  560, 560, 560, 560, 560, 560, 560, 560, 560, 560, 560, 560, 560, 560,  // コマンド(0x02)
  560  // ストップビット
};
const int OFF_PATTERN_LEN = sizeof(OFF_PATTERN) / sizeof(OFF_PATTERN[0]);

// コマンドタイプ
enum CommandType {
  CMD_MOVE = 0,
  CMD_WAIT = 1,
  CMD_END = 2
};

// コマンド構造体
struct Command {
  CommandType type;
  int value;
};

const Command OFF_COMMANDS[] = {
  {CMD_MOVE, 180}, {CMD_WAIT, 800}, {CMD_MOVE, 167}, {CMD_END, 0}
};

const Command ON_COMMANDS[] = {
  {CMD_MOVE, 120}, {CMD_WAIT, 800}, {CMD_MOVE, 130}, {CMD_END, 0}
};

struct ScenarioDef {
  const Command* commands;
};

const ScenarioDef SCENARIOS[] = {
  {OFF_COMMANDS}, {ON_COMMANDS}
};

struct TaskContext {
  int scenario;
  int currentStep;
  unsigned long stepStartTime;
};

// グローバル変数
Servo myServo;
int currentAngle = 0;
int logicalState = -1;
uint16_t sendBuf[24];

TaskContext currentTask = {-1, 0, 0};

// デバウンス用
unsigned long lastDebounceTime = 0;
int lastDebouncedState = -1;
int lastRawSwitchState = -1;

// IR受信関連
unsigned long lastIRReceiveTime = 0;

// ========== ヘルパー関数 ==========

// 赤外線信号送信（3.9.0対応）
void sendIRSignal(int state) {
  if (state == 1) {
    for (int i = 0; i < ON_PATTERN_LEN; i++) {
      sendBuf[i] = pgm_read_word(&ON_PATTERN[i]);
    }
    IrSender.sendRaw(sendBuf, ON_PATTERN_LEN, 38);
    Serial.println("IR sent:ON");
  } else if (state == 0) {
    for (int i = 0; i < OFF_PATTERN_LEN; i++) {
      sendBuf[i] = pgm_read_word(&OFF_PATTERN[i]);
    }
    IrSender.sendRaw(sendBuf, OFF_PATTERN_LEN, 38);
    Serial.println("IR sent:OFF");
  }
}

// スイッチからのシナリオ選択
int selectScenarioFromSwitch(int currentSwitchState, int lastDebouncedState) {
  if (logicalState < 0) {
    if (lastDebouncedState == HIGH && currentSwitchState == LOW) return 0;
    else if (lastDebouncedState == LOW && currentSwitchState == HIGH) return 1;
    return -1;
  }

  if (logicalState == 0) {
    if (lastDebouncedState == LOW && currentSwitchState == HIGH) return 1;
    return -1;
  }

  if (logicalState == 1) {
    if (lastDebouncedState == HIGH && currentSwitchState == LOW) return 0;
    return -1;
  }

  return -1;
}

// 赤外線受信パターンの判定（簡易版：パルス数で判定）
int checkIRPattern(const uint16_t* pattern, int patternLen) {
  if (IrReceiver.decodedIRData.rawDataPtr == nullptr)
    return 0;

  int dataLen = IrReceiver.decodedIRData.rawDataPtr->rawlen;

  // パルス数だけで判定
  // ON: 20個以上
  // OFF: 10個以上19個以下
  if (dataLen >= 20) {
    Serial.print("-> Detected ON (len:");
    Serial.print(dataLen);
    Serial.println(")");
    return 1;  // ONと判定
  } else if (dataLen >= 10 && dataLen < 20) {
    Serial.print("-> Detected OFF (len:");
    Serial.print(dataLen);
    Serial.println(")");
    return 0;  // OFFと判定
  }

  return -1;  // 不明
}

void moveServoTo(int angle) {
  if (currentAngle != angle) {
    myServo.write(angle);
    currentAngle = angle;
  }
}

void activateScenario(int scenarioId) {
  currentTask.scenario = scenarioId;
  currentTask.currentStep = 0;
  currentTask.stepStartTime = millis();

  if (scenarioId == 0) logicalState = 0;
  else if (scenarioId == 1) logicalState = 1;

  Serial.print("Act:");
  Serial.println(logicalState);

  sendIRSignal(logicalState);
}

// 赤外線受信処理（3.9.0対応）
void handleIRReception() {
  if (millis() - lastIRReceiveTime < IR_COOLDOWN)
    return;

  if (IrReceiver.decode()) {
    lastIRReceiveTime = millis();

    if (IrReceiver.decodedIRData.rawDataPtr != nullptr) {
      int dataLen = IrReceiver.decodedIRData.rawDataPtr->rawlen;

      Serial.print("IR rx len:");
      Serial.print(dataLen);

      // 簡易判定：パルス数で判定
      if (dataLen >= 20) {
        Serial.print(" -> ON (current logicalState:");
        Serial.print(logicalState);
        // シナリオ実行中でも受け付ける
        activateScenario(1);
        IrReceiver.resume();
        return;
      } else if (dataLen >= 10 && dataLen < 20) {
        Serial.print(" -> OFF (current logicalState:");
        Serial.print(logicalState);
        // シナリオ実行中でも受け付ける
        activateScenario(0);
        IrReceiver.resume();
        return;
      }

      Serial.println(" -> UNKNOWN");
    }

    IrReceiver.resume();
  }
}

// タスク実行エンジン
void executeTask() {
  if (currentTask.scenario < 0)
    return;

  const Command* commands = SCENARIOS[currentTask.scenario].commands;
  Command cmd = commands[currentTask.currentStep];

  switch (cmd.type) {
    case CMD_MOVE:
      moveServoTo(cmd.value);
      currentTask.currentStep++;
      currentTask.stepStartTime = millis();
      break;

    case CMD_WAIT:
      if (millis() - currentTask.stepStartTime >= cmd.value) {
        currentTask.currentStep++;
        currentTask.stepStartTime = millis();
      }
      break;

    case CMD_END:
      Serial.println("Done");
      currentTask.scenario = -1;
      currentTask.currentStep = 0;
      break;
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  myServo.attach(SERVO_PIN);

  // IRremote 3.9.0: begin()を使用
  IrReceiver.begin(IR_RECV_PIN, ENABLE_LED_FEEDBACK);
  IrSender.begin(IR_LED_PIN, DISABLE_LED_FEEDBACK);

  Serial.println("START");

  // 初期スイッチ状態を読み取り、初期位置を設定
  int switchState = digitalRead(SWITCH_PIN);
  int initialScenarioId = (switchState == HIGH) ? 1 : 0;
  const Command* initialCommands = SCENARIOS[initialScenarioId].commands;

  for (int i = 0; initialCommands[i].type != CMD_END; i++) {
    if (initialCommands[i].type == CMD_MOVE)
      currentAngle = initialCommands[i].value;
  }

  lastDebouncedState = switchState;
  lastRawSwitchState = switchState;
  logicalState = initialScenarioId;

  myServo.write(currentAngle);

  Serial.print("A:"); Serial.println(currentAngle);
  Serial.print("Sw:"); Serial.println(switchState == HIGH ? "H" : "L");
  Serial.print("L:"); Serial.println(logicalState);
  Serial.println("READY");
}

void loop() {
  static int lastStep = -1;

  // スイッチ処理
  int switchState = digitalRead(SWITCH_PIN);

  if (switchState != lastRawSwitchState) {
    lastDebounceTime = millis();
    lastRawSwitchState = switchState;
  }

  if (currentTask.scenario < 0 &&
      (millis() - lastDebounceTime) >= DEBOUNCE_DELAY &&
      switchState != lastDebouncedState) {

    int scenarioId = selectScenarioFromSwitch(switchState, lastDebouncedState);
    if (scenarioId >= 0) {
      activateScenario(scenarioId);
    }
    lastDebouncedState = switchState;
  }

  executeTask();
  handleIRReception();

  if (currentTask.currentStep != lastStep) {
    Serial.print("S:");
    Serial.println(currentTask.currentStep);
    lastStep = currentTask.currentStep;
  }
}
