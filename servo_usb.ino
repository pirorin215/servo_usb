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
const int BUZZER_PIN = 6;     // ブザー

// 定義値
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long IR_COOLDOWN = 1000;  // チャタリング対策：1秒に変更

// 独自IRパターン定義（NEC形式を採用）
// NEC形式の正しい実装（32ビット完全版）
// ビット0 = 560,560  ビット1 = 560,1680

const uint16_t ON_PATTERN[] PROGMEM = {
  9000, 4500,  // NECリーダー
  // アドレス 0x00 (0000 0000)
  560,560, 560,560, 560,560, 560,560,
  560,560, 560,560, 560,560, 560,560,
  // アドレス反転 0xFF (1111 1111)
  560,1680, 560,1680, 560,1680, 560,1680,
  560,1680, 560,1680, 560,1680, 560,1680,
  // コマンド 0x01 (LSBファースト: 1,0,0,0,0,0,0,0)
  560,1680, 560,560, 560,560, 560,560,
  560,560, 560,560, 560,560, 560,560,
  // コマンド反転 0xFE (LSBファースト: 0,1,1,1,1,1,1,1)
  560,560, 560,1680, 560,1680, 560,1680,
  560,1680, 560,1680, 560,1680, 560,1680,
  560  // ストップビット
};
const int ON_PATTERN_LEN = sizeof(ON_PATTERN) / sizeof(ON_PATTERN[0]);

const uint16_t OFF_PATTERN[] PROGMEM = {
  9000, 4500,  // NECリーダー
  // アドレス 0x00 (0000 0000)
  560,560, 560,560, 560,560, 560,560,
  560,560, 560,560, 560,560, 560,560,
  // アドレス反転 0xFF (1111 1111)
  560,1680, 560,1680, 560,1680, 560,1680,
  560,1680, 560,1680, 560,1680, 560,1680,
  // コマンド 0x02 (LSBファースト: 0,1,0,0,0,0,0,0)
  560,560, 560,1680, 560,560, 560,560,
  560,560, 560,560, 560,560, 560,560,
  // コマンド反転 0xFD (LSBファースト: 1,0,1,1,1,1,1,1)
  560,1680, 560,560, 560,1680, 560,1680,
  560,1680, 560,1680, 560,1680, 560,1680,
  560  // ストップビット
};
const int OFF_PATTERN_LEN = sizeof(OFF_PATTERN) / sizeof(OFF_PATTERN[0]);

// コマンドタイプ
enum CommandType {
  CMD_MOVE = 0,
  CMD_WAIT = 1,
  CMD_DETACH = 2,
  CMD_END = 3
};

// コマンド構造体
struct Command {
  CommandType type;
  int value;
};

const Command OFF_COMMANDS[] = {
  {CMD_MOVE, 160}, {CMD_WAIT, 800}, {CMD_MOVE, 147}, {CMD_DETACH, 0}, {CMD_END, 0}
};

const Command ON_COMMANDS[] = {
  {CMD_MOVE,  80}, {CMD_WAIT, 800}, {CMD_MOVE,  90}, {CMD_DETACH, 0}, {CMD_END, 0}
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
uint16_t sendBuf[70];  // NEC完全形式用バッファ（67要素必要）
bool servoAttached = true;  // サーボのアタッチ状態を追跡

TaskContext currentTask = {-1, 0, 0};

// デバウンス用
unsigned long lastDebounceTime = 0;
int lastDebouncedState = -1;
int lastRawSwitchState = -1;

// IR受信関連
unsigned long lastIRReceiveTime = 0;

// ========== ヘルパー関数 ==========

// ブザー音関数（digitalWriteで制御）
void beepShort() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}

void beepLong() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);
}

void beepDouble() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}

// 赤外線信号送信（3.9.0対応）
void sendIRSignal(int state) {
  Serial.print("IR sending:");
  Serial.print(state == 1 ? "ON" : "OFF");
  Serial.print(" len:");

  if (state == 1) {
    for (int i = 0; i < ON_PATTERN_LEN; i++) {
      sendBuf[i] = pgm_read_word(&ON_PATTERN[i]);
    }
    Serial.print(ON_PATTERN_LEN);
    Serial.print(" [");
    for (int i = 0; i < ON_PATTERN_LEN && i < 67; i++) {
      Serial.print(sendBuf[i]);
      if (i < ON_PATTERN_LEN - 1 && i < 66) Serial.print(",");
    }
    Serial.println("]");
    IrSender.sendRaw(sendBuf, ON_PATTERN_LEN, 38);
    Serial.println("IR sent:ON");
  } else if (state == 0) {
    for (int i = 0; i < OFF_PATTERN_LEN; i++) {
      sendBuf[i] = pgm_read_word(&OFF_PATTERN[i]);
    }
    Serial.print(OFF_PATTERN_LEN);
    Serial.print(" [");
    for (int i = 0; i < OFF_PATTERN_LEN && i < 67; i++) {
      Serial.print(sendBuf[i]);
      if (i < OFF_PATTERN_LEN - 1 && i < 66) Serial.print(",");
    }
    Serial.println("]");
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

// checkIRPattern()関数は削除 - NECデコーダーを使用するため不要

void moveServoTo(int angle) {
  if (currentAngle != angle) {
    // デタッチ状態ならアタッチ
    if (!servoAttached) {
      myServo.attach(SERVO_PIN);
      servoAttached = true;
      Serial.println("Servo attached");
    }
    myServo.write(angle);
    currentAngle = angle;
  }
}

void activateScenario(int scenarioId) {
  currentTask.scenario = scenarioId;
  currentTask.currentStep = 0;
  currentTask.stepStartTime = millis();

  if (scenarioId == 0) {
    logicalState = 0;
  } else if (scenarioId == 1) {
    logicalState = 1;
  }

  Serial.print("Act:");
  Serial.println(logicalState);

  sendIRSignal(logicalState);
}

// 赤外線受信処理（NECデコーダー使用）
void handleIRReception() {
  if (millis() - lastIRReceiveTime < IR_COOLDOWN)
    return;

  if (IrReceiver.decode()) {
    lastIRReceiveTime = millis();

    // NECプロトコルとして解析できた場合
    if (IrReceiver.decodedIRData.protocol == NEC) {
      uint8_t address = IrReceiver.decodedIRData.address;
      uint8_t command = IrReceiver.decodedIRData.command;

      Serial.print("IR rx NEC addr:0x");
      Serial.print(address, HEX);
      Serial.print(" cmd:0x");
      Serial.println(command, HEX);

      // address=0x00 でコマンドを判定
      // 0x01 またはその反転 0x80 → ON
      // 0x02 またはその反転 0xFD → OFF
      if (address == 0x00) {
        if (command == 0x01 || command == 0x80) {  // ONコマンド
          Serial.println("-> ON");
          beepLong();  // ON信号受信: ピーー
          activateScenario(1);
        } else if (command == 0x02 || command == 0xFD) {  // OFFコマンド
          Serial.println("-> OFF");
          beepDouble();  // OFF信号受信: ピッピッ
          activateScenario(0);
        } else {
          Serial.print("-> UNKNOWN cmd:0x");
          Serial.println(command, HEX);
        }
      } else {
        Serial.print("-> UNKNOWN addr:0x");
        Serial.println(address, HEX);
      }

    } else {
      // NEC以外のプロトコルまたは解析失敗
      Serial.print("IR rx protocol:");
      Serial.print(IrReceiver.decodedIRData.protocol);
      Serial.print(" len:");
      Serial.println(IrReceiver.decodedIRData.rawDataPtr ? IrReceiver.decodedIRData.rawDataPtr->rawlen : 0);
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

    case CMD_DETACH:
      if (servoAttached) {
        myServo.detach();
        servoAttached = false;
        Serial.println("Servo detached (low power mode)");
      }
      currentTask.currentStep++;
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
  //while (!Serial);

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  myServo.attach(SERVO_PIN);

  // IRremote 3.9.0: begin()を使用
  IrReceiver.begin(IR_RECV_PIN, ENABLE_LED_FEEDBACK);
  IrSender.begin(IR_LED_PIN, DISABLE_LED_FEEDBACK);

  // USB通電検知時: ピッ
  beepShort();

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
