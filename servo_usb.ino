// Pro Micro + SG90 サーボモーター制御
// オルタネートスイッチで2つの角度を制御（経由地を通る移動）
// 赤外線送受信機能でスマートリモコンと連携
// IRremote 3.9.0対応
// 赤外線学習機能対応

#include <Servo.h>
#include <IRremote.hpp>  // 赤外線送受信ライブラリ 3.9.0
#include <Keyboard.h>    // HIDキーボード機能
#include <EEPROM.h>      // EEPROMアクセス

// ピン設定
const int SERVO_PIN = 5;
const int SWITCH_PIN = 4;
const int IR_LED_PIN = 3;     // 赤外線LED（送信）
const int IR_RECV_PIN = 2;    // 赤外線受信モジュール
const int BUZZER_PIN = 6;     // ブザー

// 定義値
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long IR_COOLDOWN = 1000;  // チャタリング対策：1秒に変更

// HIDキーコード定数（未定義の無効なキーコード）
const uint8_t WAKE_KEYCODE = 0xA5;

// キーボード初期化フラグ
bool keyboard_initialized = false;

// ブザー音の長さ定義
const unsigned long BEEP_SHORT_DURATION = 100;
const unsigned long BEEP_LONG_DURATION = 300;
const unsigned long BEEP_GAP_DURATION = 100;

// コマンドタイプ
enum CommandType {
  CMD_MOVE = 0,
  CMD_WAIT = 1,
  CMD_DETACH = 2,
  CMD_END = 3,
  CMD_KEY = 4
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
  {CMD_MOVE, 80}, {CMD_WAIT, 800}, {CMD_MOVE, 90}, {CMD_DETACH, 0}, {CMD_WAIT, 1000}, {CMD_KEY, WAKE_KEYCODE}, {CMD_END, 0}
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

// EEPROM設定
const uint16_t MAX_PATTERN_LENGTH = 70;
const char EEPROM_MAGIC[] = "IRL2";
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_ON_LEN_ADDR = 4;
const int EEPROM_ON_DATA_ADDR = 5;
const int EEPROM_OFF_LEN_ADDR = 150;
const int EEPROM_OFF_DATA_ADDR = 151;

// 学習モード
enum LearnMode {
  LEARN_NONE,
  LEARN_ON,
  LEARN_OFF
};
LearnMode learnMode = LEARN_NONE;
uint16_t learnedPattern[70];
uint8_t learnedPatternLength = 0;
bool eepromValid = false;

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

// ブザー状態管理（非ブロッキング制御用）
enum BuzzerPhase {
  BUZZER_OFF,
  BUZZER_BEEP1_ON,
  BUZZER_BEEP1_OFF,
  BUZZER_BEEP2_ON
};

struct BuzzerState {
  bool active;
  unsigned long phaseStartTime;
  BuzzerPhase phase;
  bool doubleBeep;  // ダブルビープモードかどうか
};
BuzzerState buzzerState = {false, 0, BUZZER_OFF, false};

// ========== ヘルパー関数 ==========

// ブザー更新関数（ループ内で呼び出し）
void updateBuzzer() {
  if (!buzzerState.active)
    return;

  unsigned long elapsed = millis() - buzzerState.phaseStartTime;

  switch (buzzerState.phase) {
    case BUZZER_BEEP1_ON:
      if (elapsed >= BEEP_SHORT_DURATION) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState.phase = BUZZER_BEEP1_OFF;
        buzzerState.phaseStartTime = millis();
      }
      break;

    case BUZZER_BEEP1_OFF:
      if (elapsed >= BEEP_GAP_DURATION) {
        if (buzzerState.doubleBeep) {
          // ダブルビープ：2回目のビープ開始
          buzzerState.phase = BUZZER_BEEP2_ON;
          buzzerState.phaseStartTime = millis();
          digitalWrite(BUZZER_PIN, HIGH);
        } else {
          // シングルビープ：終了
          buzzerState.active = false;
        }
      }
      break;

    case BUZZER_BEEP2_ON:
      if (elapsed >= BEEP_SHORT_DURATION) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState.active = false;
      }
      break;

    default:
      buzzerState.active = false;
      break;
  }
}

// ブザー開始関数（非ブロッキング）
void startShortBeep() {
  buzzerState.active = true;
  buzzerState.phaseStartTime = millis();
  buzzerState.phase = BUZZER_BEEP1_ON;
  buzzerState.doubleBeep = false;
  digitalWrite(BUZZER_PIN, HIGH);
}

void startLongBeep() {
  buzzerState.active = true;
  buzzerState.phaseStartTime = millis();
  buzzerState.phase = BUZZER_BEEP2_ON;
  buzzerState.doubleBeep = false;
  digitalWrite(BUZZER_PIN, HIGH);
}

void startDoubleBeep() {
  buzzerState.active = true;
  buzzerState.phaseStartTime = millis();
  buzzerState.phase = BUZZER_BEEP1_ON;
  buzzerState.doubleBeep = true;
  digitalWrite(BUZZER_PIN, HIGH);
}

// 既存のインターフェースを維持（下位互換性）
void beepShort() {
  startShortBeep();
}

void beepLong() {
  startLongBeep();
}

void beepDouble() {
  startDoubleBeep();
}

// ========== EEPROM操作関数 ==========

bool checkEEPROMValid() {
  char magic[4];
  for (int i = 0; i < 4; i++) {
    magic[i] = EEPROM.read(EEPROM_MAGIC_ADDR + i);
  }
  return (magic[0] == 'I' && magic[1] == 'R' && magic[2] == 'L' && magic[3] == '2');
}

void writeMagicNumber() {
  for (int i = 0; i < 4; i++) {
    EEPROM.update(EEPROM_MAGIC_ADDR + i, EEPROM_MAGIC[i]);
  }
}

void savePatternToEEPROM(bool isOnPattern, const uint16_t* pattern, uint8_t len) {
  int lenAddr = isOnPattern ? EEPROM_ON_LEN_ADDR : EEPROM_OFF_LEN_ADDR;
  int dataAddr = isOnPattern ? EEPROM_ON_DATA_ADDR : EEPROM_OFF_DATA_ADDR;

  EEPROM.update(lenAddr, len);

  for (int i = 0; i < len && i < MAX_PATTERN_LENGTH; i++) {
    EEPROM.update(dataAddr + (i * 2), lowByte(pattern[i]));
    EEPROM.update(dataAddr + (i * 2) + 1, highByte(pattern[i]));
  }

  writeMagicNumber();
  eepromValid = true;

  Serial.print("Saved ");
  Serial.print(isOnPattern ? "ON" : "OFF");
  Serial.print(" pattern to EEPROM (len=");
  Serial.print(len);
  Serial.println(")");
}

bool loadPatternFromEEPROM(bool isOnPattern, uint16_t* pattern, uint8_t* len) {
  if (!eepromValid) return false;

  int lenAddr = isOnPattern ? EEPROM_ON_LEN_ADDR : EEPROM_OFF_LEN_ADDR;
  int dataAddr = isOnPattern ? EEPROM_ON_DATA_ADDR : EEPROM_OFF_DATA_ADDR;

  *len = EEPROM.read(lenAddr);
  if (*len == 0 || *len > MAX_PATTERN_LENGTH) return false;

  for (int i = 0; i < *len; i++) {
    pattern[i] = word(EEPROM.read(dataAddr + (i * 2) + 1), EEPROM.read(dataAddr + (i * 2)));
  }

  return true;
}

void resetEEPROMPatterns() {
  EEPROM.update(EEPROM_MAGIC_ADDR, 0);
  eepromValid = false;
  Serial.println("EEPROM reset");
}

// ========== シリアルコマンド処理 ==========

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "LEARN_ON") {
      learnMode = LEARN_ON;
      Serial.println("OK LEARN_ON");
      startShortBeep();
      Serial.println("Send ON signal...");

    } else if (cmd == "LEARN_OFF") {
      learnMode = LEARN_OFF;
      Serial.println("OK LEARN_OFF");
      startShortBeep();
      Serial.println("Send OFF signal...");

    } else if (cmd == "RESET_PATTERNS") {
      resetEEPROMPatterns();
      Serial.println("OK RESET");

    } else if (cmd == "DUMP_PATTERNS") {
      Serial.println("=== EEPROM DUMP ===");

      uint8_t len;
      uint16_t pattern[MAX_PATTERN_LENGTH];

      // ONパターン
      if (loadPatternFromEEPROM(true, pattern, &len)) {
        Serial.print("ON Pattern (len=");
        Serial.print(len);
        Serial.println("):");
        Serial.print("[");
        for (int i = 0; i < len && i < 30; i++) {
          Serial.print(pattern[i]);
          if (i < len - 1 && i < 29) Serial.print(",");
        }
        if (len > 30) Serial.print("...");
        Serial.println("]");
      } else {
        Serial.println("ON Pattern: NOT FOUND");
      }

      // OFFパターン
      if (loadPatternFromEEPROM(false, pattern, &len)) {
        Serial.print("OFF Pattern (len=");
        Serial.print(len);
        Serial.println("):");
        Serial.print("[");
        for (int i = 0; i < len && i < 30; i++) {
          Serial.print(pattern[i]);
          if (i < len - 1 && i < 29) Serial.print(",");
        }
        if (len > 30) Serial.print("...");
        Serial.println("]");
      } else {
        Serial.println("OFF Pattern: NOT FOUND");
      }

      Serial.println("==================");
    }
  }
}

// Raw HIDキー送信関数（無効なキーコードを送信してディスプレイを起こす）
void sendRawHIDKey(uint8_t hidKeycode) {
  Serial.print("HID key:0x");
  Serial.println(hidKeycode, HEX);

  // 初回使用時のみKeyboard.begin()を呼ぶ
  if (!keyboard_initialized) {
    Serial.println("Keyboard init");
    Keyboard.begin();
    delay(1000);
    keyboard_initialized = true;
  }

  uint8_t buf[8] = {0};
  buf[2] = hidKeycode;
  HID().SendReport(2, buf, 8);
  delay(50);
  memset(buf, 0, 8);
  HID().SendReport(2, buf, 8);

  Serial.println("HID sent");
}

// 赤外線信号送信（3.9.0対応、EEPROMのみ）
void sendIRSignal(int state) {
  Serial.print("IR sending:");
  Serial.print(state == 1 ? "ON" : "OFF");

  uint8_t patternLen;

  if (state == 1) {
    if (!loadPatternFromEEPROM(true, sendBuf, &patternLen)) {
      Serial.println(" - ERROR: ON pattern not learned!");
      return;
    }
  } else {
    if (!loadPatternFromEEPROM(false, sendBuf, &patternLen)) {
      Serial.println(" - ERROR: OFF pattern not learned!");
      return;
    }
  }

  Serial.print(" len:");
  Serial.println(patternLen);

  // パターンをダンプ（最初の20要素）
  Serial.print("Sending pattern [");
  for (int i = 0; i < patternLen && i < 20; i++) {
    Serial.print(sendBuf[i]);
    if (i < patternLen - 1 && i < 19) Serial.print(",");
  }
  if (patternLen > 20) Serial.print("...");
  Serial.println("]");

  IrSender.sendRaw(sendBuf, patternLen, 38);
  Serial.print("IR sent:");
  Serial.println(state == 1 ? "ON" : "OFF");
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

// 赤外線受信処理（NECデコーダー使用、学習モード対応）
void handleIRReception() {
  if (millis() - lastIRReceiveTime < IR_COOLDOWN)
    return;

  if (IrReceiver.decode()) {
    lastIRReceiveTime = millis();

    // デバッグ：受信情報をダンプ（通常モード時）
    if (learnMode == LEARN_NONE) {
      Serial.print("IR RX Protocol:");
      Serial.print(IrReceiver.decodedIRData.protocol);
      Serial.print(" Addr:0x");
      Serial.print(IrReceiver.decodedIRData.address, HEX);
      Serial.print(" Cmd:0x");
      Serial.println(IrReceiver.decodedIRData.command, HEX);

      // RAWパターンもダンプ
      if (IrReceiver.decodedIRData.rawDataPtr != nullptr) {
        uint16_t rawLen = IrReceiver.decodedIRData.rawDataPtr->rawlen;
        Serial.print("Raw [");
        for (uint16_t i = 1; i < rawLen && i < 21; i++) {
          Serial.print(IrReceiver.decodedIRData.rawDataPtr->rawbuf[i]);
          if (i < rawLen - 1 && i < 20) Serial.print(",");
        }
        if (rawLen > 21) Serial.print("...");
        Serial.println("]");
      }
    }

    // 学習モード中の処理
    if (learnMode != LEARN_NONE) {
      if (IrReceiver.decodedIRData.rawDataPtr != nullptr) {
        uint16_t rawLen = IrReceiver.decodedIRData.rawDataPtr->rawlen;

        Serial.print("Learn: rawLen=");
        Serial.println(rawLen);

        if (rawLen > 1 && rawLen <= MAX_PATTERN_LENGTH + 1) {
          // パターンをコピー（rawbufの値を直接使用）
          for (uint16_t i = 0; i < rawLen - 1; i++) {
            learnedPattern[i] = IrReceiver.decodedIRData.rawDataPtr->rawbuf[i + 1];
          }
          learnedPatternLength = rawLen - 1;

          // パターンをダンプ
          Serial.print("Pattern [");
          for (int i = 0; i < learnedPatternLength && i < 20; i++) {
            Serial.print(learnedPattern[i]);
            if (i < learnedPatternLength - 1 && i < 19) Serial.print(",");
          }
          if (learnedPatternLength > 20) Serial.print("...");
          Serial.println("]");

          // EEPROMに保存
          savePatternToEEPROM(learnMode == LEARN_ON, learnedPattern, learnedPatternLength);

          startDoubleBeep();
          Serial.print("Learned ");
          Serial.print(learnMode == LEARN_ON ? "ON" : "OFF");
          Serial.print(" pattern (len=");
          Serial.print(learnedPatternLength);
          Serial.println(")");

        } else {
          Serial.println("Pattern length error");
          startLongBeep();
        }
      } else {
        Serial.println("No RAW data");
        startLongBeep();
      }

      learnMode = LEARN_NONE;
      IrReceiver.resume();
      return;
    }

    // 通常モード中の処理
    // NECプロトコルとして解析できた場合
    if (IrReceiver.decodedIRData.protocol == NEC) {
      uint8_t address = IrReceiver.decodedIRData.address;
      uint8_t command = IrReceiver.decodedIRData.command;

      // スマートリモコン対応: address=0x82
      if (address == 0x82) {
        if (command == 0x26) {  // ONコマンド（スマートリモコン）
          Serial.println("-> ON (smart remote)");
          beepLong();
          activateScenario(1);
        } else if (command == 0x3E) {  // OFFコマンド（スマートリモコン）
          Serial.println("-> OFF (smart remote)");
          beepDouble();
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
      Serial.println("-> Non-NEC protocol");
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

    case CMD_KEY:
      sendRawHIDKey(cmd.value);
      currentTask.currentStep++;
      currentTask.stepStartTime = millis();
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

  // EEPROMチェック
  eepromValid = checkEEPROMValid();
  if (eepromValid) {
    Serial.println("EEPROM OK");
  } else {
    Serial.println("EEPROM empty");
  }

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

  // ブザー状態更新（非ブロッキング）
  updateBuzzer();

  // シリアルコマンド処理
  handleSerialCommands();

  // スイッチ処理（学習モード中は無効化）
  if (learnMode == LEARN_NONE) {
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
  }

  executeTask();
  handleIRReception();

  if (currentTask.currentStep != lastStep) {
    Serial.print("S:");
    Serial.println(currentTask.currentStep);
    lastStep = currentTask.currentStep;
  }
}
