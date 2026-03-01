// Pro Micro + SG90 サーボモーター制御
// オルタネートスイッチで2つの角度を制御（経由地を通る移動）
// 赤外線送受信機能でスマートリモコンと連携
// IRremote 3.9.0対応
// 赤外線学習機能対応
// HID-Projectライブラリ使用（Keyboard + Consumer Page対応）

#include <Servo.h>
#include <IRremote.hpp>  // 赤外線送受信ライブラリ 3.9.0
#include <HID-Project.h> // HID-Project: Keyboard + Consumer対応
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

// HIDキーコード定数（HID-Project形式）
const uint8_t WAKE_KEYCODE = 0xA5;           // Keyboard: 未定義キーでディスプレイウェイク
const uint16_t PLAYPAUSE_KEYCODE = MEDIA_PLAY_PAUSE;  // Consumer: Play/Pause (HID-Project定義済み)

// RAW照合パラメータ
const uint16_t MATCH_TOLERANCE_US = 200;   // 絶対誤差[μs]
const uint8_t  MATCH_TOLERANCE_PCT = 20;    // 相対誤差[%]
const uint8_t  MATCH_LEN_TOLERANCE = 2;     // パターン長の許容差
const uint8_t  MATCH_THRESHOLD_PCT = 85;    // 一致とみなす最低一致率[%]
const uint8_t  STRICT_MATCH_COUNT = 10;    // 最初のN要素は厳密に一致させる

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
  CMD_KEY = 4,        // Keyboardキー
  CMD_CONSUMER = 5    // Consumerキー（Play/Pauseなど）
};

// コマンド構造体
struct Command {
  CommandType type;
  uint16_t value;  // HIDキーコード（Keyboard: 8bit, Consumer: 16bit）
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
const char EEPROM_MAGIC[] = "IRL5";  // バージョンアップ（プロトコル情報追加）
const int EEPROM_MAGIC_ADDR = 0;

// 各パターンのEEPROM配置（RAWデータ + プロトコル情報）
const int EEPROM_ON_LEN_ADDR = 4;
const int EEPROM_ON_DATA_ADDR = 5;              // 5 〜 144 (RAW: 140バイト)
const int EEPROM_ON_PROTOCOL_ADDR = 145;        // 145
const int EEPROM_ON_ADDR_ADDR = 146;            // 146
const int EEPROM_ON_CMD_ADDR = 147;              // 147

const int EEPROM_OFF_LEN_ADDR = 148;
const int EEPROM_OFF_DATA_ADDR = 149;            // 149 〜 288 (RAW: 140バイト)
const int EEPROM_OFF_PROTOCOL_ADDR = 289;       // 289
const int EEPROM_OFF_ADDR_ADDR = 290;           // 290
const int EEPROM_OFF_CMD_ADDR = 291;             // 291

const int EEPROM_PLAYPAUSE_LEN_ADDR = 292;
const int EEPROM_PLAYPAUSE_DATA_ADDR = 293;     // 293 〜 432 (RAW: 140バイト)
const int EEPROM_PLAYPAUSE_PROTOCOL_ADDR = 437;  // 437
const int EEPROM_PLAYPAUSE_ADDR_ADDR = 438;      // 438
const int EEPROM_PLAYPAUSE_CMD_ADDR = 439;        // 439

// 学習モード
enum LearnMode {
  LEARN_NONE,
  LEARN_ON,
  LEARN_OFF,
  LEARN_PLAYPAUSE
};
LearnMode learnMode = LEARN_NONE;

// パターンタイプ
enum PatternType {
  PATTERN_ON,
  PATTERN_OFF,
  PATTERN_PLAYPAUSE
};

// 学習したプロトコル情報
struct PatternProtocolInfo {
  bool isValid;
  uint8_t protocol;
  uint8_t address;
  uint8_t command;
};

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
  // IRL3, IRL4, IRL5 のいずれかを受け入れる（下位互換性）
  return (magic[0] == 'I' && magic[1] == 'R' && magic[2] == 'L' &&
          (magic[3] == '3' || magic[3] == '4' || magic[3] == '5'));
}

void writeMagicNumber() {
  for (int i = 0; i < 4; i++) {
    EEPROM.update(EEPROM_MAGIC_ADDR + i, EEPROM_MAGIC[i]);
  }
}

void savePatternToEEPROM(PatternType patternType, const uint16_t* pattern, uint8_t len,
                           uint8_t protocol, uint8_t address, uint8_t command) {
  int lenAddr, dataAddr, protocolAddr, addrAddr, cmdAddr;
  const char* name;

  switch (patternType) {
    case PATTERN_ON:
      lenAddr = EEPROM_ON_LEN_ADDR;
      dataAddr = EEPROM_ON_DATA_ADDR;
      protocolAddr = EEPROM_ON_PROTOCOL_ADDR;
      addrAddr = EEPROM_ON_ADDR_ADDR;
      cmdAddr = EEPROM_ON_CMD_ADDR;
      name = "ON";
      break;
    case PATTERN_OFF:
      lenAddr = EEPROM_OFF_LEN_ADDR;
      dataAddr = EEPROM_OFF_DATA_ADDR;
      protocolAddr = EEPROM_OFF_PROTOCOL_ADDR;
      addrAddr = EEPROM_OFF_ADDR_ADDR;
      cmdAddr = EEPROM_OFF_CMD_ADDR;
      name = "OFF";
      break;
    case PATTERN_PLAYPAUSE:
      lenAddr = EEPROM_PLAYPAUSE_LEN_ADDR;
      dataAddr = EEPROM_PLAYPAUSE_DATA_ADDR;
      protocolAddr = EEPROM_PLAYPAUSE_PROTOCOL_ADDR;
      addrAddr = EEPROM_PLAYPAUSE_ADDR_ADDR;
      cmdAddr = EEPROM_PLAYPAUSE_CMD_ADDR;
      name = "PLAYPAUSE";
      break;
  }

  // RAWデータを保存
  EEPROM.update(lenAddr, len);
  for (int i = 0; i < len && i < MAX_PATTERN_LENGTH; i++) {
    EEPROM.update(dataAddr + (i * 2), lowByte(pattern[i]));
    EEPROM.update(dataAddr + (i * 2) + 1, highByte(pattern[i]));
  }

  // プロトコル情報を保存
  EEPROM.update(protocolAddr, protocol);
  EEPROM.update(addrAddr, address);
  EEPROM.update(cmdAddr, command);

  writeMagicNumber();
  eepromValid = true;

  Serial.print("Saved ");
  Serial.print(name);
  Serial.print(" pattern (len=");
  Serial.print(len);
  Serial.print(") addr=0x");
  Serial.print(address, HEX);
  Serial.print(" cmd=0x");
  Serial.println(command, HEX);
}

bool loadPatternFromEEPROM(PatternType patternType, uint16_t* pattern, uint8_t* len) {
  if (!eepromValid) return false;

  int lenAddr, dataAddr;

  switch (patternType) {
    case PATTERN_ON:
      lenAddr = EEPROM_ON_LEN_ADDR;
      dataAddr = EEPROM_ON_DATA_ADDR;
      break;
    case PATTERN_OFF:
      lenAddr = EEPROM_OFF_LEN_ADDR;
      dataAddr = EEPROM_OFF_DATA_ADDR;
      break;
    case PATTERN_PLAYPAUSE:
      lenAddr = EEPROM_PLAYPAUSE_LEN_ADDR;
      dataAddr = EEPROM_PLAYPAUSE_DATA_ADDR;
      break;
  }

  *len = EEPROM.read(lenAddr);
  if (*len == 0 || *len > MAX_PATTERN_LENGTH) return false;

  for (int i = 0; i < *len; i++) {
    pattern[i] = word(EEPROM.read(dataAddr + (i * 2) + 1), EEPROM.read(dataAddr + (i * 2)));
  }

  return true;
}

PatternProtocolInfo loadProtocolInfo(PatternType patternType) {
  PatternProtocolInfo info = {false, 0, 0, 0};
  int protocolAddr, addrAddr, cmdAddr;

  switch (patternType) {
    case PATTERN_ON:
      protocolAddr = EEPROM_ON_PROTOCOL_ADDR;
      addrAddr = EEPROM_ON_ADDR_ADDR;
      cmdAddr = EEPROM_ON_CMD_ADDR;
      break;
    case PATTERN_OFF:
      protocolAddr = EEPROM_OFF_PROTOCOL_ADDR;
      addrAddr = EEPROM_OFF_ADDR_ADDR;
      cmdAddr = EEPROM_OFF_CMD_ADDR;
      break;
    case PATTERN_PLAYPAUSE:
      protocolAddr = EEPROM_PLAYPAUSE_PROTOCOL_ADDR;
      addrAddr = EEPROM_PLAYPAUSE_ADDR_ADDR;
      cmdAddr = EEPROM_PLAYPAUSE_CMD_ADDR;
      break;
  }

  info.protocol = EEPROM.read(protocolAddr);
  info.address = EEPROM.read(addrAddr);
  info.command = EEPROM.read(cmdAddr);

  // プロトコルが0以外なら有効とみなす
  info.isValid = (info.protocol != 0);

  return info;
}

bool hasProtocolInfo(PatternType patternType) {
  PatternProtocolInfo info = loadProtocolInfo(patternType);
  return info.isValid;
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

    } else if (cmd == "LEARN_PLAYPAUSE") {
      learnMode = LEARN_PLAYPAUSE;
      Serial.println("OK LEARN_PLAYPAUSE");
      startShortBeep();
      Serial.println("Send PLAYPAUSE signal...");

    } else if (cmd == "RESET_PATTERNS") {
      resetEEPROMPatterns();
      Serial.println("OK RESET");

    } else if (cmd == "DUMP_PATTERNS") {
      Serial.println("=== EEPROM DUMP ===");

      uint8_t len;
      uint16_t pattern[MAX_PATTERN_LENGTH];
      PatternProtocolInfo proto;

      // ONパターン
      if (loadPatternFromEEPROM(PATTERN_ON, pattern, &len)) {
        Serial.print("ON Pattern (len=");
        Serial.print(len);
        proto = loadProtocolInfo(PATTERN_ON);
        if (proto.isValid) {
          Serial.print(") Protocol:");
          Serial.print(proto.protocol);
          Serial.print(" Addr:0x");
          Serial.print(proto.address, HEX);
          Serial.print(" Cmd:0x");
          Serial.println(proto.command, HEX);
        } else {
          Serial.println(") - RAW only");
        }
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
      if (loadPatternFromEEPROM(PATTERN_OFF, pattern, &len)) {
        Serial.print("OFF Pattern (len=");
        Serial.print(len);
        proto = loadProtocolInfo(PATTERN_OFF);
        if (proto.isValid) {
          Serial.print(") Protocol:");
          Serial.print(proto.protocol);
          Serial.print(" Addr:0x");
          Serial.print(proto.address, HEX);
          Serial.print(" Cmd:0x");
          Serial.println(proto.command, HEX);
        } else {
          Serial.println(") - RAW only");
        }
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

      // PLAYPAUSEパターン
      if (loadPatternFromEEPROM(PATTERN_PLAYPAUSE, pattern, &len)) {
        Serial.print("PLAYPAUSE Pattern (len=");
        Serial.print(len);
        proto = loadProtocolInfo(PATTERN_PLAYPAUSE);
        if (proto.isValid) {
          Serial.print(") Protocol:");
          Serial.print(proto.protocol);
          Serial.print(" Addr:0x");
          Serial.print(proto.address, HEX);
          Serial.print(" Cmd:0x");
          Serial.println(proto.command, HEX);
        } else {
          Serial.println(") - RAW only");
        }
        Serial.print("[");
        for (int i = 0; i < len && i < 30; i++) {
          Serial.print(pattern[i]);
          if (i < len - 1 && i < 29) Serial.print(",");
        }
        if (len > 30) Serial.print("...");
        Serial.println("]");
      } else {
        Serial.println("PLAYPAUSE Pattern: NOT FOUND");
      }

      Serial.println("==================");
    }
  }
}

// ========== RAW波形照合 ==========

// 1パルスが一致しているか（絶対誤差 OR 相対誤差）
bool pulseMatches(uint16_t a, uint16_t b) {
  uint16_t diff = (a > b) ? (a - b) : (b - a);
  if (diff <= MATCH_TOLERANCE_US) return true;
  uint16_t larger = (a > b) ? a : b;
  if (larger == 0) return true;
  // 相対誤差チェック（整数演算）
  if ((uint32_t)diff * 100 <= (uint32_t)larger * MATCH_TOLERANCE_PCT) return true;
  return false;
}

// 受信RAWパターンと保存済みパターンの類似度を計算（0〜100%）
int calcMatchScore(const uint16_t* stored, uint8_t storedLen,
                   const uint16_t* received, uint16_t receivedLen) {
  // パターン長チェック
  int lenDiff = (int)receivedLen - (int)storedLen;
  if (lenDiff < 0) lenDiff = -lenDiff;
  if (lenDiff > MATCH_LEN_TOLERANCE) return -1;

  // 短い方の長さで比較
  uint8_t compareLen = (storedLen < receivedLen) ? storedLen : (uint8_t)receivedLen;
  if (compareLen == 0) return -1;

  uint8_t matched = 0;
  uint8_t strictMatched = 0;  // 厳密モードで一致した数

  // 最初のSTRICT_MATCH_COUNT要素は厳密に比較
  for (uint8_t i = 0; i < compareLen; i++) {
    bool strictMode = (i < STRICT_MATCH_COUNT && i < (uint8_t)STRICT_MATCH_COUNT && i < storedLen && i < receivedLen);

    if (strictMode) {
      // 厳密モード：完全一致に近い必要がある
      uint16_t diff = (stored[i] > received[i]) ? (stored[i] - received[i]) : (received[i] - stored[i]);
      if (diff <= 2) {  // 2μs以内なら一致とみなす
        strictMatched++;
        matched++;
      }
    } else {
      // 緩いモード：通常の許容値
      if (pulseMatches(stored[i], received[i])) matched++;
    }
  }

  // デバッグ：厳密モードでの一致数を表示
  Serial.print("  StrictMatch: "); Serial.print(strictMatched); Serial.print("/"); Serial.println(STRICT_MATCH_COUNT);

  return (int)matched * 100 / compareLen;
}

// 受信RAWパターンが指定パターンタイプと一致するか判定
bool matchesStoredPattern(PatternType patternType,
                           const uint16_t* received, uint16_t receivedLen) {
  uint16_t stored[MAX_PATTERN_LENGTH];
  uint8_t storedLen = 0;

  if (!loadPatternFromEEPROM(patternType, stored, &storedLen)) return false;

  int score = calcMatchScore(stored, storedLen, received, receivedLen);

  // デバッグ出力
  const char* name;
  switch (patternType) {
    case PATTERN_ON:        name = "ON";        break;
    case PATTERN_OFF:       name = "OFF";       break;
    case PATTERN_PLAYPAUSE: name = "PLAYPAUSE"; break;
  }
  Serial.print("  Match "); Serial.print(name);
  Serial.print(": "); Serial.print(score); Serial.println("%");

  return (score >= MATCH_THRESHOLD_PCT);
}

// HID Keyboardキー送信
void sendKeyboardKey(uint8_t keycode) {
  Serial.print("HID Keyboard:0x");
  Serial.println(keycode, HEX);

  Keyboard.write(keycode);
  Serial.println("HID Keyboard sent");
}

// HID Consumerキー送信（Play/Pauseなど）
void sendConsumerKey(uint16_t keycode) {
  Serial.print("HID Consumer:0x");
  Serial.println(keycode, HEX);

  Consumer.write(keycode);
  Serial.println("HID Consumer sent");
}

// 赤外線信号送信（3.9.0対応、EEPROMのみ）
void sendIRSignal(int state) {
  Serial.print("IR sending:");
  Serial.print(state == 1 ? "ON" : "OFF");

  uint8_t patternLen;
  PatternType ptype = (state == 1) ? PATTERN_ON : PATTERN_OFF;

  if (!loadPatternFromEEPROM(ptype, sendBuf, &patternLen)) {
    Serial.println(" - ERROR: pattern not learned!");
    return;
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

// 赤外線受信処理（RAW波形照合方式）
void handleIRReception() {
  if (millis() - lastIRReceiveTime < IR_COOLDOWN)
    return;

  if (!IrReceiver.decode()) return;

  lastIRReceiveTime = millis();

  // RAWデータ取得
  if (IrReceiver.decodedIRData.rawDataPtr == nullptr) {
    Serial.println("IR: no raw data");
    IrReceiver.resume();
    return;
  }

  // NECリピート信号を無視（rawlenが短すぎる場合はリピート）
  uint16_t rawLen = IrReceiver.decodedIRData.rawDataPtr->rawlen;
  if (rawLen < 10) {
    Serial.print("IR: repeat signal ignored (rawLen=");
    Serial.print(rawLen);
    Serial.println(")");
    IrReceiver.resume();
    return;
  }

  // rawbuf[0]はギャップなので1始まりで読む
  // 実際の信号長は rawLen-1
  uint16_t sigLen = rawLen - 1;
  uint16_t rxBuf[MAX_PATTERN_LENGTH + 5];

  if (sigLen > MAX_PATTERN_LENGTH + 4) sigLen = MAX_PATTERN_LENGTH + 4;
  for (uint16_t i = 0; i < sigLen; i++) {
    rxBuf[i] = IrReceiver.decodedIRData.rawDataPtr->rawbuf[i + 1];
  }

  // ========== 学習モード ==========
  if (learnMode != LEARN_NONE) {
    Serial.print("Learn: rawLen=");
    Serial.println(rawLen);

    if (sigLen >= 3 && sigLen <= MAX_PATTERN_LENGTH) {
      for (uint16_t i = 0; i < sigLen; i++) learnedPattern[i] = rxBuf[i];
      learnedPatternLength = (uint8_t)sigLen;

      Serial.print("Pattern [");
      for (int i = 0; i < learnedPatternLength && i < 20; i++) {
        Serial.print(learnedPattern[i]);
        if (i < learnedPatternLength - 1 && i < 19) Serial.print(",");
      }
      if (learnedPatternLength > 20) Serial.print("...");
      Serial.println("]");

      // プロトコル情報を取得
      uint8_t protocol = IrReceiver.decodedIRData.protocol;
      uint8_t address = IrReceiver.decodedIRData.address;
      uint8_t command = IrReceiver.decodedIRData.command;

      Serial.print("Protocol: ");
      Serial.print(protocol);
      if (protocol != UNKNOWN) {
        Serial.print(" Addr:0x");
        Serial.print(address, HEX);
        Serial.print(" Cmd:0x");
        Serial.println(command, HEX);
      } else {
        Serial.println(" (RAW only)");
      }

      PatternType ptype;
      const char* pname;
      if      (learnMode == LEARN_ON)        { ptype = PATTERN_ON;        pname = "ON"; }
      else if (learnMode == LEARN_OFF)       { ptype = PATTERN_OFF;       pname = "OFF"; }
      else                                   { ptype = PATTERN_PLAYPAUSE; pname = "PLAYPAUSE"; }

      // プロトコル情報付きで保存
      savePatternToEEPROM(ptype, learnedPattern, learnedPatternLength, protocol, address, command);
      startDoubleBeep();
      Serial.print("Learned ");
      Serial.print(pname);
      Serial.print(" pattern (len=");
      Serial.print(learnedPatternLength);
      Serial.println(")");

    } else {
      Serial.println("Pattern length error");
      startLongBeep();
    }

    learnMode = LEARN_NONE;
    IrReceiver.resume();
    return;
  }

  // ========== 通常モード：Method A+B（プロトコル照合+RAWフォールバック）==========

  // プロトコル情報を取得
  uint8_t rxProtocol = IrReceiver.decodedIRData.protocol;
  uint8_t rxAddress = IrReceiver.decodedIRData.address;
  uint8_t rxCommand = IrReceiver.decodedIRData.command;

  Serial.print("IR RX Protocol:");
  Serial.print(rxProtocol);
  if (rxProtocol != UNKNOWN) {
    Serial.print(" Addr:0x");
    Serial.print(rxAddress, HEX);
    Serial.print(" Cmd:0x");
    Serial.print(rxCommand, HEX);
  }
  Serial.print(" rawLen:");
  Serial.print(sigLen);
  Serial.print(" [");
  for (uint16_t i = 0; i < sigLen && i < 10; i++) {
    Serial.print(rxBuf[i]);
    if (i < sigLen - 1 && i < 9) Serial.print(",");
  }
  if (sigLen > 10) Serial.print("...");
  Serial.println("]");

  bool matched = false;
  PatternType matchedPattern = PATTERN_ON;  // ダミー初期値

  // Method A: プロトコルがNECであれば、address+commandで照合
  if (rxProtocol == NEC) {
    PatternProtocolInfo onInfo = loadProtocolInfo(PATTERN_ON);
    PatternProtocolInfo offInfo = loadProtocolInfo(PATTERN_OFF);
    PatternProtocolInfo ppInfo = loadProtocolInfo(PATTERN_PLAYPAUSE);

    // ON照合
    if (onInfo.isValid && rxAddress == onInfo.address && rxCommand == onInfo.command) {
      Serial.println("-> ON (by protocol)");
      matched = true;
      matchedPattern = PATTERN_ON;
    }
    // OFF照合
    else if (offInfo.isValid && rxAddress == offInfo.address && rxCommand == offInfo.command) {
      Serial.println("-> OFF (by protocol)");
      matched = true;
      matchedPattern = PATTERN_OFF;
    }
    // PLAYPAUSE照合
    else if (ppInfo.isValid && rxAddress == ppInfo.address && rxCommand == ppInfo.command) {
      Serial.println("-> PLAYPAUSE (by protocol)");
      matched = true;
      matchedPattern = PATTERN_PLAYPAUSE;
    }
  }

  // Method B: プロトコル照合失敗時、またはUNKNOWNの場合はRAW波形照合
  if (!matched) {
    if (matchesStoredPattern(PATTERN_ON, rxBuf, sigLen)) {
      Serial.println("-> ON (by RAW)");
      matched = true;
      matchedPattern = PATTERN_ON;
    } else if (matchesStoredPattern(PATTERN_OFF, rxBuf, sigLen)) {
      Serial.println("-> OFF (by RAW)");
      matched = true;
      matchedPattern = PATTERN_OFF;
    } else if (matchesStoredPattern(PATTERN_PLAYPAUSE, rxBuf, sigLen)) {
      Serial.println("-> PLAYPAUSE (by RAW)");
      matched = true;
      matchedPattern = PATTERN_PLAYPAUSE;
    }
  }

  // マッチング結果を処理
  if (matched) {
    switch (matchedPattern) {
      case PATTERN_ON:
        beepLong();
        activateScenario(1);
        break;
      case PATTERN_OFF:
        beepDouble();
        activateScenario(0);
        break;
      case PATTERN_PLAYPAUSE:
        sendConsumerKey(PLAYPAUSE_KEYCODE);
        break;
    }
  } else {
    Serial.println("-> UNKNOWN (no match)");
  }

  IrReceiver.resume();
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
      sendKeyboardKey(cmd.value);
      currentTask.currentStep++;
      currentTask.stepStartTime = millis();
      break;

    case CMD_CONSUMER:
      sendConsumerKey(cmd.value);
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

  // HID-Project初期化
  Keyboard.begin();
  Consumer.begin();
  keyboard_initialized = true;

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
