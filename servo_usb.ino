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
const uint8_t  MATCH_THRESHOLD_PCT = 90;    // 一致とみなす最低一致率[%]
const uint8_t  STRICT_MATCH_COUNT = 32;    // 最初のN要素は厳密に一致させる（16ビット分）

// IR信号処理パラメータ
const uint16_t MIN_RAW_SIGNAL_LENGTH = 10; // 最小IR信号長（NECリピート信号フィルタ用）
const uint16_t BIT_THRESHOLD_US = 25;      // 0/1ビット判定閾値[μs]

// デバッグ出力設定（0:無効, 1:有効）
#define DEBUG_MODE 1

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
const uint8_t EEPROM_RAW_DATA_SIZE = MAX_PATTERN_LENGTH * 2;  // uint16_t × 70 = 140バイト
const char EEPROM_MAGIC[] = "IRL5";  // バージョンアップ（プロトコル情報追加）
const int EEPROM_MAGIC_ADDR = 0;

// EEPROMアドレス計算用マクロ（マジックナンバー削減）
#define CALC_PATTERN_DATA_ADDR(lenAddr) ((lenAddr) + 1)
#define CALC_PROTOCOL_ADDR(dataAddr) ((dataAddr) + EEPROM_RAW_DATA_SIZE)
#define CALC_ADDR_ADDR(protocolAddr) ((protocolAddr) + 1)
#define CALC_CMD_ADDR(addrAddr) ((addrAddr) + 1)
#define CALC_NEXT_PATTERN_LEN_ADDR(cmdAddr) ((cmdAddr) + 1)

// 各パターンのEEPROM配置（計算式で自動算出）
const int EEPROM_ON_LEN_ADDR = 4;
const int EEPROM_ON_DATA_ADDR = CALC_PATTERN_DATA_ADDR(EEPROM_ON_LEN_ADDR);
const int EEPROM_ON_PROTOCOL_ADDR = CALC_PROTOCOL_ADDR(EEPROM_ON_DATA_ADDR);
const int EEPROM_ON_ADDR_ADDR = CALC_ADDR_ADDR(EEPROM_ON_PROTOCOL_ADDR);
const int EEPROM_ON_CMD_ADDR = CALC_CMD_ADDR(EEPROM_ON_ADDR_ADDR);

const int EEPROM_OFF_LEN_ADDR = CALC_NEXT_PATTERN_LEN_ADDR(EEPROM_ON_CMD_ADDR);
const int EEPROM_OFF_DATA_ADDR = CALC_PATTERN_DATA_ADDR(EEPROM_OFF_LEN_ADDR);
const int EEPROM_OFF_PROTOCOL_ADDR = CALC_PROTOCOL_ADDR(EEPROM_OFF_DATA_ADDR);
const int EEPROM_OFF_ADDR_ADDR = CALC_ADDR_ADDR(EEPROM_OFF_PROTOCOL_ADDR);
const int EEPROM_OFF_CMD_ADDR = CALC_CMD_ADDR(EEPROM_OFF_ADDR_ADDR);

const int EEPROM_PLAYPAUSE_LEN_ADDR = CALC_NEXT_PATTERN_LEN_ADDR(EEPROM_OFF_CMD_ADDR);
const int EEPROM_PLAYPAUSE_DATA_ADDR = CALC_PATTERN_DATA_ADDR(EEPROM_PLAYPAUSE_LEN_ADDR);
const int EEPROM_PLAYPAUSE_PROTOCOL_ADDR = CALC_PROTOCOL_ADDR(EEPROM_PLAYPAUSE_DATA_ADDR);
const int EEPROM_PLAYPAUSE_ADDR_ADDR = CALC_ADDR_ADDR(EEPROM_PLAYPAUSE_PROTOCOL_ADDR);
const int EEPROM_PLAYPAUSE_CMD_ADDR = CALC_CMD_ADDR(EEPROM_PLAYPAUSE_ADDR_ADDR);

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

// EEPROMアドレス構造体（重複排除のため）
struct EEPROMAddresses {
  int lenAddr;
  int dataAddr;
  int protocolAddr;
  int addrAddr;
  int cmdAddr;
  const char* name;
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

// 配列表示ヘルパー関数（重複排除）
void printArray(const uint16_t* arr, uint8_t len) {
  Serial.print(F("["));
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(arr[i]);
    if (i < len - 1) Serial.print(F(","));
  }
  Serial.println(F("]"));
}

// プロトコル情報表示ヘルパー関数（重複排除）
void printProtocolInfo(uint8_t address, uint8_t command) {
  Serial.print(F(" Addr:0x"));
  Serial.print(address, HEX);
  Serial.print(F(" Cmd:0x"));
  Serial.println(command, HEX);
}

// ========== EEPROM操作関数 ==========

// PatternTypeに対応するEEPROMアドレスを取得
EEPROMAddresses getEEPROMAddresses(PatternType patternType) {
  EEPROMAddresses addrs;
  switch (patternType) {
    case PATTERN_ON:
      addrs.lenAddr = EEPROM_ON_LEN_ADDR;
      addrs.dataAddr = EEPROM_ON_DATA_ADDR;
      addrs.protocolAddr = EEPROM_ON_PROTOCOL_ADDR;
      addrs.addrAddr = EEPROM_ON_ADDR_ADDR;
      addrs.cmdAddr = EEPROM_ON_CMD_ADDR;
      addrs.name = "ON";
      break;
    case PATTERN_OFF:
      addrs.lenAddr = EEPROM_OFF_LEN_ADDR;
      addrs.dataAddr = EEPROM_OFF_DATA_ADDR;
      addrs.protocolAddr = EEPROM_OFF_PROTOCOL_ADDR;
      addrs.addrAddr = EEPROM_OFF_ADDR_ADDR;
      addrs.cmdAddr = EEPROM_OFF_CMD_ADDR;
      addrs.name = "OFF";
      break;
    case PATTERN_PLAYPAUSE:
      addrs.lenAddr = EEPROM_PLAYPAUSE_LEN_ADDR;
      addrs.dataAddr = EEPROM_PLAYPAUSE_DATA_ADDR;
      addrs.protocolAddr = EEPROM_PLAYPAUSE_PROTOCOL_ADDR;
      addrs.addrAddr = EEPROM_PLAYPAUSE_ADDR_ADDR;
      addrs.cmdAddr = EEPROM_PLAYPAUSE_CMD_ADDR;
      addrs.name = "PLAYPAUSE";
      break;
  }
  return addrs;
}

bool checkEEPROMValid() {
  char magic[4];
  for (int i = 0; i < 4; i++) {
    magic[i] = EEPROM.read(EEPROM_MAGIC_ADDR + i);
  }
#if DEBUG_MODE
  Serial.print(F("EEPROM Magic: ["));
  Serial.print(magic[0]);
  Serial.print(magic[1]);
  Serial.print(magic[2]);
  Serial.print(magic[3]);
  Serial.println(F("]"));
#endif
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
  EEPROMAddresses addrs = getEEPROMAddresses(patternType);

  // RAWデータを保存
  EEPROM.update(addrs.lenAddr, len);
  for (int i = 0; i < len && i < MAX_PATTERN_LENGTH; i++) {
    EEPROM.update(addrs.dataAddr + (i * 2), lowByte(pattern[i]));
    EEPROM.update(addrs.dataAddr + (i * 2) + 1, highByte(pattern[i]));
  }

  // プロトコル情報を保存
  EEPROM.update(addrs.protocolAddr, protocol);
  EEPROM.update(addrs.addrAddr, address);
  EEPROM.update(addrs.cmdAddr, command);

  writeMagicNumber();
  eepromValid = true;

  Serial.print(F("Saved "));
  Serial.print(addrs.name);
  Serial.print(F(" pattern (len="));
  Serial.print(len);
  Serial.print(F(") addr=0x"));
  Serial.print(address, HEX);
  Serial.print(F(" cmd=0x"));
  Serial.println(command, HEX);
}

bool loadPatternFromEEPROM(PatternType patternType, uint16_t* pattern, uint8_t* len) {
  if (!eepromValid) return false;

  EEPROMAddresses addrs = getEEPROMAddresses(patternType);

  *len = EEPROM.read(addrs.lenAddr);
  if (*len == 0 || *len > MAX_PATTERN_LENGTH) return false;

  for (int i = 0; i < *len; i++) {
    pattern[i] = word(EEPROM.read(addrs.dataAddr + (i * 2) + 1), EEPROM.read(addrs.dataAddr + (i * 2)));
  }

  return true;
}

PatternProtocolInfo loadProtocolInfo(PatternType patternType) {
  PatternProtocolInfo info = {false, 0, 0, 0};

  if (!eepromValid) return info;  // EEPROMが無効な場合は空情報を返す

  EEPROMAddresses addrs = getEEPROMAddresses(patternType);

  info.protocol = EEPROM.read(addrs.protocolAddr);
  info.address = EEPROM.read(addrs.addrAddr);
  info.command = EEPROM.read(addrs.cmdAddr);

  // プロトコルが0以外なら有効とみなす
  info.isValid = (info.protocol != 0);

  return info;
}

bool hasProtocolInfo(PatternType patternType) {
  PatternProtocolInfo info = loadProtocolInfo(patternType);
  return info.isValid;
}

// パターンダンプ用ヘルパー関数
void dumpPattern(PatternType patternType) {
  uint8_t len;
  uint16_t pattern[MAX_PATTERN_LENGTH];
  EEPROMAddresses addrs = getEEPROMAddresses(patternType);

  if (!loadPatternFromEEPROM(patternType, pattern, &len)) {
    Serial.print(addrs.name);
    Serial.println(F(" Pattern: NOT FOUND"));
    return;
  }

  Serial.print(addrs.name);
  Serial.print(F(" Pattern (len="));
  Serial.print(len);

  PatternProtocolInfo proto = loadProtocolInfo(patternType);
  if (proto.isValid) {
    Serial.print(F(") Protocol:"));
    Serial.print(proto.protocol);
    printProtocolInfo(proto.address, proto.command);
  } else {
    Serial.println(F(") - RAW only"));
  }

  printArray(pattern, len);

  // バイナリ表現も表示
  printBinaryPattern(pattern, len);
}

void resetEEPROMPatterns() {
  EEPROM.update(EEPROM_MAGIC_ADDR, 0);
  eepromValid = false;
  Serial.println(F("EEPROM reset"));
}

// ========== シリアルコマンド処理 ==========

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "LEARN_ON") {
      learnMode = LEARN_ON;
      Serial.println(F("OK LEARN_ON"));
      startShortBeep();
      Serial.println(F("Send ON signal..."));

    } else if (cmd == "LEARN_OFF") {
      learnMode = LEARN_OFF;
      Serial.println(F("OK LEARN_OFF"));
      startShortBeep();
      Serial.println(F("Send OFF signal..."));

    } else if (cmd == "LEARN_PLAYPAUSE") {
      learnMode = LEARN_PLAYPAUSE;
      Serial.println(F("OK LEARN_PLAYPAUSE"));
      startShortBeep();
      Serial.println(F("Send PLAYPAUSE signal..."));

    } else if (cmd == "RESET_PATTERNS") {
      resetEEPROMPatterns();
      Serial.println(F("OK RESET"));

    } else if (cmd == "DUMP_PATTERNS") {
      Serial.println(F("=== EEPROM DUMP ==="));

      dumpPattern(PATTERN_ON);
      dumpPattern(PATTERN_OFF);
      dumpPattern(PATTERN_PLAYPAUSE);

      Serial.println(F("=================="));
    }
  }
}

// RAWパターンをバイナリ文字列に変換（デバッグ用）
void printBinaryPattern(const uint16_t* pattern, uint8_t len) {
  // インデックス0,1はリーダー部（9ms + 4.5ms）
  // インデックス2以降がデータ部（2要素で1ビット）
  Serial.print(F("Binary: "));

  int bitsPrinted = 0;
  for (uint8_t i = 2; i < len && i < 2 + 64; i += 2) {
    // LOW期間の長さで0/1を判定（しきい値25）
    if (i + 1 < len) {
      uint16_t lowTime = pattern[i + 1];
      Serial.print(lowTime < BIT_THRESHOLD_US ? "0" : "1");
      bitsPrinted++;

      // 8ビットごとにスペース
      if (bitsPrinted % 8 == 0 && bitsPrinted < 32) {
        Serial.print(F(" "));
      }
    }
  }
  Serial.println();
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
      // 厳密モード：完全一致のみ受け入れる
      uint16_t diff = (stored[i] > received[i]) ? (stored[i] - received[i]) : (received[i] - stored[i]);
      if (diff == 0) {  // 完全一致のみ
        strictMatched++;
        matched++;
      }
    } else {
      // 緩いモード：通常の許容値
      if (pulseMatches(stored[i], received[i])) matched++;
    }
  }

#if DEBUG_MODE
  // デバッグ：厳密モードでの一致数を表示
  Serial.print(F("  StrictMatch: ")); Serial.print(strictMatched); Serial.print(F("/")); Serial.println(STRICT_MATCH_COUNT);
#endif

  return (int)matched * 100 / compareLen;
}

// 受信RAWパターンが指定パターンタイプと一致するか判定
bool matchesStoredPattern(PatternType patternType,
                           const uint16_t* received, uint16_t receivedLen) {
  uint16_t stored[MAX_PATTERN_LENGTH];
  uint8_t storedLen = 0;

  if (!loadPatternFromEEPROM(patternType, stored, &storedLen)) return false;

  int score = calcMatchScore(stored, storedLen, received, receivedLen);

#if DEBUG_MODE
  // デバッグ出力
  EEPROMAddresses addrs = getEEPROMAddresses(patternType);
  Serial.print(F("  Match ")); Serial.print(addrs.name);
  Serial.print(F(": ")); Serial.print(score); Serial.println(F("%"));
#endif

  return (score >= MATCH_THRESHOLD_PCT);
}

// HID Keyboardキー送信
void sendKeyboardKey(uint8_t keycode) {
  Serial.print(F("HID Keyboard:0x"));
  Serial.println(keycode, HEX);

  Keyboard.write(keycode);
  Serial.println(F("HID Keyboard sent"));
}

// HID Consumerキー送信（Play/Pauseなど）
void sendConsumerKey(uint16_t keycode) {
  Serial.print(F("HID Consumer:0x"));
  Serial.println(keycode, HEX);

  Consumer.write(keycode);
  Serial.println(F("HID Consumer sent"));
}

// 赤外線信号送信（3.9.0対応、EEPROMのみ）
void sendIRSignal(int state) {
  uint8_t patternLen;
  PatternType ptype = (state == 1) ? PATTERN_ON : PATTERN_OFF;

  if (!loadPatternFromEEPROM(ptype, sendBuf, &patternLen)) {
    Serial.println(F(" - ERROR: pattern not learned!"));
    return;
  }

  IrSender.sendRaw(sendBuf, patternLen, 38);
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
      Serial.println(F("Servo attached"));
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

  Serial.print(F("Act:"));
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
    Serial.println(F("IR: no raw data"));
    IrReceiver.resume();
    return;
  }

  // NECリピート信号を無視（rawlenが短すぎる場合はリピート）
  // ただし学習モードの場合はスキップ（異常な信号も学習するため）
  uint16_t rawLen = IrReceiver.decodedIRData.rawDataPtr->rawlen;
  if (learnMode == LEARN_NONE && rawLen < MIN_RAW_SIGNAL_LENGTH) {
    Serial.print(F("IR: repeat signal ignored (rawLen="));
    Serial.print(rawLen);
    Serial.println(F(")"));
    IrReceiver.resume();
    return;
  }

  // rawLenが0または異常値の場合、学習モードでなければ無視
  if (learnMode == LEARN_NONE && rawLen == 0) {
    Serial.println(F("IR: no data (rawLen=0)"));
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
    Serial.print(F("Learn: rawLen="));
    Serial.println(rawLen);
    Serial.print(F("sigLen="));
    Serial.println(sigLen);

    // プロトコル情報を取得
    uint8_t rxProtocol = IrReceiver.decodedIRData.protocol;
    uint8_t rxAddress = IrReceiver.decodedIRData.address;
    uint8_t rxCommand = IrReceiver.decodedIRData.command;

    // デバッグ：プロトコル情報を常に表示
    Serial.print(F("Detected Protocol: "));
    Serial.print(rxProtocol);
    if (rxProtocol != UNKNOWN) {
      printProtocolInfo(rxAddress, rxCommand);
    } else {
      Serial.println(F(" (UNKNOWN)"));
    }

    if (sigLen >= 3 && sigLen <= MAX_PATTERN_LENGTH) {
      for (uint16_t i = 0; i < sigLen; i++) learnedPattern[i] = rxBuf[i];
      learnedPatternLength = (uint8_t)sigLen;

      printArray(learnedPattern, learnedPatternLength);

      Serial.print(F("Protocol: "));
      Serial.print(rxProtocol);
      if (rxProtocol != UNKNOWN) {
        Serial.print(F(" Addr:0x"));
        Serial.print(rxAddress, HEX);
        Serial.print(F(" Cmd:0x"));
        Serial.println(rxCommand, HEX);
      } else {
        Serial.println(F(" (UNKNOWN)"));
      }

      // NECプロトコル警告
      if (rxProtocol != NEC) {
        Serial.println(F(""));
        Serial.println(F("=================================="));
        Serial.println(F("WARNING: This device is designed for NEC protocol only!"));
        Serial.println(F("Non-NEC protocols may not work properly."));
        Serial.println(F("Recommended: Use a NEC remote for learning."));
        Serial.println(F("=================================="));
        Serial.println(F(""));
        startLongBeep();
        // 学習を中止
        learnMode = LEARN_NONE;
        IrReceiver.resume();
        return;
      }

      PatternType ptype;
      if      (learnMode == LEARN_ON)        ptype = PATTERN_ON;
      else if (learnMode == LEARN_OFF)       ptype = PATTERN_OFF;
      else                                   ptype = PATTERN_PLAYPAUSE;

      EEPROMAddresses addrs = getEEPROMAddresses(ptype);

      // プロトコル情報付きで保存
      savePatternToEEPROM(ptype, learnedPattern, learnedPatternLength, rxProtocol, rxAddress, rxCommand);
      startDoubleBeep();
      Serial.print(F("Learned "));
      Serial.print(addrs.name);
      Serial.print(F(" pattern (len="));
      Serial.print(learnedPatternLength);
      Serial.println(F(")"));

    } else {
      Serial.print(F("Pattern length error: sigLen="));
      Serial.print(sigLen);
      Serial.print(F(" (expected 3-"));
      Serial.print(MAX_PATTERN_LENGTH);
      Serial.println(F(")"));
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

#if DEBUG_MODE
  Serial.print(F("IR RX Protocol:"));
  Serial.print(rxProtocol);
  if (rxProtocol != UNKNOWN) {
    printProtocolInfo(rxAddress, rxCommand);
  }
  Serial.print(F(" rawLen:"));
  Serial.print(sigLen);
  Serial.print(F(" ["));
  for (uint16_t i = 0; i < sigLen && i < 10; i++) {
    Serial.print(rxBuf[i]);
    if (i < sigLen - 1 && i < 9) Serial.print(F(","));
  }
  if (sigLen > 10) Serial.print(F("..."));
  Serial.println(F("]"));
#endif

  bool matched = false;
  PatternType matchedPattern = PATTERN_ON;  // ダミー初期値

  // 未知のアドレスチェック（学習済みアドレス以外は無視）
  // ライブラリがアドレスを提供している場合のみチェック
  if (rxAddress != 0) {
    PatternProtocolInfo onInfo = loadProtocolInfo(PATTERN_ON);
    PatternProtocolInfo offInfo = loadProtocolInfo(PATTERN_OFF);
    PatternProtocolInfo ppInfo = loadProtocolInfo(PATTERN_PLAYPAUSE);

    bool onAddrMatch = onInfo.isValid && rxAddress == onInfo.address;
    bool offAddrMatch = offInfo.isValid && rxAddress == offInfo.address;
    bool ppAddrMatch = ppInfo.isValid && rxAddress == ppInfo.address;

    if (!onAddrMatch && !offAddrMatch && !ppAddrMatch) {
      // 学習済みアドレスと完全に一致しない場合は無視
      Serial.print(F("Unknown address: 0x"));
      Serial.println(rxAddress, HEX);
      IrReceiver.resume();
      return;
    }
  }

  // Method A: プロトコル照合（高速・高精度）
  if (rxAddress != 0 && rxCommand != 0) {
    PatternProtocolInfo onInfo = loadProtocolInfo(PATTERN_ON);
    if (onInfo.isValid && rxAddress == onInfo.address && rxCommand == onInfo.command) {
      Serial.println(F("-> ON"));
      matched = true;
      matchedPattern = PATTERN_ON;
    } else {
      PatternProtocolInfo offInfo = loadProtocolInfo(PATTERN_OFF);
      if (offInfo.isValid && rxAddress == offInfo.address && rxCommand == offInfo.command) {
        Serial.println(F("-> OFF"));
        matched = true;
        matchedPattern = PATTERN_OFF;
      } else {
        PatternProtocolInfo ppInfo = loadProtocolInfo(PATTERN_PLAYPAUSE);
        if (ppInfo.isValid && rxAddress == ppInfo.address && rxCommand == ppInfo.command) {
          Serial.println(F("-> PLAYPAUSE"));
          matched = true;
          matchedPattern = PATTERN_PLAYPAUSE;
        }
      }
    }
  }

  // Method B: プロトコル照合失敗時のみRAW波形照合
  if (!matched) {
    if (matchesStoredPattern(PATTERN_PLAYPAUSE, rxBuf, sigLen)) {
      Serial.println(F("-> PLAYPAUSE (by RAW)"));
      matched = true;
      matchedPattern = PATTERN_PLAYPAUSE;
    } else if (matchesStoredPattern(PATTERN_ON, rxBuf, sigLen)) {
      Serial.println(F("-> ON (by RAW)"));
      matched = true;
      matchedPattern = PATTERN_ON;
    } else if (matchesStoredPattern(PATTERN_OFF, rxBuf, sigLen)) {
      Serial.println(F("-> OFF (by RAW)"));
      matched = true;
      matchedPattern = PATTERN_OFF;
    }
  }

  // マッチング結果を処理
  if (matched) {
    // 共通：受信パターンをダンプ
    Serial.print(F("Received pattern "));
    printArray(rxBuf, sigLen);

    // バイナリ表現も表示
    printBinaryPattern(rxBuf, sigLen);

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
        beepShort();  // ピッ（短い音）
        Serial.println(F("Act:PLAYPAUSE"));
        sendConsumerKey(PLAYPAUSE_KEYCODE);
        Serial.println(F("Done"));
        break;
    }
  } else {
    Serial.println(F("-> UNKNOWN (no match)"));
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
        Serial.println(F("Servo detached (low power mode)"));
      }
      currentTask.currentStep++;
      break;

    case CMD_END:
      Serial.println(F("Done"));
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
    Serial.println(F("EEPROM OK"));
  } else {
    Serial.println(F("EEPROM empty"));
  }

  // USB通電検知時: ピッ
  beepShort();

  Serial.println(F("START"));

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

  Serial.print(F("A:")); Serial.println(currentAngle);
  Serial.print(F("Sw:")); Serial.println(switchState == HIGH ? "H" : "L");
  Serial.print(F("L:")); Serial.println(logicalState);
  Serial.println(F("READY"));
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
    Serial.print(F("S:"));
    Serial.println(currentTask.currentStep);
    lastStep = currentTask.currentStep;
  }
}
