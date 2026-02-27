// Pro Micro + SG90 サーボモーター制御
// オルタネートスイッチで2つの角度を制御（経由地を通る移動）

#include <Servo.h>
#include <stdarg.h>

// ピン設定
const int SERVO_PIN = 5;
const int SWITCH_PIN = 4;

// 定義値
const unsigned long DEBOUNCE_DELAY = 50;       // デバウンス遅延(ms)

// コマンドタイプ
enum CommandType {
  CMD_MOVE = 0,    // 角度へ移動
  CMD_WAIT = 1,    // 待機(ms)
  CMD_END = 2      // 終了
};

// コマンド構造体
struct Command {
  CommandType type;
  int value;  // 角度または待機時間(ms)
};

// シナリオ定義配列
// フォーマット: {type, value} - type: CMD_MOVE(角度) or CMD_WAIT(ms) or CMD_END
const Command OFF_COMMANDS[] = {
  {CMD_MOVE, 180},    // 経由地180°へ移動
  {CMD_WAIT, 800},    // 800ms待機
  {CMD_MOVE, 167},    // 目標167°へ移動
  {CMD_END, 0}        // 終了
};

const Command ON_COMMANDS[] = {
  {CMD_MOVE, 120},    // 経由地120°へ移動
  {CMD_WAIT, 800},    // 800ms待機
  {CMD_MOVE, 130},    // 目標130°へ移動
  {CMD_END, 0}        // 終了
};

// シナリオ定義構造体
struct ScenarioDef {
  const Command* commands;
};

// シナリオ配列（インデックスがシナリオID）
const ScenarioDef SCENARIOS[] = {
  {OFF_COMMANDS},   // 0: OFFにする
  {ON_COMMANDS}     // 1: ONにする
};

// タスク実行コンテキスト
struct TaskContext {
  int scenario;              // シナリオID (-1: なし, 0: OFF, 1: ON)
  int currentStep;           // 現在のコマンドインデックス
  unsigned long stepStartTime;  // 現在のステップ開始時刻
};

// グローバル変数
Servo myServo;
int currentAngle = 0;             // 初期位置（setupで設定）

TaskContext currentTask = {
  -1,  // シナリオなし
  0,
  0
};

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

// ========== ヘルパー関数 ==========

// シナリオ選択（シナリオIDを返す）
int selectScenario(int currentSwitchState, int lastDebouncedState) {
  if (lastDebouncedState == HIGH && currentSwitchState == LOW)
    return 0;   // HIGH → LOW でOFFにする
  else if (lastDebouncedState == LOW && currentSwitchState == HIGH)
    return 1;   // LOW → HIGH でONにする
  return -1;    // シナリオなし
}

// 直接サーボ移動
void moveServoTo(int angle) {
  if (currentAngle != angle) {
    myServo.write(angle);
    currentAngle = angle;
    log("Servo moved to: %d", currentAngle);
  }
}

// シナリオ起動
void activateScenario(int scenarioId) {
  currentTask.scenario = scenarioId;
  currentTask.currentStep = 0;
  currentTask.stepStartTime = millis();

  log("Scenario started: %d", scenarioId);
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
      log("Step %d: Moved to %d°", currentTask.currentStep, cmd.value);
      currentTask.currentStep++;
      currentTask.stepStartTime = millis();
      break;

    case CMD_WAIT:
      if (millis() - currentTask.stepStartTime >= cmd.value) {
        log("Step %d: Waited %lums", currentTask.currentStep, cmd.value);
        currentTask.currentStep++;
        currentTask.stepStartTime = millis();
      }
      break;

    case CMD_END:
      log("Scenario complete");
      currentTask.scenario = -1;
      currentTask.currentStep = 0;
      break;
  }
}


void setup() {
  Serial.begin(9600);
  //while (!Serial);  // シリアルモニタを待つ

  // スイッチの設定（プルアップ抵抗方式）
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // 初期スイッチ状態を読み取り、初期位置を設定
  int switchState = digitalRead(SWITCH_PIN);
  // 各シナリオの最終移動コマンドを初期位置とする
  int initialScenarioId = (switchState == HIGH) ? 1 : 0;  // HIGH=ON(1), LOW=OFF(0)
  const Command* initialCommands = SCENARIOS[initialScenarioId].commands;

  // 最後のCMD_MOVEコマンドを探す
  for (int i = 0; initialCommands[i].type != CMD_END; i++) {
    if (initialCommands[i].type == CMD_MOVE)
      currentAngle = initialCommands[i].value;
  }

  // デバウンス状態の初期化
  lastDebouncedState = switchState;
  lastRawSwitchState = switchState;

  // サーボの設定
  myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);

  Serial.println("=== Startup ===");
  log("Initial currentAngle: %d", currentAngle);
  log("Initial switchState: %s", switchState == HIGH ? "HIGH" : "LOW");
}

void loop() {
  static int lastStep = -1;  // 前回のステップ

  // 1. スイッチ読み取り
  int switchState = digitalRead(SWITCH_PIN);

  // 2. デバッグログ
  static unsigned long lastPrint = 0;
  static int lastLoggedState = -1;
  if (millis() - lastPrint > 1000 || switchState != lastLoggedState) {
    log("Switch: %d (%s) CurrentAngle: %d", switchState, switchState == HIGH ? "HIGH" : "LOW", currentAngle);
    lastPrint = millis();
    lastLoggedState = switchState;
  }

  // 3. デバウンス処理
  if (switchState != lastRawSwitchState) {
    lastDebounceTime = millis();
    lastRawSwitchState = switchState;
  }

  // 4. シナリオ起動（IDLE時のみ）
  if (currentTask.scenario < 0 &&
      (millis() - lastDebounceTime) >= DEBOUNCE_DELAY &&
      switchState != lastDebouncedState) {

    int scenarioId = selectScenario(switchState, lastDebouncedState);
    if (scenarioId >= 0) {
      activateScenario(scenarioId);
    }
    lastDebouncedState = switchState;
  }

  // 5. タスク実行
  executeTask();

  // 6. ステップ変更ログ
  if (currentTask.currentStep != lastStep) {
    log("Step: %d -> %d", lastStep, currentTask.currentStep);
    lastStep = currentTask.currentStep;
  }
}
