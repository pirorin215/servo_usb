// Pro Micro + SG90 サーボモーター制御
// オルタネートスイッチで2つの角度を制御（経由地を通る移動）

#include <Servo.h>
#include <stdarg.h>

// ピン設定
const int SERVO_PIN = 5;
const int SWITCH_PIN = 4;

// 定義値
const unsigned long DEBOUNCE_DELAY = 50;       // デバウンス遅延(ms)

// シナリオ列挙型
enum Scenario {
  SCENARIO_NONE = 0,
  SCENARIO_TURN_OFF = 1,     // OFFにする
  SCENARIO_TURN_ON = 2       // ONにする
};

// タスクフェーズ
enum TaskPhase {
  PHASE_IDLE = 0,
  PHASE_MOVE_TO_WAYPOINT = 1,
  PHASE_WAIT_AT_WAYPOINT = 2,
  PHASE_MOVE_TO_TARGET = 3
};

// シナリオ設定構造体
struct ScenarioConfig {
  int targetAngle;           // 目標角度
  int waypoint;              // 経由地角度
  unsigned long waypointDelay;  // 経由地での遅延(ms)
  const char* name;          // シナリオ名
};

// シナリオ定義配列
// フォーマット: {targetAngle, waypoint, waypointDelay, name}
const ScenarioConfig SCENARIO_CONFIGS[] = {
  {167, 180, 800, "OFFにする"},     // 目標、経由地、待機
  {130, 120, 800, "ONにする"}       // 目標、経由地、待機
};

// タスク実行コンテキスト
struct TaskContext {
  Scenario scenario;
  TaskPhase phase;
  unsigned long phaseStartTime;
  int targetAngle;
  int waypointAngle;
};

// グローバル変数
Servo myServo;
int currentAngle = 0;             // 初期位置（setupで設定）

TaskContext currentTask = {
  SCENARIO_NONE,
  PHASE_IDLE,
  0,
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

// シナリオ設定取得
ScenarioConfig getScenarioConfig(Scenario scenario) {
  if (scenario == SCENARIO_TURN_OFF)
    return SCENARIO_CONFIGS[0];
  if (scenario == SCENARIO_TURN_ON)
    return SCENARIO_CONFIGS[1];
  return {0, 0, 0, "NONE"};
}

// シナリオ選択
Scenario selectScenario(int currentSwitchState, int lastDebouncedState) {
  if (lastDebouncedState == HIGH && currentSwitchState == LOW)
    return SCENARIO_TURN_OFF;   // HIGH → LOW でOFFにする
  else if (lastDebouncedState == LOW && currentSwitchState == HIGH)
    return SCENARIO_TURN_ON;    // LOW → HIGH でONにする
  return SCENARIO_NONE;
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
void activateScenario(Scenario scenario) {
  currentTask.scenario = scenario;
  currentTask.phase = PHASE_MOVE_TO_WAYPOINT;
  currentTask.phaseStartTime = millis();

  ScenarioConfig config = getScenarioConfig(scenario);
  currentTask.targetAngle = config.targetAngle;
  currentTask.waypointAngle = config.waypoint;

  log("Scenario: %s (waypoint: %d→target: %d, delay: %lums)",
      config.name, config.waypoint, config.targetAngle, config.waypointDelay);
}

// タスク実行エンジン
void executeTask() {
  switch (currentTask.phase) {
    case PHASE_IDLE:
      break;

    case PHASE_MOVE_TO_WAYPOINT:
      moveServoTo(currentTask.waypointAngle);
      log("Waypoint reached: %d", currentTask.waypointAngle);
      currentTask.phase = PHASE_WAIT_AT_WAYPOINT;
      currentTask.phaseStartTime = millis();
      break;

    case PHASE_WAIT_AT_WAYPOINT:
      {
        ScenarioConfig config = getScenarioConfig(currentTask.scenario);
        if (millis() - currentTask.phaseStartTime >= config.waypointDelay) {
          log("Waypoint wait complete");
          currentTask.phase = PHASE_MOVE_TO_TARGET;
        }
      }
      break;

    case PHASE_MOVE_TO_TARGET:
      moveServoTo(currentTask.targetAngle);
      log("Target reached: %d", currentTask.targetAngle);
      currentTask.phase = PHASE_IDLE;
      currentTask.scenario = SCENARIO_NONE;
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
  ScenarioConfig initialConfig = (switchState == HIGH) ?
    SCENARIO_CONFIGS[1] :  // ONにする
    SCENARIO_CONFIGS[0];   // OFFにする
  currentAngle = initialConfig.targetAngle;

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
  static TaskPhase lastPhase = PHASE_IDLE;  // 前回のフェーズ

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
  if (currentTask.phase == PHASE_IDLE &&
      (millis() - lastDebounceTime) >= DEBOUNCE_DELAY &&
      switchState != lastDebouncedState) {

    Scenario scenario = selectScenario(switchState, lastDebouncedState);
    if (scenario != SCENARIO_NONE) {
      activateScenario(scenario);
    }
    lastDebouncedState = switchState;
  }

  // 5. タスク実行
  executeTask();

  // 6. フェーズ変更ログ
  if (currentTask.phase != lastPhase) {
    ScenarioConfig config = getScenarioConfig(currentTask.scenario);
    log("Phase: %d -> %d [%s]", lastPhase, currentTask.phase, config.name);
    lastPhase = currentTask.phase;
  }
}
