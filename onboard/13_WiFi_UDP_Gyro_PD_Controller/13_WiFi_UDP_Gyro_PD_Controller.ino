#include <M5Unified.h>
#include <M5HatBugC.h>
#include <WiFi.h>
#include <WiFiUdp.h>

M5HatBugC bugc;
WiFiUDP udp;

// ==================== 競技ネットワーク設定 ====================
const char* wifiSsid = "CQ-WS-24Gnew";
const char* wifiPassword = "00000000";
const uint16_t udpListenPort = 5005;

// ==================== 低速・方向別PWM設定 ====================
// PCからpwm_limit=28が届く場合、実際の上限は前進28、後退20となる。
// 本番机で距離を測り、後退が速ければbackwardPwmCeilingを下げる。
const float forwardPwmCeiling = 32.0f;
const float backwardPwmCeiling = 20.0f;
const float lateralPwmCeiling = 24.0f;  // 左右移動は実機確認後に使用する。
const float rotationPwmCeiling = 28.0f;
const float absoluteMotorPwmCeiling = 32.0f;

// 低すぎるPWMで一部車輪だけ止まることを避ける最低始動値。
const float forwardMinimumPwm = 18.0f;
const float backwardMinimumPwm = 16.0f;
const float lateralMinimumPwm = 18.0f;
const float inPlaceTurnMinimumPwm = 18.0f;

// 10～12の実機結果から引き継ぐジャイロPD設定。
const float correctionPolarity = -1.0f;
const float kp = 0.25f;
const float kd = 0.04f;
const float yawDeadbandDeg = 1.5f;
const float maxHeadingCorrection = 5.0f;
const float maxCorrectionStep = 0.5f;
const float gyroFilterAlpha = 0.20f;

// 急発進を避けるための成分別変化上限（10ms周期あたりのPWM）。
const float translationStepPerCycle = 0.50f;
const float rotationStepPerCycle = 0.80f;
const uint32_t controlIntervalMs = 10;
const uint32_t displayIntervalMs = 200;
const uint32_t serialIntervalMs = 200;
const uint32_t calibrationMs = 1500;
const uint32_t wifiRetryIntervalMs = 3000;
const uint32_t defaultCommandTtlMs = 350;
const uint32_t minimumCommandTtlMs = 100;
const uint32_t maximumCommandTtlMs = 1000;

const size_t packetBufferSize = 512;
char packetBuffer[packetBufferSize];

float gyroZBias = 0.0f;
float gyroNoise = 0.0f;
float filteredGyroZ = 0.0f;
float yawDeg = 0.0f;
float headingTargetYawDeg = 0.0f;
float headingCorrection = 0.0f;

float requestedForward = 0.0f;
float requestedLateral = 0.0f;
float requestedTurn = 0.0f;
float requestedHeadingErrorDeg = 0.0f;
float appliedForwardPwm = 0.0f;
float appliedLateralPwm = 0.0f;
float appliedRotationPwm = 0.0f;

uint8_t requestedPwmLimit = 0;
uint32_t commandTtlMs = defaultCommandTtlMs;
uint32_t lastPacketMs = 0;
uint32_t lastSequence = 0;
uint32_t lastControlMs = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastSerialMs = 0;
uint32_t lastWifiAttemptMs = 0;
uint32_t previousGyroUs = 0;

bool armed = false;
bool commandActive = false;
bool haveSequence = false;
bool udpStarted = false;
bool localHeadingMode = false;
char stateReason[40] = "BOOT";

int8_t clampMotorCommand(float value) {
  if (value > 100.0f) return 100;
  if (value < -100.0f) return -100;
  return static_cast<int8_t>(roundf(value));
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value > maxValue) return maxValue;
  if (value < minValue) return minValue;
  return value;
}

float approach(float current, float target, float maxStep) {
  return current + clampFloat(target - current, -maxStep, maxStep);
}

void setStateReason(const char* reason) {
  strncpy(stateReason, reason, sizeof(stateReason) - 1);
  stateReason[sizeof(stateReason) - 1] = '\0';
}

void stopAllMotors() {
  bugc.move(MOVE_STOP);
}

void resetAppliedCommands() {
  appliedForwardPwm = 0.0f;
  appliedLateralPwm = 0.0f;
  appliedRotationPwm = 0.0f;
  headingCorrection = 0.0f;
}

void stopImmediately(const char* reason) {
  stopAllMotors();
  resetAppliedCommands();
  commandActive = false;
  setStateReason(reason);
}

void disarm(const char* reason) {
  armed = false;
  stopImmediately(reason);
  bugc.setAllLedColor(0x200000, 0x200000);
}

void drawStatus() {
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(4, 4);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(armed ? GREEN : YELLOW);
  M5.Display.println(armed ? "UDP ARMED" : "UDP SAFE");
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(WHITE);
  M5.Display.printf("WiFi: %s\n",
                    WiFi.status() == WL_CONNECTED ? "OK" : "WAIT");
  if (WiFi.status() == WL_CONNECTED) {
    M5.Display.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  }
  M5.Display.printf("UDP: %u  age:%lu\n", udpListenPort,
                    lastPacketMs == 0 ? 0UL : millis() - lastPacketMs);
  M5.Display.printf("F/L/T: %.2f %.2f %.2f\n", requestedForward,
                    requestedLateral, requestedTurn);
  M5.Display.printf("Yaw/Tgt: %.1f %.1f\n", yawDeg, headingTargetYawDeg);
  M5.Display.printf("PWM lim: %u\n", requestedPwmLimit);
  M5.Display.printf("State: %s\n", stateReason);
  M5.Display.println(armed ? "A: EMERGENCY STOP" : "A: CAL + ARM");
}

bool calibrateGyroZ() {
  stopAllMotors();
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(4, 4);
  M5.Display.setTextColor(YELLOW);
  M5.Display.setTextSize(2);
  M5.Display.println("CALIBRATING");
  M5.Display.setTextSize(1);
  M5.Display.println("KEEP ROBOT STILL");

  double sum = 0.0;
  double sumSquares = 0.0;
  uint32_t samples = 0;
  const uint32_t startMs = millis();
  while (millis() - startMs < calibrationMs) {
    M5.update();
    if (M5.Imu.update()) {
      const auto data = M5.Imu.getImuData();
      const float value = data.gyro.z;
      sum += value;
      sumSquares += value * value;
      samples++;
    }
    delay(2);
  }

  if (samples == 0) {
    setStateReason("IMU NO DATA");
    return false;
  }
  gyroZBias = static_cast<float>(sum / samples);
  const double variance = (sumSquares / samples) - gyroZBias * gyroZBias;
  gyroNoise = sqrt(variance > 0.0 ? variance : 0.0);
  if (gyroNoise > 2.0f) {
    setStateReason("CAL MOVED RETRY");
    return false;
  }

  filteredGyroZ = 0.0f;
  yawDeg = 0.0f;
  headingTargetYawDeg = 0.0f;
  previousGyroUs = micros();
  Serial.printf("GYRO_CAL,bias=%.4f,noise=%.4f,samples=%lu\n", gyroZBias,
                gyroNoise, static_cast<unsigned long>(samples));
  return true;
}

void integrateGyro() {
  if (!M5.Imu.update()) return;
  const uint32_t nowUs = micros();
  if (previousGyroUs == 0) {
    previousGyroUs = nowUs;
    return;
  }
  const float dt = (nowUs - previousGyroUs) * 0.000001f;
  previousGyroUs = nowUs;
  const auto data = M5.Imu.getImuData();
  const float rawGyroZ = data.gyro.z - gyroZBias;
  filteredGyroZ += gyroFilterAlpha * (rawGyroZ - filteredGyroZ);
  if (dt > 0.0f && dt < 0.050f) yawDeg += filteredGyroZ * dt;
}

const char* findJsonValue(const char* json, const char* key) {
  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char* found = strstr(json, pattern);
  if (found == nullptr) return nullptr;
  found = strchr(found + strlen(pattern), ':');
  if (found == nullptr) return nullptr;
  found++;
  while (*found == ' ' || *found == '\t') found++;
  return found;
}

bool readJsonFloat(const char* json, const char* key, float& result) {
  const char* value = findJsonValue(json, key);
  if (value == nullptr) return false;
  char* endPointer = nullptr;
  result = strtof(value, &endPointer);
  return endPointer != value && isfinite(result);
}

bool readJsonUInt(const char* json, const char* key, uint32_t& result) {
  const char* value = findJsonValue(json, key);
  if (value == nullptr || *value == '-') return false;
  char* endPointer = nullptr;
  const unsigned long parsed = strtoul(value, &endPointer, 10);
  if (endPointer == value) return false;
  result = static_cast<uint32_t>(parsed);
  return true;
}

bool readJsonString(const char* json, const char* key, char* destination,
                    size_t destinationSize) {
  const char* value = findJsonValue(json, key);
  if (value == nullptr || *value != '\"' || destinationSize == 0) return false;
  value++;
  const char* end = strchr(value, '\"');
  if (end == nullptr) return false;
  const size_t length = static_cast<size_t>(end - value);
  if (length >= destinationSize) return false;
  memcpy(destination, value, length);
  destination[length] = '\0';
  return true;
}

void flushUdpPackets() {
  while (udpStarted) {
    const int packetSize = udp.parsePacket();
    if (packetSize <= 0) break;
    while (udp.available()) udp.read();
  }
}

bool processMotionPacket(const char* json) {
  uint32_t version = 0;
  uint32_t sequence = 0;
  uint32_t ttl = defaultCommandTtlMs;
  uint32_t pwmLimit = 0;
  float forward = 0.0f;
  float lateral = 0.0f;
  float turn = 0.0f;
  float headingError = 0.0f;
  char type[16];
  char mode[16];

  if (!readJsonUInt(json, "v", version) || version != 1 ||
      !readJsonString(json, "type", type, sizeof(type)) ||
      strcmp(type, "motion") != 0 ||
      !readJsonString(json, "mode", mode, sizeof(mode)) ||
      !readJsonUInt(json, "seq", sequence) ||
      !readJsonUInt(json, "ttl_ms", ttl) ||
      !readJsonUInt(json, "pwm_limit", pwmLimit) ||
      !readJsonFloat(json, "forward", forward) ||
      !readJsonFloat(json, "lateral", lateral) ||
      !readJsonFloat(json, "turn", turn) ||
      !readJsonFloat(json, "heading_error_deg", headingError)) {
    stopImmediately("INVALID PACKET");
    return false;
  }

  // 同一接続中の古いUDPパケットが後着した場合は無視する。
  if (haveSequence && static_cast<int32_t>(sequence - lastSequence) <= 0 &&
      millis() - lastPacketMs < maximumCommandTtlMs) {
    return false;
  }

  const bool nextLocalHeadingMode = strcmp(mode, "velocity_local") == 0;
  if (strcmp(mode, "stop") != 0 && strcmp(mode, "velocity") != 0 &&
      !nextLocalHeadingMode) {
    stopImmediately("UNKNOWN MODE");
    return false;
  }

  lastSequence = sequence;
  haveSequence = true;
  lastPacketMs = millis();
  commandTtlMs = constrain(ttl, minimumCommandTtlMs, maximumCommandTtlMs);
  requestedPwmLimit = static_cast<uint8_t>(
      constrain(pwmLimit, 0UL, static_cast<unsigned long>(absoluteMotorPwmCeiling)));

  if (strcmp(mode, "stop") == 0 || requestedPwmLimit == 0) {
    requestedForward = requestedLateral = requestedTurn = 0.0f;
    requestedHeadingErrorDeg = 0.0f;
    headingTargetYawDeg = yawDeg;
    localHeadingMode = false;
    stopImmediately("REMOTE STOP");
    return true;
  }

  const float previousTurn = requestedTurn;
  const bool wasCommandActive = commandActive;
  const bool wasLocalHeadingMode = localHeadingMode;
  requestedForward = clampFloat(forward, -1.0f, 1.0f);
  requestedLateral = clampFloat(lateral, -1.0f, 1.0f);
  requestedTurn = clampFloat(turn, -1.0f, 1.0f);
  requestedHeadingErrorDeg = clampFloat(headingError, -90.0f, 90.0f);
  localHeadingMode = nextLocalHeadingMode;
  if (localHeadingMode) {
    // カメラなし試験では開始時の方位を保持する。旋回指令中はPDが
    // 旋回を打ち消さないよう現在Yawへ追従し、旋回終了時に再ロックする。
    const bool turnJustStopped = fabs(previousTurn) >= 0.03f &&
                                 fabs(requestedTurn) < 0.03f;
    if (!wasCommandActive || !wasLocalHeadingMode || turnJustStopped ||
        fabs(requestedTurn) >= 0.03f) {
      headingTargetYawDeg = yawDeg;
    }
  } else {
    headingTargetYawDeg = yawDeg + requestedHeadingErrorDeg;
  }
  commandActive = true;
  setStateReason("COMMAND OK");
  return true;
}

void receiveUdpPackets() {
  if (!udpStarted) return;
  int packetSize = 0;
  while ((packetSize = udp.parsePacket()) > 0) {
    if (packetSize >= static_cast<int>(packetBufferSize)) {
      while (udp.available()) udp.read();
      stopImmediately("PACKET TOO LARGE");
      continue;
    }
    const int length = udp.read(packetBuffer, packetBufferSize - 1);
    if (length <= 0) continue;
    packetBuffer[length] = '\0';
    processMotionPacket(packetBuffer);
  }
}

float commandToPwm(float command, float minimumPwm, float maximumPwm) {
  const float magnitude = fabs(command);
  if (magnitude < 0.03f || maximumPwm <= 0.0f) return 0.0f;
  const float shaped = minimumPwm + (maximumPwm - minimumPwm) * magnitude;
  return command > 0.0f ? shaped : -shaped;
}

void writeMotorMix(float forwardPwm, float lateralPwm, float rotationPwm,
                   float outputLimit) {
  // M5HatBugC.cppのベクトル:
  // 前進 [+, -, +, -] / 右 [+,+,-,-] / 時計回り [+,+,+,+]
  float motor[4] = {
      forwardPwm + lateralPwm + rotationPwm,
      -forwardPwm + lateralPwm + rotationPwm,
      forwardPwm - lateralPwm + rotationPwm,
      -forwardPwm - lateralPwm + rotationPwm,
  };
  float peak = 0.0f;
  for (uint8_t i = 0; i < 4; i++) peak = max(peak, fabs(motor[i]));
  if (peak > outputLimit && peak > 0.0f) {
    const float scale = outputLimit / peak;
    for (uint8_t i = 0; i < 4; i++) motor[i] *= scale;
  }
  bugc.setAllMotorSpeed(clampMotorCommand(motor[0]),
                        clampMotorCommand(motor[1]),
                        clampMotorCommand(motor[2]),
                        clampMotorCommand(motor[3]));
}

void updateMotorControl() {
  const uint32_t nowMs = millis();
  if (nowMs - lastControlMs < controlIntervalMs) return;
  lastControlMs = nowMs;

  if (!armed) {
    stopImmediately("DISARMED");
    return;
  }
  if (WiFi.status() != WL_CONNECTED || !udpStarted) {
    stopImmediately("WIFI LOST");
    return;
  }
  if (!commandActive || lastPacketMs == 0 ||
      nowMs - lastPacketMs > commandTtlMs) {
    stopImmediately("UDP TIMEOUT");
    return;
  }

  const float packetLimit = min(static_cast<float>(requestedPwmLimit),
                                absoluteMotorPwmCeiling);
  const float forwardLimit = min(packetLimit, forwardPwmCeiling);
  const float backwardLimit = min(packetLimit, backwardPwmCeiling);
  const float lateralLimit = min(packetLimit, lateralPwmCeiling);
  const float rotationLimit = min(packetLimit, rotationPwmCeiling);

  float targetForwardPwm = 0.0f;
  if (requestedForward > 0.0f) {
    targetForwardPwm = commandToPwm(requestedForward, forwardMinimumPwm,
                                    forwardLimit);
  } else if (requestedForward < 0.0f) {
    targetForwardPwm = commandToPwm(requestedForward, backwardMinimumPwm,
                                    backwardLimit);
  }
  const float targetLateralPwm = commandToPwm(
      requestedLateral, lateralMinimumPwm, lateralLimit);

  const bool translationStopped = fabs(requestedForward) < 0.03f &&
                                  fabs(requestedLateral) < 0.03f;
  float directRotationPwm = 0.0f;
  if (translationStopped) {
    // PCのturnは反時計回りが正、BugCのモータ回転成分は時計回りが正。
    directRotationPwm = -commandToPwm(
        requestedTurn, inPlaceTurnMinimumPwm, rotationLimit);
  } else {
    directRotationPwm = -requestedTurn * rotationLimit;
  }

  float yawError = headingTargetYawDeg - yawDeg;
  if (fabs(yawError) < yawDeadbandDeg) yawError = 0.0f;
  const float desiredCorrection = kp * yawError - kd * filteredGyroZ;
  const float correctionTarget = clampFloat(
      correctionPolarity * desiredCorrection, -maxHeadingCorrection,
      maxHeadingCorrection);
  headingCorrection += clampFloat(correctionTarget - headingCorrection,
                                  -maxCorrectionStep, maxCorrectionStep);
  const float targetRotationPwm = directRotationPwm + headingCorrection;

  appliedForwardPwm = approach(appliedForwardPwm, targetForwardPwm,
                               translationStepPerCycle);
  appliedLateralPwm = approach(appliedLateralPwm, targetLateralPwm,
                               translationStepPerCycle);
  appliedRotationPwm = approach(appliedRotationPwm, targetRotationPwm,
                                rotationStepPerCycle);
  writeMotorMix(appliedForwardPwm, appliedLateralPwm, appliedRotationPwm,
                packetLimit);
}

void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!udpStarted) {
      udpStarted = udp.begin(udpListenPort) == 1;
      setStateReason(udpStarted ? "UDP READY" : "UDP BEGIN FAIL");
    }
    return;
  }

  if (udpStarted) {
    udp.stop();
    udpStarted = false;
  }
  if (millis() - lastWifiAttemptMs < wifiRetryIntervalMs) return;
  lastWifiAttemptMs = millis();
  WiFi.disconnect(false, false);
  WiFi.begin(wifiSsid, wifiPassword);
  setStateReason("WIFI CONNECTING");
}

void armAfterCalibration() {
  stopAllMotors();
  if (!calibrateGyroZ()) {
    armed = false;
    drawStatus();
    return;
  }
  flushUdpPackets();
  lastPacketMs = 0;
  haveSequence = false;
  commandActive = false;
  resetAppliedCommands();
  armed = true;
  setStateReason("ARMED WAIT UDP");
  bugc.setAllLedColor(0x002000, 0x002000);
  drawStatus();
}

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  M5.Display.setRotation(1);

  while (!bugc.begin(&Wire, BUGC_DEFAULT_I2C_ADDR, 0, 26, 400000U)) {
    M5.Display.fillScreen(BLACK);
    M5.Display.setCursor(4, 4);
    M5.Display.setTextColor(RED);
    M5.Display.setTextSize(2);
    M5.Display.println("BugC2 missing");
    delay(1000);
  }
  stopAllMotors();

  if (!M5.Imu.isEnabled()) {
    M5.Display.fillScreen(BLACK);
    M5.Display.setCursor(4, 4);
    M5.Display.setTextColor(RED);
    M5.Display.setTextSize(2);
    M5.Display.println("IMU missing");
    while (true) {
      stopAllMotors();
      delay(100);
    }
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(wifiSsid, wifiPassword);
  lastWifiAttemptMs = millis();
  disarm("PRESS A TO ARM");
  drawStatus();
  Serial.printf("UDP_CONTROLLER,ssid=%s,port=%u\n", wifiSsid, udpListenPort);
}

void loop() {
  M5.update();
  integrateGyro();
  maintainWiFi();
  receiveUdpPackets();

  if (M5.BtnA.wasPressed()) {
    if (armed) {
      disarm("BUTTON E-STOP");
      drawStatus();
    } else {
      armAfterCalibration();
    }
  }

  updateMotorControl();

  const uint32_t nowMs = millis();
  if (nowMs - lastDisplayMs >= displayIntervalMs) {
    lastDisplayMs = nowMs;
    drawStatus();
  }
  if (nowMs - lastSerialMs >= serialIntervalMs) {
    lastSerialMs = nowMs;
    Serial.printf(
        "STATE,armed=%d,wifi=%d,udp=%d,age=%lu,seq=%lu,f=%.3f,l=%.3f,"
        "t=%.3f,yaw=%.2f,target=%.2f,corr=%.2f,pwm=%u,reason=%s\n",
        armed, WiFi.status() == WL_CONNECTED, udpStarted,
        lastPacketMs == 0 ? 0UL : nowMs - lastPacketMs,
        static_cast<unsigned long>(lastSequence), requestedForward,
        requestedLateral, requestedTurn, yawDeg, headingTargetYawDeg,
        headingCorrection, requestedPwmLimit, stateReason);
  }
  delay(1);
}
