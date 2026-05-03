#include "arduino_secrets.h"

// =============================================================================
// breathalyzer.ino
// CSE 328 IoT Term Project - Breath Alcohol Detection System
// Author: Akin Tatar (20220808075)
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "thingProperties.h"

// =============================================================================
//   CONFIG
// =============================================================================

#define PIN_MQ3_AO        34
#define PIN_BUZZER        25
#define PIN_LED_GREEN     26
#define PIN_LED_RED       27

#define I2C_ADDR_OLED     0x3C
#define I2C_ADDR_MAX30102 0x57

#define OLED_WIDTH        128
#define OLED_HEIGHT       64
#define OLED_RESET        -1

#define MAX30102_BUFFER_LENGTH    100
#define MAX30102_SHIFT_SIZE       25
#define MAX30102_FINGER_THRESHOLD 50000
#define MAX30102_HISTORY_SIZE     5
#define HR_VALID_MIN              40
#define HR_VALID_MAX              180
#define SPO2_VALID_MIN            80
#define SPO2_VALID_MAX            100

#define HR_NORMAL_MIN             40
#define HR_NORMAL_MAX             100

#define MQ3_WARMUP_SECONDS        30
#define MQ3_SAMPLE_COUNT          5   // was 10
#define MQ3_SAMPLE_DELAY_MS       5   // was 10ms — reduces per-read blocking from 100ms to 25ms
#define MQ3_BASELINE_DEFAULT      300

#define MQ3_BREATH_DETECT_FACTOR  1.05
#define MQ3_TAMPER_DETECT_FACTOR  8.0

#define IDLE_MQ3_CHECK_INTERVAL_MS 500
#define DISPLAY_REFRESH_INTERVAL_MS 500
#define BREATH_PEAK_UPDATE_INTERVAL_MS 150
#define POST_MEASUREMENT_COOLDOWN_MS 15000

#define BAC_LOW_FACTOR            1.37f   // baseline × 1.37 → IMPAIRED boundary (~0.08% BAC)
#define BAC_MEDIUM_FACTOR         1.61f   // baseline × 1.61 → DANGEROUS boundary (~0.15% BAC)

#define TIMEOUT_WAITING_BREATH_MS 30000
#define MEASUREMENT_BREATH_MS     5000
#define RESULT_DISPLAY_MS         10000
#define TAMPER_REJECT_MS          3000

#define BEEP_SHORT_MS             200
#define BEEP_LONG_MS              1000
#define ALARM_BEEP_MS             100
#define ALARM_GAP_MS              100
#define ALARM_REPEAT              3

#define DEBUG_SERIAL              true
#define SERIAL_BAUD               9600

// =============================================================================
//   ENUMS & STRUCTS
// =============================================================================

enum ResultClass {
  RESULT_SOBER,
  RESULT_IMPAIRED,
  RESULT_DANGEROUS
};

enum BACLevel {
  BAC_LOW,
  BAC_MEDIUM,
  BAC_HIGH
};

enum HRStatus {
  HR_NORMAL,
  HR_ANOMALOUS
};

enum State {
  STATE_IDLE,
  STATE_MEASURING_HR,
  STATE_WAITING_BREATH,
  STATE_MEASURING_BREATH,
  STATE_TAMPER_REJECT,
  STATE_CLASSIFY,
  STATE_RESULT,
  STATE_CLOUD_PUBLISH
};

struct VitalsReading {
  int  heartRate;
  int  spo2;
  bool valid;
  bool fingerPresent;
};

struct BreathReading {
  int  peakADC;
  int  baselineADC;
  bool valid;
};

struct MeasurementResult {
  int heartRate;
  int spo2;
  int bacPeak;
  int bacBaseline;
  unsigned long timestamp;
};

struct DisplayData {
  bool  hasBAC;
  float bacPercent;
  bool  hasHR;
  int   heartRate;
  int   spo2;
  bool  hasStatus;
  ResultClass status;
  const char* stepText;
};

// =============================================================================
//   GLOBAL VARIABLES
// =============================================================================

static MAX30105 particleSensor;
static Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

static int mq3Baseline = MQ3_BASELINE_DEFAULT;

static uint32_t irBuffer[MAX30102_BUFFER_LENGTH];
static uint32_t redBuffer[MAX30102_BUFFER_LENGTH];

// MAX30102 algoritmasinin doldurdugu degiskenler
// (cloud variable'lariyla cakismamasi icin maxXxx prefix'i)
static int32_t maxSpo2;
static int8_t  maxValidSpo2;
static int32_t maxHeartRate;
static int8_t  maxValidHeartRate;

static int  hrHistory[MAX30102_HISTORY_SIZE];
static int  spo2History[MAX30102_HISTORY_SIZE];
static int  historyIndex = 0;
static int  historyCount = 0;

static int  currentPeak = 0;

static State        currentState   = STATE_IDLE;
static unsigned long stateEnteredAt = 0;
static bool          forceRefresh   = true;
static unsigned long lastDisplayRefresh = 0;
static unsigned long lastIdleMQ3Check    = 0;
static unsigned long lastPeakUpdate      = 0;

static MeasurementResult lastResult = { 0, 0, 0, 0, 0 };
static ResultClass        lastResultClass = RESULT_SOBER;
static unsigned long      cooldownUntil = 0;

static const char* currentStep = "Place finger";

enum BuzzerPattern {
  BZ_NONE,
  BZ_SHORT,
  BZ_LONG,
  BZ_ALARM
};
static BuzzerPattern bzPattern = BZ_NONE;
static unsigned long bzPhaseStart = 0;
static int           bzStep = 0;
static bool          bzOn = false;

enum RedLEDMode {
  RED_OFF_MODE,
  RED_SOLID_MODE,
  RED_BLINK_MODE
};
static RedLEDMode    redMode = RED_OFF_MODE;
static unsigned long redBlinkInterval = 500;
static unsigned long redLastToggle = 0;
static bool          redLEDState = false;
static bool          greenLEDState = false;

// =============================================================================
//   FORWARD DECLARATIONS
// =============================================================================

void  mq3_init();
void  mq3_calibrateBaseline();
int   mq3_readRaw();
int   mq3_getBaseline();
bool  mq3_isBreathDetected();
bool  mq3_isAboveFactor(float factor);
BreathReading mq3_measurePeak(unsigned long durationMs);
void  mq3_peakReset();
void  mq3_peakUpdate();
int   mq3_peakValue();

bool  max30102_init();
void  max30102_fillInitialBuffer();
VitalsReading max30102_step();
void  max30102_resetHistory();

bool  display_init();
void  display_render(const DisplayData& data);
void  display_calibrating(int secondsLeft);
void  display_error(const char* message);

void  feedback_init();
void  feedback_update();
void  feedback_setLEDs(bool green, bool red);
void  feedback_blinkRed(unsigned long intervalMs);
void  feedback_beepShort();
void  feedback_beepLong();
void  feedback_alarm();
void  feedback_silence();
void  feedback_playResult(ResultClass result);

ResultClass classify(int bacPeak, int baseline, int heartRate, int spo2);
BACLevel    classifyBAC(int bacPeak, int baseline);
HRStatus    classifyHR(int heartRate);
const char* resultToString(ResultClass r);
float       adcToBACPercent(int adcPeak, int baseline);

void        states_init();
void        states_run();
const char* states_name(State s);

// =============================================================================
//   MQ-3
// =============================================================================

void mq3_init() {
  pinMode(PIN_MQ3_AO, INPUT);
}

int mq3_readRaw() {
  long sum = 0;
  for (int i = 0; i < MQ3_SAMPLE_COUNT; i++) {
    sum += analogRead(PIN_MQ3_AO);
    delay(MQ3_SAMPLE_DELAY_MS);
  }
  return (int)(sum / MQ3_SAMPLE_COUNT);
}

void mq3_calibrateBaseline() {
  const int BASELINE_SAMPLES = 50;
  long sum = 0;
  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    sum += mq3_readRaw();
  }
  mq3Baseline = (int)(sum / BASELINE_SAMPLES);

  if (DEBUG_SERIAL) {
    Serial.print("[MQ-3] Baseline calibrated: ");
    Serial.println(mq3Baseline);
  }
}

int mq3_getBaseline() {
  return mq3Baseline;
}

bool mq3_isBreathDetected() {
  int raw = mq3_readRaw();
  float threshold = mq3Baseline * MQ3_BREATH_DETECT_FACTOR;
  return raw > threshold;
}

bool mq3_isAboveFactor(float factor) {
  int raw = mq3_readRaw();
  float threshold = mq3Baseline * factor;
  return raw > threshold;
}

BreathReading mq3_measurePeak(unsigned long durationMs) {
  unsigned long startTime = millis();
  int peak = 0;

  while (millis() - startTime < durationMs) {
    int raw = mq3_readRaw();
    if (raw > peak) peak = raw;
  }

  BreathReading r = { peak, mq3Baseline, true };
  return r;
}

void mq3_peakReset() {
  currentPeak = 0;
}

void mq3_peakUpdate() {
  int raw = mq3_readRaw();
  if (raw > currentPeak) currentPeak = raw;
}

int mq3_peakValue() {
  return currentPeak;
}

// =============================================================================
//   MAX30102
// =============================================================================

bool max30102_init() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    if (DEBUG_SERIAL) Serial.println("[MAX30102] Sensor not found");
    return false;
  }

  byte ledBrightness = 40;
  byte sampleAverage = 4;
  byte ledMode       = 2;
  byte sampleRate    = 100;
  int  pulseWidth    = 411;
  int  adcRange      = 4096;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode,
                       sampleRate, pulseWidth, adcRange);

  if (DEBUG_SERIAL) Serial.println("[MAX30102] Initialized");
  return true;
}

void max30102_fillInitialBuffer() {
  for (byte i = 0; i < MAX30102_BUFFER_LENGTH; i++) {
    while (particleSensor.available() == false) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }
}

VitalsReading max30102_step() {
  VitalsReading result = { 0, 0, false, false };

  for (byte i = MAX30102_SHIFT_SIZE; i < MAX30102_BUFFER_LENGTH; i++) {
    redBuffer[i - MAX30102_SHIFT_SIZE] = redBuffer[i];
    irBuffer[i - MAX30102_SHIFT_SIZE]  = irBuffer[i];
  }

  for (byte i = MAX30102_BUFFER_LENGTH - MAX30102_SHIFT_SIZE;
       i < MAX30102_BUFFER_LENGTH; i++) {
    while (particleSensor.available() == false) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, MAX30102_BUFFER_LENGTH, redBuffer,
    &maxSpo2, &maxValidSpo2, &maxHeartRate, &maxValidHeartRate
  );

  uint32_t avgIR = 0;
  for (byte i = 0; i < MAX30102_BUFFER_LENGTH; i++) avgIR += irBuffer[i];
  avgIR /= MAX30102_BUFFER_LENGTH;

  result.fingerPresent = (avgIR >= MAX30102_FINGER_THRESHOLD);

  if (!result.fingerPresent) {
    max30102_resetHistory();
    return result;
  }

  bool reasonable = maxValidHeartRate && maxValidSpo2 &&
                    maxHeartRate >= HR_VALID_MIN  && maxHeartRate <= HR_VALID_MAX &&
                    maxSpo2      >= SPO2_VALID_MIN && maxSpo2     <= SPO2_VALID_MAX;

  if (!reasonable) return result;

  hrHistory[historyIndex]   = maxHeartRate / 2;
  spo2History[historyIndex] = maxSpo2;
  historyIndex = (historyIndex + 1) % MAX30102_HISTORY_SIZE;
  if (historyCount < MAX30102_HISTORY_SIZE) historyCount++;

  int hrSum = 0, spo2Sum = 0;
  for (int i = 0; i < historyCount; i++) {
    hrSum   += hrHistory[i];
    spo2Sum += spo2History[i];
  }
  result.heartRate = hrSum   / historyCount;
  result.spo2      = spo2Sum / historyCount;
  result.valid     = (historyCount >= MAX30102_HISTORY_SIZE);

  return result;
}

void max30102_resetHistory() {
  historyIndex = 0;
  historyCount = 0;
}

// =============================================================================
//   CLASSIFIER
// =============================================================================

BACLevel classifyBAC(int bacPeak, int baseline) {
  int lowMax    = (int)(baseline * BAC_LOW_FACTOR);
  int mediumMax = (int)(baseline * BAC_MEDIUM_FACTOR);
  if (bacPeak <= lowMax)    return BAC_LOW;
  if (bacPeak <= mediumMax) return BAC_MEDIUM;
  return BAC_HIGH;
}

HRStatus classifyHR(int heartRate) {
  if (heartRate >= HR_NORMAL_MIN && heartRate <= HR_NORMAL_MAX) {
    return HR_NORMAL;
  }
  return HR_ANOMALOUS;
}

ResultClass classify(int bacPeak, int baseline, int heartRate, int spo2) {
  BACLevel bac = classifyBAC(bacPeak, baseline);
  HRStatus hr  = classifyHR(heartRate);

  if (bac == BAC_HIGH)   return RESULT_DANGEROUS;

  if (bac == BAC_MEDIUM && spo2 > 0 && spo2 < 94) return RESULT_DANGEROUS;

  if (bac == BAC_MEDIUM) return RESULT_IMPAIRED;

  if (hr == HR_NORMAL)   return RESULT_SOBER;
  return RESULT_IMPAIRED;
}

const char* resultToString(ResultClass r) {
  switch (r) {
    case RESULT_SOBER:     return "SOBER";
    case RESULT_IMPAIRED:  return "IMPAIRED";
    case RESULT_DANGEROUS: return "DANGEROUS";
  }
  return "UNKNOWN";
}

float adcToBACPercent(int adcPeak, int baseline) {
  if (adcPeak <= baseline) return 0.0f;

  int lowMax    = (int)(baseline * BAC_LOW_FACTOR);
  int mediumMax = (int)(baseline * BAC_MEDIUM_FACTOR);

  if (adcPeak <= lowMax) {
    float ratio = (float)(adcPeak - baseline) / (float)(lowMax - baseline);
    return 0.08f * ratio;
  }
  if (adcPeak <= mediumMax) {
    float ratio = (float)(adcPeak - lowMax) / (float)(mediumMax - lowMax);
    return 0.08f + 0.07f * ratio;
  }
  int adcCap = 4095;
  if (adcPeak >= adcCap) return 0.30f;
  float ratio = (float)(adcPeak - mediumMax) / (float)(adcCap - mediumMax);
  return 0.15f + 0.15f * ratio;
}

// =============================================================================
//   DISPLAY
// =============================================================================

static const int LABEL_X    = 0;
static const int VALUE_X    = 50;
static const int ROW_Y[4]   = { 4, 20, 36, 52 };

static void drawCentered(const char* text, int y, uint8_t size) {
  oled.setTextSize(size);
  int charWidth = 6 * size;
  int textWidth = (int)strlen(text) * charWidth;
  int x = (OLED_WIDTH - textWidth) / 2;
  if (x < 0) x = 0;
  oled.setCursor(x, y);
  oled.print(text);
}

static void drawLabelValue(int row, const char* label, const char* value) {
  oled.setTextSize(1);
  oled.setCursor(LABEL_X, ROW_Y[row]);
  oled.print(label);
  oled.setCursor(VALUE_X, ROW_Y[row]);
  oled.print(value);
}

bool display_init() {
  if (!oled.begin(SSD1306_SWITCHCAPVCC, I2C_ADDR_OLED)) {
    if (DEBUG_SERIAL) Serial.println("[OLED] Init failed");
    return false;
  }
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.display();
  if (DEBUG_SERIAL) Serial.println("[OLED] Initialized");
  return true;
}

void display_render(const DisplayData& data) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  char buf[24];

  if (data.hasBAC) {
    snprintf(buf, sizeof(buf), "%.2f%% (%d)", data.bacPercent, lastResult.bacPeak);
  } else {
    snprintf(buf, sizeof(buf), "---");
  }
  drawLabelValue(0, "BAC:", buf);

  if (data.hasHR) {
    snprintf(buf, sizeof(buf), "%dbpm %d%%", data.heartRate, data.spo2);
  } else {
    snprintf(buf, sizeof(buf), "---");
  }
  drawLabelValue(1, "HR:", buf);

  const char* statusStr;
  if (data.hasStatus) {
    statusStr = resultToString(data.status);
  } else {
    statusStr = "---";
  }
  drawLabelValue(2, "Status:", statusStr);

  drawLabelValue(3, "Step:", data.stepText ? data.stepText : "");

  oled.display();
}

void display_calibrating(int secondsLeft) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  drawCentered("Calibrating", 8, 1);
  drawCentered("MQ-3", 20, 1);

  char buf[8];
  snprintf(buf, sizeof(buf), "%d", secondsLeft);
  drawCentered(buf, 36, 3);

  oled.display();
}

void display_error(const char* message) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  drawCentered("ERROR", 4, 2);
  drawCentered(message, 36, 1);
  drawCentered("Restart device", 52, 1);

  oled.display();
}

// =============================================================================
//   FEEDBACK
// =============================================================================

static void buzzerOn() {
  
  bzOn = true;
}

static void buzzerOff() {
  
  bzOn = false;
}

static void buzzerEngineUpdate() {
  if (bzPattern == BZ_NONE) return;

  unsigned long now = millis();
  unsigned long elapsedTime = now - bzPhaseStart;

  switch (bzPattern) {
    case BZ_SHORT:
      if (elapsedTime >= BEEP_SHORT_MS) {
        buzzerOff();
        bzPattern = BZ_NONE;
      }
      break;

    case BZ_LONG:
      if (elapsedTime >= BEEP_LONG_MS) {
        buzzerOff();
        bzPattern = BZ_NONE;
      }
      break;

    case BZ_ALARM:
      if (bzStep >= ALARM_REPEAT * 2) {
        buzzerOff();
        bzPattern = BZ_NONE;
        bzStep = 0;
        return;
      }

      if (bzStep % 2 == 0) {
        if (elapsedTime >= ALARM_BEEP_MS) {
          buzzerOff();
          bzStep++;
          bzPhaseStart = now;
        }
      } else {
        if (elapsedTime >= ALARM_GAP_MS) {
          if (bzStep + 1 < ALARM_REPEAT * 2) {
            buzzerOn();
          }
          bzStep++;
          bzPhaseStart = now;
        }
      }
      break;

    case BZ_NONE:
      break;
  }
}

static void greenLED(bool on) {
  greenLEDState = on;
}

static void redLEDApply(bool on) {
  redLEDState = on;
}

static void ledEngineUpdate() {
  if (redMode != RED_BLINK_MODE) return;

  unsigned long now = millis();
  if (now - redLastToggle >= redBlinkInterval) {
    redLEDApply(!redLEDState);
    redLastToggle = now;
  }
}

void feedback_init() {
  pinMode(PIN_BUZZER,    OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED,   OUTPUT);
  buzzerOff();
  greenLED(false);
  redLEDApply(false);
  redMode = RED_OFF_MODE;
  bzPattern = BZ_NONE;

  feedback_update();
}

void feedback_update() {
  buzzerEngineUpdate();
  ledEngineUpdate();

  digitalWrite(PIN_BUZZER, bzOn || manual_buzzer);
  digitalWrite(PIN_LED_RED, redLEDState || manual_red);
  digitalWrite(PIN_LED_GREEN, greenLEDState || manual_green);
}

void feedback_setLEDs(bool green, bool red) {
  greenLED(green);
  redMode = red ? RED_SOLID_MODE : RED_OFF_MODE;
  redLEDApply(red);
}

void feedback_blinkRed(unsigned long intervalMs) {
  redMode = RED_BLINK_MODE;
  redBlinkInterval = intervalMs;
  redLastToggle = millis();
  redLEDApply(true);
}

void feedback_beepShort() {
  bzPattern = BZ_SHORT;
  bzPhaseStart = millis();
  buzzerOn();
}

void feedback_beepLong() {
  bzPattern = BZ_LONG;
  bzPhaseStart = millis();
  buzzerOn();
}

void feedback_alarm() {
  bzPattern = BZ_ALARM;
  bzPhaseStart = millis();
  bzStep = 0;
  buzzerOn();
}

void feedback_silence() {
  bzPattern = BZ_NONE;
  bzStep = 0;
  buzzerOff();
}

void feedback_playResult(ResultClass result) {
  switch (result) {
    case RESULT_SOBER:
      feedback_setLEDs(true, false);
      feedback_silence();
      break;

    case RESULT_IMPAIRED:
      feedback_setLEDs(false, true);
      feedback_beepShort();
      break;

    case RESULT_DANGEROUS:
      greenLED(false);
      feedback_blinkRed(300);
      feedback_alarm();
      break;
  }
}

// =============================================================================
//   STATE MACHINE
// =============================================================================

static void setState(State next) {
  if (DEBUG_SERIAL) {
    Serial.print("[STATE] ");
    Serial.print(states_name(currentState));
    Serial.print(" -> ");
    Serial.println(states_name(next));
  }
  currentState   = next;
  stateEnteredAt = millis();
  forceRefresh   = true;
}

static unsigned long elapsedInState() {
  return millis() - stateEnteredAt;
}

static void refreshDisplay() {
  unsigned long now = millis();
  if (!forceRefresh && (now - lastDisplayRefresh < DISPLAY_REFRESH_INTERVAL_MS)) {
    return;
  }

  DisplayData d = { false, 0.0f, false, 0, 0, false, RESULT_SOBER, currentStep };

  if (lastResult.bacPeak > 0) {
    d.hasBAC = true;
    d.bacPercent = adcToBACPercent(lastResult.bacPeak, lastResult.bacBaseline);
  }

  if (lastResult.heartRate > 0) {
    d.hasHR = true;
    d.heartRate = lastResult.heartRate;
    d.spo2 = lastResult.spo2;
  }

  if (currentState == STATE_RESULT || currentState == STATE_CLOUD_PUBLISH) {
    d.hasStatus = true;
    d.status = lastResultClass;
  }

  display_render(d);
  lastDisplayRefresh = now;
  forceRefresh = false;
}

static void resetMeasurement() {
  lastResult = { 0, 0, 0, 0, 0 };
  lastResultClass = RESULT_SOBER;
  max30102_resetHistory();
  mq3_peakReset();
}

static void handleIdle() {
  currentStep = "Place finger";
  refreshDisplay();

  VitalsReading v = max30102_step();
  if (v.fingerPresent) {
    setState(STATE_MEASURING_HR);
    return;
  }

  unsigned long now = millis();
  if (now < cooldownUntil) {
    return;
  }
  if (now - lastIdleMQ3Check >= IDLE_MQ3_CHECK_INTERVAL_MS) {
    lastIdleMQ3Check = now;
    
    // Anlık değeri oku
    int raw = mq3_readRaw();
    
    if (DEBUG_SERIAL) {
      Serial.print("[IDLE] BAC raw=");
      Serial.print(raw);
      Serial.print(" baseline=");
      Serial.println(mq3Baseline);
    }

    lastResult.bacPeak     = raw;
    lastResult.bacBaseline = mq3Baseline;

    // (Tamper) Kontrolü (8x sınırı aşıldıysa alarm ver)
    if (raw > mq3Baseline * MQ3_TAMPER_DETECT_FACTOR) {
      setState(STATE_TAMPER_REJECT);
      return;
    }

    // Parmak yokken BAC yüksekse kullanıcıyı uyar.
    BACLevel idleBAC = classifyBAC(raw, mq3Baseline);
    if (idleBAC == BAC_MEDIUM || idleBAC == BAC_HIGH) {
      feedback_setLEDs(false, true);
      if (bzPattern == BZ_NONE) feedback_beepShort();
    } else {
      feedback_setLEDs(false, false);
      feedback_silence();
    }

    // Dinamik baseline: sadece hava temizken (raw <= baseline * 1.02) güncelle.
    // Eğer ortamda alkol varsa baseline yukarı sürüklenmesin.
    if (raw <= (int)(mq3Baseline * 1.02f)) {
      mq3Baseline = (int)((mq3Baseline * 0.95) + (raw * 0.05));
    }
    
    // if (DEBUG_SERIAL) { Serial.print("Baseline: "); Serial.println(mq3Baseline); }
  }
}

static void handleMeasuringHR() {
  currentStep = "Reading pulse...";

  VitalsReading v = max30102_step();

  if (v.heartRate > 0) {
    lastResult.heartRate = v.heartRate;
    lastResult.spo2      = v.spo2;
  }

  refreshDisplay();

  if (!v.fingerPresent) {
    resetMeasurement();
    setState(STATE_IDLE);
    return;
  }

  if (v.valid) {
    if (DEBUG_SERIAL) {
      Serial.print("[HR] Captured: ");
      Serial.print(v.heartRate);
      Serial.print(" bpm, SpO2 ");
      Serial.println(v.spo2);
    }
    setState(STATE_WAITING_BREATH);
    return;
  }
}

static void handleWaitingBreath() {
  unsigned long elapsedMs = elapsedInState();
  if (elapsedMs >= TIMEOUT_WAITING_BREATH_MS) {
    if (DEBUG_SERIAL) Serial.println("[WAITING_BREATH] Timeout -> classifying as SOBER (no breath detected)");
    // No breath above threshold in 30s means BAC is at baseline = SOBER.
    // Use the live reading captured during the wait as the peak, then classify normally.
    lastResult.bacPeak     = mq3_readRaw();
    lastResult.bacBaseline = mq3_getBaseline();
    lastResult.timestamp   = millis();
    setState(STATE_CLASSIFY);
    return;
  }

  static unsigned long lastLiveUpdate = 0;
  unsigned long now = millis();
  
  if (now - lastLiveUpdate > 500) {
    int liveRaw = mq3_readRaw();
    
    // OLED ekran state'ini güncelle
    lastResult.bacPeak = liveRaw;
    lastResult.bacBaseline = mq3_getBaseline();
    
    // Cloud Dashboard variable'larını doğrudan güncelle
    bac_raw = liveRaw;
    bac_percent = adcToBACPercent(liveRaw, lastResult.bacBaseline);

    if (DEBUG_SERIAL) {
      Serial.print("[WAITING] BAC raw=");
      Serial.print(liveRaw);
      Serial.print(" baseline=");
      Serial.print(lastResult.bacBaseline);
      Serial.print(" pct=");
      Serial.println(bac_percent, 3);
    }

    lastLiveUpdate = now;
  }
  
// -----------------------------
  
  unsigned long secsLeft = (TIMEOUT_WAITING_BREATH_MS - elapsedMs) / 1000;
  static char stepBuf[24];
  snprintf(stepBuf, sizeof(stepBuf), "Blow now (%lus)", secsLeft);
  currentStep = stepBuf;

  refreshDisplay();

  if (mq3_isBreathDetected()) {
    if (DEBUG_SERIAL) Serial.println("[WAITING_BREATH] Breath detected");
    mq3_peakReset();
    lastPeakUpdate = millis();
    setState(STATE_MEASURING_BREATH);
    return;
  }
}

static void handleMeasuringBreath() {
  currentStep = "Measuring...";

  unsigned long now = millis();
  if (now - lastPeakUpdate >= BREATH_PEAK_UPDATE_INTERVAL_MS) {
    mq3_peakUpdate();
    lastPeakUpdate = now;

    lastResult.bacPeak     = mq3_peakValue();
    lastResult.bacBaseline = mq3_getBaseline();
  }

  refreshDisplay();

  if (elapsedInState() >= MEASUREMENT_BREATH_MS) {
    lastResult.bacPeak     = mq3_peakValue();
    lastResult.bacBaseline = mq3_getBaseline();
    lastResult.timestamp   = millis();
    if (DEBUG_SERIAL) {
      Serial.print("[BREATH] Peak ADC: ");
      Serial.println(lastResult.bacPeak);
    }
    setState(STATE_CLASSIFY);
    return;
  }
}

static void handleTamperReject() {
  if (forceRefresh) {
    feedback_alarm();
    feedback_setLEDs(false, true);
  }

  currentStep = "TAMPERING!";
  refreshDisplay();

  if (elapsedInState() >= TAMPER_REJECT_MS) {
    feedback_setLEDs(false, false);
    feedback_silence();
    resetMeasurement();
    cooldownUntil = millis() + POST_MEASUREMENT_COOLDOWN_MS;
    setState(STATE_IDLE);
    return;
  }
}

static void handleClassify() {
  lastResultClass = classify(lastResult.bacPeak, lastResult.bacBaseline, lastResult.heartRate, lastResult.spo2);

  if (DEBUG_SERIAL) {
    Serial.print("[CLASSIFY] BAC=");
    Serial.print(lastResult.bacPeak);
    Serial.print(" HR=");
    Serial.print(lastResult.heartRate);
    Serial.print(" SpO2=");
    Serial.print(lastResult.spo2);
    Serial.print(" -> ");
    Serial.println(resultToString(lastResultClass));
  }

  setState(STATE_RESULT);
}

static void handleResult() {
  if (forceRefresh) {
    feedback_playResult(lastResultClass);
  }

  currentStep = "Done";
  refreshDisplay();

  if (elapsedInState() >= RESULT_DISPLAY_MS) {
    setState(STATE_CLOUD_PUBLISH);
    return;
  }
}

static void handleCloudPublish() {
  // Cloud variable'larina deger ata
  bac_percent = adcToBACPercent(lastResult.bacPeak, lastResult.bacBaseline);
  bac_raw     = lastResult.bacPeak;
  heart_rate  = lastResult.heartRate;
  spo2        = lastResult.spo2;
  status      = String(resultToString(lastResultClass));

  unsigned long totalSec = lastResult.timestamp / 1000;
  unsigned long mins = totalSec / 60;
  unsigned long secs = totalSec % 60;
  char timeBuf[24];
  snprintf(timeBuf, sizeof(timeBuf), "%lum %lus uptime", mins, secs);
  last_test_time = String(timeBuf);

  if (DEBUG_SERIAL) {
    Serial.print("[CLOUD] Published: BAC=");
    Serial.print(bac_percent, 2);
    Serial.print("% raw=");
    Serial.print(bac_raw);
    Serial.print(" HR=");
    Serial.print(heart_rate);
    Serial.print(" SpO2=");
    Serial.print(spo2);
    Serial.print(" status=");
    Serial.println(status);
  }

  feedback_setLEDs(false, false);
  feedback_silence();
  resetMeasurement();
  cooldownUntil = millis() + POST_MEASUREMENT_COOLDOWN_MS;
  setState(STATE_IDLE);
}

void states_init() {
  currentState   = STATE_IDLE;
  stateEnteredAt = millis();
  forceRefresh   = true;
  lastDisplayRefresh = 0;
  lastIdleMQ3Check   = 0;
  cooldownUntil      = millis() + POST_MEASUREMENT_COOLDOWN_MS;
  resetMeasurement();
}

void states_run() {
  switch (currentState) {
    case STATE_IDLE:              handleIdle();             break;
    case STATE_MEASURING_HR:      handleMeasuringHR();      break;
    case STATE_WAITING_BREATH:    handleWaitingBreath();    break;
    case STATE_MEASURING_BREATH:  handleMeasuringBreath();  break;
    case STATE_TAMPER_REJECT:     handleTamperReject();     break;
    case STATE_CLASSIFY:          handleClassify();         break;
    case STATE_RESULT:            handleResult();           break;
    case STATE_CLOUD_PUBLISH:     handleCloudPublish();     break;
  }
}

const char* states_name(State s) {
  switch (s) {
    case STATE_IDLE:             return "IDLE";
    case STATE_MEASURING_HR:     return "MEASURING_HR";
    case STATE_WAITING_BREATH:   return "WAITING_BREATH";
    case STATE_MEASURING_BREATH: return "MEASURING_BREATH";
    case STATE_TAMPER_REJECT:    return "TAMPER_REJECT";
    case STATE_CLASSIFY:         return "CLASSIFY";
    case STATE_RESULT:           return "RESULT";
    case STATE_CLOUD_PUBLISH:    return "CLOUD_PUBLISH";
  }
  return "UNKNOWN";
}

// =============================================================================
//   SETUP & LOOP
// =============================================================================

void setup() {
  if (DEBUG_SERIAL) {
    Serial.begin(SERIAL_BAUD);
    delay(500);
    Serial.println("\n=== Breathalyzer booting ===");
  }

  Wire.begin();

  if (!display_init()) {
    Serial.println("OLED init failed");
    while (1) delay(1000);
  }

  feedback_init();

  if (!max30102_init()) {
    Serial.println("MAX30102 init failed");
    display_error("Pulse sensor missing");
    while (1) delay(1000);
  }

  mq3_init();
  Serial.print("MQ-3 warming up ");
  Serial.print(MQ3_WARMUP_SECONDS);
  Serial.println("s...");
  for (int i = MQ3_WARMUP_SECONDS; i > 0; i--) {
    display_calibrating(i);
    delay(1000);
  }
  mq3_calibrateBaseline();
  Serial.print("MQ-3 baseline = ");
  Serial.println(mq3_getBaseline());

  max30102_fillInitialBuffer();

  // Cloud baglantisi
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  states_init();

  Serial.println("=== Ready ===");
}

void loop() {
  ArduinoCloud.update();
  feedback_update();
  states_run();
}

/*
  Since ManualBuzzer is READ_WRITE variable, onManualBuzzerChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onManualBuzzerChange() {
  Serial.print("[CLOUD] Manual Buzzer: ");
  Serial.println(manual_buzzer ? "ON" : "OFF");
}

void onManualRedChange() {
  Serial.print("[CLOUD] Manual Red LED: ");
  Serial.println(manual_red ? "ON" : "OFF");
}

void onManualGreenChange() {
  Serial.print("[CLOUD] Manual Green LED: ");
  Serial.println(manual_green ? "ON" : "OFF");
}