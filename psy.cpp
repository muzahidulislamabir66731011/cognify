

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C   // try 0x3D if this fails
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// -----------------------------
// PINS
// -----------------------------
const uint8_t TOUCH_PIN = 4;
const uint8_t TRIG_PIN  = 25;
const uint8_t ECHO_PIN  = 26;

// -----------------------------
// TIMING / TEST SETTINGS
// -----------------------------
const uint32_t SONAR_INTERVAL_MS      = 50;     // HC-SR04 sample rate, 20 Hz is safe
const uint32_t SONAR_TIMEOUT_US       = 3000;   // max echo wait time
const uint32_t RESPONSE_TIMEOUT_MS    = 2000;   // max time allowed to respond
const uint32_t MIN_INTER_TRIAL_MS     = 1200;   // random blank delay before X
const uint32_t MAX_INTER_TRIAL_MS     = 2500;
const uint32_t BREAK_MS               = 120000UL; // 2 minutes
const uint8_t  NUM_TRIALS             = 4;

// Touch sensitivity for ESP32 built-in touch
// Increase TOUCH_DELTA to make it LESS sensitive
// Decrease TOUCH_DELTA to make it MORE sensitive
const uint16_t TOUCH_DELTA = 25;

// -----------------------------
// STATE MACHINE
// -----------------------------
enum Phase : uint8_t {
  PHASE_IDLE,
  PHASE_WAITING,
  PHASE_STIMULUS,
  PHASE_TOO_SOON,
  PHASE_BREAK,
  PHASE_DONE
};

Phase phase = PHASE_IDLE;

// -----------------------------
// TRIAL VARIABLES
// -----------------------------
uint32_t waitUntil = 0;
uint32_t stimulusTime = 0;
uint32_t tooSoonUntil = 0;
uint32_t breakEnd = 0;
uint32_t lastBreakSecond = 0;

uint8_t  currentTrial = 0;
uint32_t validResponses = 0;
uint32_t misses = 0;
uint32_t falseStarts = 0;
int32_t  reactionTimes[NUM_TRIALS];

// -----------------------------
// TOUCH CALIBRATION
// -----------------------------
uint16_t touchBaseline = 0;
int touchThreshold = 0;

// -----------------------------
// SONAR / MOVEMENT VARIABLES
// -----------------------------
uint32_t lastSonarMs = 0;
bool havePrevSonar = false;
uint32_t prevSonarRaw = 0;

uint32_t sonarSamples = 0;
uint32_t sonarMin = 0;
uint32_t sonarMax = 0;
uint64_t sonarSum = 0;
uint64_t sonarSumSq = 0;

uint64_t totalMotion = 0;
uint32_t motionDeltas = 0;

// -----------------------------
// HELPERS
// -----------------------------
const char* phaseName(Phase p) {
  switch (p) {
    case PHASE_IDLE:     return "IDLE";
    case PHASE_WAITING:  return "WAITING";
    case PHASE_STIMULUS: return "STIMULUS";
    case PHASE_TOO_SOON: return "TOO_SOON";
    case PHASE_BREAK:    return "BREAK";
    case PHASE_DONE:     return "DONE";
  }
  return "UNKNOWN";
}

void logEvent(const char* name, int32_t value) {
  Serial.print(millis());
  Serial.print(F(",EVENT,"));
  Serial.print(name);
  Serial.print(',');
  Serial.print(value);
  Serial.println(',');
}

void logEventExtra(const char* name, int32_t value, int32_t extra) {
  Serial.print(millis());
  Serial.print(F(",EVENT,"));
  Serial.print(name);
  Serial.print(',');
  Serial.print(value);
  Serial.print(',');
  Serial.print(extra);
  Serial.println(',');
}

void logSonar(uint32_t raw) {
  Serial.print(millis());
  Serial.print(F(",SONAR,"));
  Serial.print(phaseName(phase));
  Serial.print(',');
  Serial.print(raw);
  Serial.println(',');
}

void resetSonarStats() {
  sonarSamples = 0;
  sonarSum = 0;
  sonarSumSq = 0;
  sonarMin = 0;
  sonarMax = 0;

  totalMotion = 0;
  motionDeltas = 0;

  havePrevSonar = false;
  prevSonarRaw = 0;
}

// -----------------------------
// TOUCH FUNCTIONS
// -----------------------------
uint16_t readTouchAverage(uint16_t samples) {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += touchRead(TOUCH_PIN);
    delay(3);
  }
  return (uint16_t)(sum / samples);
}

// ESP32 built-in touch: value usually goes DOWN when touched
bool touchPressed() {
  static uint8_t consecutive = 0;
  static bool latched = false;

  uint16_t v = touchRead(TOUCH_PIN);
  bool rawTouched = (v < touchThreshold);

  if (rawTouched) {
    if (consecutive < 3) consecutive++;
  } else {
    consecutive = 0;
    latched = false;
  }

  if (consecutive >= 2 && !latched) {
    latched = true;
    return true;
  }

  return false;
}

// -----------------------------
// SONAR FUNCTION
// -----------------------------
// Returns raw echo pulse width in microseconds.
// No distance conversion. No divide by 2.
uint32_t readSonarRawUs() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  uint32_t us = pulseIn(ECHO_PIN, HIGH, SONAR_TIMEOUT_US);

  // If no echo, use timeout as a stable "far/no echo" value
  if (us == 0) us = SONAR_TIMEOUT_US;

  return us;
}

void updateSonarStats(uint32_t raw) {
  if (sonarSamples == 0) {
    sonarMin = raw;
    sonarMax = raw;
  } else {
    if (raw < sonarMin) sonarMin = raw;
    if (raw > sonarMax) sonarMax = raw;
  }

  sonarSamples++;
  sonarSum += raw;
  sonarSumSq += (uint64_t)raw * raw;

  if (havePrevSonar) {
    uint32_t delta = (raw > prevSonarRaw) ? (raw - prevSonarRaw) : (prevSonarRaw - raw);
    totalMotion += delta;
    motionDeltas++;
  } else {
    havePrevSonar = true;
  }

  prevSonarRaw = raw;
}

void sampleSonar() {
  uint32_t now = millis();

  if (now - lastSonarMs < SONAR_INTERVAL_MS) return;

  lastSonarMs = now;

  uint32_t raw = readSonarRawUs();
  updateSonarStats(raw);
  logSonar(raw);
}

// -----------------------------
// DISPLAY FUNCTIONS
// -----------------------------
void screenClearNow() {
  display.clearDisplay();
  display.display();
}

void drawCenterText(const char* text, uint8_t textSize, int16_t yOffset = 0) {
  display.clearDisplay();
  display.setTextSize(textSize);
  display.setTextColor(SSD1306_WHITE);

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

  int16_t x = (SCREEN_WIDTH - w) / 2 - x1;
  int16_t y = (SCREEN_HEIGHT - h) / 2 - y1 + yOffset;

  display.setCursor(x, y);
  display.print(text);
  display.display();
}

void drawX() {
  drawCenterText("X", 6);
}

void drawIdle() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println(F("STEP 1"));
  display.println();
  display.println(F("Touch pad to start"));
  display.println(F("Then touch fast when X appears"));
  display.display();
}

void drawTooSoon() {
  drawCenterText("TOO SOON", 2);
}

void drawBreak(uint32_t secondsLeft) {
  char secText[8];
  snprintf(secText, sizeof(secText), "%lu", (unsigned long)secondsLeft);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  int16_t x1, y1;
  uint16_t w, h;

  display.getTextBounds("BREAK", 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2 - x1, 8 - y1);
  display.print("BREAK");

  display.setTextSize(3);
  display.getTextBounds(secText, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2 - x1, 36 - y1);
  display.print(secText);

  display.display();
}

void drawDone(float avgRT, float accuracy) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println(F("STEP 1 DONE"));

  display.setCursor(0, 16);
  display.print(F("Avg RT: "));
  display.print(avgRT, 0);
  display.println(F(" ms"));

  display.setCursor(0, 26);
  display.print(F("Accuracy: "));
  display.print(accuracy, 1);
  display.println(F("%"));

  display.setCursor(0, 36);
  display.print(F("False starts: "));
  display.print(falseStarts);

  display.display();
}

// -----------------------------
// SESSION FLOW
// -----------------------------
void startWaiting() {
  phase = PHASE_WAITING;
  screenClearNow();

  // Random delay to prevent user anticipation
  waitUntil = millis() + random(MIN_INTER_TRIAL_MS, MAX_INTER_TRIAL_MS + 1);
}

void startStimulus() {
  drawX();

  // Time stamp after display update, as close as practical to visual onset
  stimulusTime = millis();

  phase = PHASE_STIMULUS;
  logEvent("STIM_ON", currentTrial + 1);
}

void startBreak() {
  phase = PHASE_BREAK;
  breakEnd = millis() + BREAK_MS;
  lastBreakSecond = 0xFFFFFFFF;

  logEvent("BREAK_START", (int32_t)(BREAK_MS / 1000));
}

void printSummary() {
  uint32_t rtSum = 0;
  uint8_t rtCount = 0;

  for (uint8_t i = 0; i < NUM_TRIALS; i++) {
    if (reactionTimes[i] >= 0) {
      rtSum += reactionTimes[i];
      rtCount++;
    }
  }

  float avgRT = rtCount ? (float)rtSum / rtCount : 0.0f;

  float accuracy = 0.0f;
  if ((NUM_TRIALS + falseStarts) > 0) {
    accuracy = 100.0f * validResponses / (NUM_TRIALS + falseStarts);
  }

  float sonarMean = sonarSamples ? (float)sonarSum / sonarSamples : 0.0f;

  float sonarStd = 0.0f;
  if (sonarSamples > 1) {
    float variance = ((float)sonarSumSq / sonarSamples) - (sonarMean * sonarMean);
    if (variance < 0) variance = 0;
    sonarStd = sqrt(variance);
  }

  float avgMotionDelta = motionDeltas ? (float)totalMotion / motionDeltas : 0.0f;

  Serial.print(millis()); Serial.print(F(",SUMMARY,AVG_RT_MS,")); Serial.print(avgRT, 1); Serial.println(',');
  Serial.print(millis()); Serial.print(F(",SUMMARY,VALID_RESPONSES,")); Serial.print(validResponses); Serial.println(',');
  Serial.print(millis()); Serial.print(F(",SUMMARY,MISSES,")); Serial.print(misses); Serial.println(',');
  Serial.print(millis()); Serial.print(F(",SUMMARY,FALSE_STARTS,")); Serial.print(falseStarts); Serial.println(',');
  Serial.print(millis()); Serial.print(F(",SUMMARY,ACCURACY_PERCENT,")); Serial.print(accuracy, 1); Serial.println(',');

  Serial.print(millis()); Serial.print(F(",SUMMARY,SONAR_SAMPLES,")); Serial.print(sonarSamples); Serial.println(',');
  Serial.print(millis()); Serial.print(F(",SUMMARY,SONAR_MEAN_RAW_US,")); Serial.print(sonarMean, 1); Serial.println(',');
  Serial.print(millis()); Serial.print(F(",SUMMARY,SONAR_STD_RAW_US,")); Serial.print(sonarStd, 1); Serial.println(',');
  Serial.print(millis()); Serial.print(F(",SUMMARY,SONAR_MIN_RAW_US,")); Serial.print(sonarMin); Serial.println(',');
  Serial.print(millis()); Serial.print(F(",SUMMARY,SONAR_MAX_RAW_US,")); Serial.print(sonarMax); Serial.println(',');

  Serial.print(millis()); Serial.print(F(",SUMMARY,TOTAL_MOTION_RAW_US,")); Serial.print((unsigned long)totalMotion); Serial.println(',');
  Serial.print(millis()); Serial.print(F(",SUMMARY,AVG_MOTION_DELTA_RAW_US,")); Serial.print(avgMotionDelta, 1); Serial.println(',');

  drawDone(avgRT, accuracy);
}

void finishSession() {
  phase = PHASE_DONE;
  logEvent("BREAK_END", 0);
  printSummary();
}

// -----------------------------
// SETUP
// -----------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Use a floating analog pin for random seed
  randomSeed(analogRead(34));

  Wire.begin(21, 22);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("ERROR,SSD1306_NOT_FOUND,,"));
    while (1) {
      delay(100);
    }
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Calibrating touch..."));
  display.println(F("DO NOT touch pad"));
  display.display();

  delay(1000);

  touchBaseline = readTouchAverage(100);

  if (touchBaseline > TOUCH_DELTA + 10) {
    touchThreshold = touchBaseline - TOUCH_DELTA;
  } else {
    touchThreshold = touchBaseline / 2;
  }

  if (touchThreshold < 5) touchThreshold = 5;

  for (uint8_t i = 0; i < NUM_TRIALS; i++) {
    reactionTimes[i] = -1;
  }

  Serial.println(F("time_ms,type,name,value,extra"));

  logEvent("TOUCH_BASELINE", touchBaseline);
  logEvent("TOUCH_THRESHOLD", touchThreshold);

  drawIdle();

  lastSonarMs = millis();
}

// -----------------------------
// LOOP
// -----------------------------
void loop() {
  sampleSonar();

  uint32_t now = millis();

  switch (phase) {

    case PHASE_IDLE:
      if (touchPressed()) {
        // Start official measurement here
        resetSonarStats();

        logEvent("SESSION_START", 0);

        currentTrial = 0;
        validResponses = 0;
        misses = 0;
        falseStarts = 0;

        for (uint8_t i = 0; i < NUM_TRIALS; i++) {
          reactionTimes[i] = -1;
        }

        startWaiting();
      }
      break;

    case PHASE_WAITING:
      if (touchPressed()) {
        falseStarts++;
        logEvent("FALSE_START", currentTrial + 1);

        drawTooSoon();
        tooSoonUntil = now + 700;
        phase = PHASE_TOO_SOON;
      } else if (now >= waitUntil) {
        startStimulus();
      }
      break;

    case PHASE_STIMULUS:
      if (touchPressed()) {
        uint32_t touchTime = millis();
        int32_t rt = (int32_t)(touchTime - stimulusTime);

        reactionTimes[currentTrial] = rt;
        validResponses++;

        // Blank screen immediately.
        // Reaction time is already captured before blanking.
        screenClearNow();

        logEventExtra("VALID_TOUCH", currentTrial + 1, rt);

        currentTrial++;

        if (currentTrial >= NUM_TRIALS) {
          startBreak();
        } else {
          startWaiting();
        }
      } else if (now - stimulusTime >= RESPONSE_TIMEOUT_MS) {
        misses++;
        reactionTimes[currentTrial] = -1;

        screenClearNow();
        logEvent("MISS", currentTrial + 1);

        currentTrial++;

        if (currentTrial >= NUM_TRIALS) {
          startBreak();
        } else {
          startWaiting();
        }
      }
      break;

    case PHASE_TOO_SOON:
      if (now >= tooSoonUntil) {
        startWaiting();
      }
      break;

    case PHASE_BREAK: {
      uint32_t remaining = 0;
      if (breakEnd > now) {
        remaining = (breakEnd - now + 999) / 1000;
      }

      if (remaining != lastBreakSecond) {
        drawBreak(remaining);
        lastBreakSecond = remaining;
      }

      if (now >= breakEnd) {
        finishSession();
      }
      break;
    }

    case PHASE_DONE:
      // Step 1 finished.
      break;
  }
}
