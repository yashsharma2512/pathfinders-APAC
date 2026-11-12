/*
  ---------------- ROBOT WALL FOLLOWER ----------------
  Features:
    - MPU6050 for orientation
    - Ultrasonic sensors for wall and obstacle detection
    - L298N Motor Driver for movement
    - NeoPixel LEDs for status indication
    - PID-based turning
    - Simple wall alignment while moving straight
    - Nicla Vision UART receiver (NEW)
*/

// ------------------- Libraries -------------------
#include <Arduino.h>
#include <Wire.h>
#include <Seeed_Arduino_SSCMA.h>
#include <MPU6050_light.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_AS7341.h>
#include <Servo.h>
#include <stdio.h>

Servo myservo;

// ---------------- AI ----------------

SSCMA AI;
volatile bool dataincoming = false;
volatile int _target = 0;
volatile int _confidence = 0;
volatile bool _startAI = false;

int sen_SCL = 17;
int sen_SDA = 16;

// ---------------- NICLA VISION UART (NEW) ----------------

#define NICLA_RX 7
#define NICLA_TX 8

String niclaBuffer = "";

int niclaTarget = -1;
int niclaConfidence = 0;

unsigned long lastNiclaMessage = 0;
bool niclaReportedCurrent = false;
int lastNiclaReportedTarget = -1;

// -------------------- AS7341 --------------------

Adafruit_AS7341 as7341;

// ------------------- NeoPixel -------------------

#define LED_PIN 23
#define LED_COUNT 2

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ------------------- Ultrasonic Pins -------------------

#define TRIG_LEFT 5
#define ECHO_LEFT 4
#define TRIG_FRONT 13
#define ECHO_FRONT 12

// ------------------- Motor Driver Pins -------------------

#define RMF 20
#define RMR 21
#define LMF 26
#define LMR 27

// ------------------- MPU6050 -------------------

#define LEFT -1
#define RIGHT 1
#define UTURN 2

#define kp_turn 9
#define kd_turn 3.2
#define MAXSPEED_turning 175
#define BASESPEED_move 120

MPU6050 mpu(Wire);

int pos = 0;

// ------------------- Globals -------------------

#define Kp_run 17
#define Kp_gyro 8

int distLeftLAST = 0;
int distFrontLAST = 0;
int wallOffset = 9;
int wallDetect = 18;


// =============================================================
// =============== NICLA VISION RECEIVER (NEW) =================
// =============================================================

void receiveNiclaData() {

  if (lastNiclaMessage != 0 && millis() - lastNiclaMessage > 1000) {
    niclaReportedCurrent = false;
    lastNiclaReportedTarget = -1;
  }

  while (Serial1.available()) {

    char c = Serial1.read();

    if (c == '\n') {

      int victim;
      unsigned long stableMs;
      int x;
      int y;

      if (sscanf(niclaBuffer.c_str(),
                 "%d,%lu,%d,%d",
                 &victim,
                 &stableMs,
                 &x,
                 &y) == 4 &&
          victim >= 0 && victim <= 2 &&
          x >= 0 && x < 160 && y >= 0 && y < 160) {

        // Working Nicla output: 2=Harmed, 1=Stable, 0=Unharmed.
        // Existing robot cases: 0=Harmed, 1=Survivor, 2=Unharmed.
        niclaTarget = 2 - victim;
        niclaConfidence = stableMs >= 750 ? 100 : 0;

        lastNiclaMessage = millis();

        Serial.print("[NICLA] Target: ");
        Serial.print(niclaTarget);

        Serial.print(" | Confidence: ");
        Serial.println(niclaConfidence);

        if (niclaConfidence > 90 && !dataincoming &&
            (!niclaReportedCurrent || niclaTarget != lastNiclaReportedTarget)) {
          _target = niclaTarget;
          _confidence = niclaConfidence;
          dataincoming = true;
          niclaReportedCurrent = true;
          lastNiclaReportedTarget = niclaTarget;
        }
      }

      niclaBuffer = "";
    }

    else if (c != '\r') {

      if (niclaBuffer.length() < 50) {
        niclaBuffer += c;
      }
      else {
        niclaBuffer = "";
      }
    }
  }
}


// =============================================================
// =============== UTILITY FUNCTIONS ===========================
// =============================================================

// ------------------------ AS7341 ------------------------

void initAS7341Sensor() {

  if (!as7341.begin(AS7341_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("Could not find AS7341");
    while (1) {
      delay(10);
    }
  }

  as7341.setATIME(10);
  as7341.setASTEP(30);
  as7341.setGain(AS7341_GAIN_512X);
  as7341.setLEDCurrent(60);
  as7341.enableLED(false);

  delay(100);

  as7341.enableLED(true);
}

int readColor() {

  uint16_t readings[12];

  if (!as7341.readAllChannels()) {
    Serial.println("Error reading all channels!");
    return -1;
  }

  uint16_t red = as7341.getChannel(AS7341_CHANNEL_630nm_F7);
  uint16_t green = as7341.getChannel(AS7341_CHANNEL_555nm_F5);
  uint16_t blue = as7341.getChannel(AS7341_CHANNEL_480nm_F3);

  int RG = (red + green) / 2;

  if (red < 180 && RG < 120) {
    if (blue > 60)
      return 1;
    else
      return 0;
  }

  return -1;
}

// ------------------- NeoPixel Helpers -------------------

void clearStrip() {
  strip.clear();
  delay(10);
  strip.show();
  delay(10);
}

void showColor(uint32_t color) {

  clearStrip();

  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, color);
  }

  strip.show();
}

// ------------------- Wait for Button Input -------------------

void waitForInput() {

  int KEY_INPUT = 21;

  pinMode(KEY_INPUT, INPUT_PULLUP);

  while (digitalRead(KEY_INPUT)) {
    delay(20);
  }

  while (!digitalRead(KEY_INPUT))
    ;

  delay(100);
}

// ------------------- Ultrasonic Distance -------------------

long getDistanceOnce(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000);

  if (duration == 0) return 200;

  return duration * 0.034 / 2;
}


// =============================================================
// =============== MOTOR FUNCTIONS =============================
// =============================================================

void setMotors(int LM, int RM) {

  if (LM >= 0) {
    analogWrite(LMF, LM);
    analogWrite(LMR, 0);
  }
  else {
    analogWrite(LMF, 0);
    analogWrite(LMR, -LM);
  }

  if (RM >= 0) {
    analogWrite(RMF, RM);
    analogWrite(RMR, 0);
  }
  else {
    analogWrite(RMF, 0);
    analogWrite(RMR, -RM);
  }
}

void stopMotors() {
  setMotors(0, 0);
}


// =============================================================
// =============== MOVEMENT FUNCTIONS ==========================
// =============================================================

void moveStraightAligned(int leftDistance) {

  int error = 0;

  mpu.update();

  int gyroerror = mpu.getAngleZ();

  if (leftDistance < wallDetect) {
    error = leftDistance - wallOffset;
  }

  int L = BASESPEED_move - error * Kp_run + gyroerror * Kp_gyro;
  int R = BASESPEED_move + error * Kp_run - gyroerror * Kp_gyro;

  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);

  setMotors(L, R);
}


// ------------------- Turn Robot -------------------

void turnBot(int degrees, int direction) {

  stopMotors();

  delay(5);

  mpu.resetAngleZ();

  unsigned long timer = 0;
  unsigned long timerExit = 0;

  bool enableExitTimer = false;

  float _pe = 0;

  unsigned long MAX_TIMER = millis();

  while (true) {

    mpu.update();

    if ((millis() - timer) > 10) {

      int error = abs(degrees) - abs(mpu.getAngleZ());

      if (error <= 0 && !enableExitTimer) {
        enableExitTimer = true;
        timerExit = millis();
      }

      float _ce = error - _pe;
      _pe = error;

      float _p = error * kp_turn;
      float _d = _ce * kd_turn;

      int drive = _p + _d;

      float turning = MAXSPEED_turning;
      int minSpeed = 60;

      drive = constrain(drive, -turning, turning);

      if (drive > 0 && drive < minSpeed) drive = minSpeed;
      if (drive < 0 && drive > -minSpeed) drive = -minSpeed;

      setMotors(direction * drive, -direction * drive);

      if (enableExitTimer && (millis() - timerExit) > 200) break;
      if ((millis() - MAX_TIMER) > 1700) break;

      timer = millis();
    }
  }

  stopMotors();

  delay(100);

  mpu.resetAngleZ();
}


// =============================================================
// =============== MPU FUNCTIONS ===============================
// =============================================================

void beginMpu() {

  byte status = mpu.begin();

  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  if (status != 0) {

    while (1) {
      Serial.println("MPU ERROR!");
      delay(500);
    }
  }

  delay(100);

  Serial.println("MPU START OK");
}


// =============================================================
// =============== SETUP =======================================
// =============================================================

void setup() {

  Wire.setSCL(17);
  Wire.setSDA(16);
  Wire.begin();

  Serial.begin(115200);

  // ---------------- NICLA UART SETUP (NEW) ----------------

  Serial1.setRX(NICLA_RX);
  Serial1.setTX(NICLA_TX);
  Serial1.begin(115200);

  Serial.println("Nicla Vision UART READY!");

  myservo.attach(1);
  myservo.write(0);

  // ------------------- NeoPixel -------------------

  strip.begin();
  strip.show();
  strip.setBrightness(50);

  // ------------------- Motors -------------------

  stopMotors();

  // ------------------- Ultrasonic -------------------

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  // -------------------- AS7341 --------------------

  initAS7341Sensor();

  // ------------------- Startup sequence -------------------

  showColor(strip.Color(127, 0, 0));

  showColor(strip.Color(0, 50, 0));

  Serial.println("start");

  delay(3000);

  // ------------------- MPU -------------------

  beginMpu();

  mpu.calcOffsets(true, true);
  mpu.resetAngleZ();

  // Warm-up sensors

  for (int i = 0; i < 10; i++) {

    mpu.update();

    getDistanceOnce(TRIG_LEFT, ECHO_LEFT);
    getDistanceOnce(TRIG_FRONT, ECHO_FRONT);

    delay(50);
  }

  Serial.println("calibrated...");

  showColor(strip.Color(0, 127, 0));

  as7341.enableLED(true);

  delay(100);

  _startAI = true;

  dataincoming = false;
  _target = 0;
  _confidence = 0;
}


// =============================================================
// =============== MAIN LOOP ===================================
// =============================================================

void loop() {

  // -------- NICLA VISION RECEIVE DATA (NEW) --------

  receiveNiclaData();

  // -------- EXISTING ROBOT PROGRAMME --------

  mpu.update();

  int distLeft = getDistanceOnce(TRIG_LEFT, ECHO_LEFT);
  int distFront = getDistanceOnce(TRIG_FRONT, ECHO_FRONT);

  distLeft = (distLeftLAST * 4 + distLeft * 6) / 10;
  distFront = (distFrontLAST * 5 + distFront * 5) / 10;

  distLeftLAST = distLeft;
  distFrontLAST = distFront;

  int color = readColor();

  if (dataincoming) {

    long curr;

    if (_confidence > 90) {

      delay(100);

      switch (_target) {

        case 0:

          Serial.println("Case Harmed triggered");

          setMotors(0, 0);

          for (int i = 0; i < 2; i++) {

            showColor(strip.Color(0, 255, 0));
            delay(300);

            showColor(strip.Color(0, 0, 0));
            delay(300);
          }

          kitdrop();
          kitdrop();

          curr = millis();

          delay(1);

          while (millis() - curr < 600) {
            moveStraightAligned(wallOffset);
            delay(10);
          }

          break;

        case 1:

          Serial.println("Case Survivor triggered");

          setMotors(0, 0);

          for (int i = 0; i < 2; i++) {

            showColor(strip.Color(0, 255, 255));
            delay(300);

            showColor(strip.Color(0, 0, 0));
            delay(300);
          }

          kitdrop();

          curr = millis();

          delay(1);

          while (millis() - curr < 600) {
            moveStraightAligned(wallOffset);
            delay(10);
          }

          break;

        case 2:

          setMotors(0, 0);

          for (int i = 0; i < 2; i++) {

            showColor(strip.Color(255, 0, 0));
            delay(100);

            showColor(strip.Color(0, 0, 0));
            delay(100);
          }

          curr = millis();

          delay(1);

          while (millis() - curr < 600) {
            moveStraightAligned(wallOffset);
            delay(10);
          }

          break;

        case 3:
          break;

        case 4:

          Serial.println("Case Y triggered");

          break;

        case 5:

          Serial.println("Case G triggered");

          break;

        case 6:

          Serial.println("Case R triggered");

          break;

        default:

          Serial.println("Unknown target");

          break;
      }
    }

    dataincoming = false;
    _target = 0;
    _confidence = 0;
  }

  // ------------------- Obstacle handling -------------------

  if (color == 0) {

    turnBot(90, RIGHT);
    turnBot(90, RIGHT);

  }
  else if (color == 1) {

    delay(10);

    long curr = millis();

    delay(1);

    while (millis() - curr < 800) {
      moveStraightAligned(wallOffset);
      delay(10);
    }

    stopMotors();

    for (int i = 0; i < 3; i++) {

      showColor(strip.Color(0, 0, 255));
      delay(50);

      showColor(strip.Color(0, 0, 0));
      delay(50);
    }

    curr = millis();

    delay(1);

    while (millis() - curr < 5000) {

      blink(5);

      stopMotors();

      delay(300);
    }

    curr = millis();

    delay(1);

    while (millis() - curr < 800) {
      moveStraightAligned(wallOffset);
      delay(3060);
    }
  }

  if (distFront < 9) {

    stopMotors();

    delay(200);

    distLeft = getDistanceOnce(TRIG_LEFT, ECHO_LEFT);

    if (distLeft < 15)
      turnBot(90, RIGHT);
    else
      turnBot(90, LEFT);
  }

  if (distLeft > 30) {

    long curr = millis();

    delay(1);

    while (millis() - curr < 700) {
      moveStraightAligned(wallOffset);
      delay(100);
    }

    if (distLeft < 30) {

      showColor(strip.Color(0, 0, 127));

      curr = millis();

      delay(1);

      while (millis() - curr < 1000) {
        moveStraightAligned(wallOffset);
        delay(800);
      }

    }
    else if (distLeft > 30) {

      delay(200);

      stopMotors();

      turnBot(90, LEFT);

      curr = millis();

      delay(1);

      while (millis() - curr < 900) {
        moveStraightAligned(wallOffset);
        delay(7);
      }
    }
  }

  // ------------------- Default: move forward -------------------

  moveStraightAligned(distLeft);

  delay(50);
}


// =============================================================
// =============== OTHER FUNCTIONS =============================
// =============================================================

void kitdrop() {

  myservo.write(0);
  delay(800);

  myservo.write(180);
  delay(800);

  myservo.write(0);
  delay(800);
}

void blink(int times) {

  for (int i = 0; i < times; i++) {

    as7341.enableLED(true);
    delay(300);

    as7341.enableLED(false);
    delay(300);
  }

  as7341.enableLED(true);
}


// =============================================================
// =============== SECOND CORE: EXISTING AI CAMERA ==============
// =============================================================

void setup1() {

  while (!_startAI) {
    delay(1);
  }

  Wire1.setSDA(18);
  Wire1.setSCL(19);

  Wire1.begin();

  AI.begin(&Wire1);
}

void loop1() {

  if (!AI.invoke(1, false, false) && !dataincoming) {

    for (int i = 0; i < AI.classes().size(); i++) {

      dataincoming = true;

      _target = AI.classes()[i].target;
      _confidence = AI.classes()[i].score;
    }
  }
}
