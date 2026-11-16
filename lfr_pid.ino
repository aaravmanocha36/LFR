/*
 * LFR - PID line follower
 * Board : ESP32 NodeMCU 30-pin (CP2102)
 * Sensor: 8-channel analog IR array (pins D1-D8, IR, VCC, GND), Driver: TB6612FNG,
 * Motors: 2x N20 6V, no encoders
 *
 * HOW TO USE
 *  1. Power on. The LED blinks for 5 s: slide the robot side to side across the
 *     line by hand so every sensor sees both black and white (calibration).
 *  2. LED goes solid. Press BOOT to start. Press BOOT again to stop.
 *
 * !!! BEFORE ANY SCORED RUN: set TUNING_MODE to 0 and re-flash. !!!
 *     Wireless on during a run = disqualification.
 */

#define TUNING_MODE 1   // 1 = Bluetooth tuning ON (practice only), 0 = all radios OFF (competition)

#define DEBUG_SERIAL 1  // 1 = print sensor position to USB Serial Monitor (115200) while the robot is stopped

#include <Arduino.h>
#include <WiFi.h>
#include "driver/gpio.h"

#if TUNING_MODE
  #include "BluetoothSerial.h"
  BluetoothSerial SerialBT;
  #warning "TUNING_MODE is 1: Bluetooth is ON. Set it to 0 before competition."
#endif

// ---------------- Pins ----------------
const uint8_t SENSOR_PIN[8] = {13, 14, 27, 26, 25, 33, 32, 34};  // D1..D8, left -> right (D8 on D34, analog-capable)
const uint8_t LEDON_PIN = 22;   // QTR LEDON (optional)
const uint8_t AIN1 = 18, AIN2 = 19, PWMA = 5;    // left motor
const uint8_t BIN1 = 16, BIN2 = 17, PWMB = 4;    // right motor
const uint8_t STBY = 23;
const uint8_t BTN  = 0;         // BOOT button
const uint8_t LED  = 2;         // onboard LED

// ---------------- Tunables ----------------
float Kp = 0.030f;              // start here, then tune (see notes)
float Ki = 0.0f;
float Kd = 0.30f;
int   baseSpeed   = 140;        // 0..MAX_DUTY
int   minSpeed    = 70;         // speed floor in sharp curves
int   MAX_DUTY    = 200;        // N20 is 6V: 200/255 of 8.4V is about 6.6V. Lower if motors get hot.
float slowdown    = 0.5f;       // fraction of baseSpeed removed at max error

const uint16_t ADC_MAX = 4095;      // 12-bit ADC range
const uint16_t LINE_THRESH = 150;   // sum of normalised values below this = line lost

// ---------------- State ----------------
uint16_t cmin[8], cmax[8];
bool running = false;
float lastError = 0, integral = 0;
float lastSeenDir = 1;              // +1 line last seen on right, -1 on left
uint32_t lastTelemetry = 0;

// ---------------- Motors ----------------
void pwmSetup(uint8_t pin, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, 20000, 8);
#else
  ledcSetup(ch, 20000, 8);
  ledcAttachPin(pin, ch);
#endif
}
void pwmWrite(uint8_t pin, uint8_t ch, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}
void setMotor(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t ch, int speed) {
  speed = constrain(speed, -MAX_DUTY, MAX_DUTY);
  if (speed >= 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else            { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); speed = -speed; }
  pwmWrite(pwmPin, ch, speed);
}
void drive(int left, int right) {
  setMotor(AIN1, AIN2, PWMA, 0, left);
  setMotor(BIN1, BIN2, PWMB, 1, right);
}
void stopMotors() { drive(0, 0); }

// ---------------- Sensors ----------------
// Reads all 8 analog sensors. t[i] = 0..4095 (higher = darker / less reflection).
void readRaw(uint16_t *t) {
  for (int i = 0; i < 8; i++) t[i] = analogRead(SENSOR_PIN[i]);
}

void calibrate() {
  for (int i = 0; i < 8; i++) { cmin[i] = ADC_MAX; cmax[i] = 0; }
  uint32_t end = millis() + 5000;
  uint16_t t[8];
  while (millis() < end) {
    readRaw(t);
    for (int i = 0; i < 8; i++) {
      if (t[i] < cmin[i]) cmin[i] = t[i];
      if (t[i] > cmax[i]) cmax[i] = t[i];
    }
    digitalWrite(LED, (millis() / 100) & 1);
  }
  digitalWrite(LED, HIGH);
}

// Returns position 0..7000 (3500 = centred) and total signal. Black line = high value.
bool readLine(float &position, uint16_t &total) {
  uint16_t t[8];
  readRaw(t);
  uint32_t num = 0, den = 0;
  for (int i = 0; i < 8; i++) {
    int range = (int)cmax[i] - (int)cmin[i];
    int v = 0;
    if (range > 100) v = constrain(((int)t[i] - (int)cmin[i]) * 1000 / range, 0, 1000);
    if (v < 80) v = 0;                 // ignore noise
    num += (uint32_t)v * (i * 1000);
    den += v;
  }
  total = den;
  if (den < LINE_THRESH) return false;
  position = (float)num / den;
  return true;
}

// ---------------- Control ----------------
void runStep() {
  float pos; uint16_t total;
  if (!readLine(pos, total)) {
    // Line lost: pivot back toward where it was last seen
    int s = baseSpeed;
    if (lastSeenDir > 0) drive(s, -s); else drive(-s, s);
    return;
  }
  float error = pos - 3500.0f;
  lastSeenDir = (error >= 0) ? 1 : -1;

  integral = constrain(integral + error, -5000, 5000);
  float derivative = error - lastError;
  lastError = error;

  float corr = Kp * error + Ki * integral + Kd * derivative;

  // Slow down in curves, full speed on straights
  float frac = fabsf(error) / 3500.0f;
  int speed = baseSpeed - (int)(slowdown * baseSpeed * frac);
  if (speed < minSpeed) speed = minSpeed;

  drive(speed + (int)corr, speed - (int)corr);
}

// ---------------- Tuning over Bluetooth ----------------
#if TUNING_MODE
void handleTuning() {
  if (!SerialBT.available()) return;
  String s = SerialBT.readStringUntil('\n'); s.trim(); s.toLowerCase();
  int sp = s.indexOf(' ');
  String k = sp > 0 ? s.substring(0, sp) : s;
  float v = sp > 0 ? s.substring(sp + 1).toFloat() : 0;
  if      (k == "kp")   Kp = v;
  else if (k == "ki")   Ki = v;
  else if (k == "kd")   Kd = v;
  else if (k == "base") baseSpeed = (int)v;
  else if (k == "min")  minSpeed = (int)v;
  else if (k == "slow") slowdown = v;
  else if (k == "max")  MAX_DUTY = (int)v;
  else if (k == "go")   { running = true; integral = 0; lastError = 0; }
  else if (k == "stop") { running = false; stopMotors(); }
  SerialBT.printf("Kp=%.4f Ki=%.5f Kd=%.4f base=%d min=%d slow=%.2f max=%d\n",
                  Kp, Ki, Kd, baseSpeed, minSpeed, slowdown, MAX_DUTY);
}
#endif

void setup() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(LEDON_PIN, OUTPUT); digitalWrite(LEDON_PIN, HIGH);
  pinMode(LED, OUTPUT);
#if DEBUG_SERIAL
  Serial.begin(115200);
#endif
  pinMode(BTN, INPUT_PULLUP);
  analogSetAttenuation(ADC_11db);   // full 0-3.3V range on the sensor pins
  for (int i = 0; i < 8; i++) pinMode(SENSOR_PIN[i], INPUT);
  pwmSetup(PWMA, 0);
  pwmSetup(PWMB, 1);
  stopMotors();

#if TUNING_MODE
  SerialBT.begin("LFR-Tune");
#else
  WiFi.mode(WIFI_OFF);
  btStop();                 // radios hard off for competition
#endif

  calibrate();
}

void loop() {
  static bool btnPrev = true;
  bool btn = digitalRead(BTN);
  if (btnPrev && !btn) {            // pressed
    running = !running;
    integral = 0; lastError = 0;
    if (running) delay(1000); else stopMotors();
  }
  btnPrev = btn;

#if DEBUG_SERIAL
  // While stopped: show the 8 normalised sensor values (0=white .. 1000=black) and line position
  static uint32_t lastDbg = 0;
  if (!running && millis() - lastDbg > 200) {
    lastDbg = millis();
    uint16_t raw[8];
    readRaw(raw);
    for (int i = 0; i < 8; i++) {
      int range = (int)cmax[i] - (int)cmin[i];
      int v = range > 100 ? constrain(((int)raw[i] - (int)cmin[i]) * 1000 / range, 0, 1000) : 0;
      Serial.printf("%4d ", v);
    }
    float pos; uint16_t total;
    if (readLine(pos, total)) Serial.printf("| pos=%.0f (3500 = centre)\n", pos);
    else                      Serial.println("| NO LINE");
  }
#endif

#if TUNING_MODE
  handleTuning();
  if (running && millis() - lastTelemetry > 160) {
    lastTelemetry = millis();
    SerialBT.printf("err=%.0f\n", lastError);
  }
#endif

  if (running) runStep();
}
