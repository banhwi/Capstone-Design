#include <DHT.h>

#define DHTPIN 2       // AM2302 (DHT22) Sensor Pin (Digital 2)
#define DHTTYPE DHT22  // Sensor type
DHT dht(DHTPIN, DHTTYPE);

// L298 Motor Driver Pins (for window open/close)
const int ENA = 3;
const int IN1 = 4;
const int IN2 = 5;

// 빗물 감지 센서 릴레이 모듈 (네오틱스 NS-RSRM)
// 배선: 모듈 +5V -> 5V, GND -> GND, COM -> GND, NO -> D6
// 물 감지 시 릴레이가 붙어 D6이 LOW가 되며, 서버로 RAIN:1 전송
const int RAIN_RELAY_PIN = 6;

// PM2.5 Dust Sensor Pins (GP2Y1023AU0F)
const int DUST_ANALOG_PIN = A0; // Dust sensor analog output
const int DUST_LED_PIN = 8;     // Dust sensor internal LED control pin
const int DUST_SAMPLES = 5;     // 미세먼지는 여러 번 측정해 평균을 보낸다 (잡음 감소)

// 미세먼지 단계 표시용 RGB LED (PWM 가능 핀 + 각 색마다 220Ω 저항)
const int LED_R_PIN = 9;
const int LED_G_PIN = 10;
const int LED_B_PIN = 11;
const bool LED_COMMON_ANODE = false;  // 공통 애노드 RGB LED라면 true (공통 캐소드는 false)

// ---------------- 센서 오류 처리 ----------------
const float SENSOR_ERROR = -999.0;

float validOrError(float v, float lo, float hi) {
  if (isnan(v) || v < lo || v > hi) return SENSOR_ERROR;
  return v;
}

// ---------------- 모터 안전장치 ----------------
const int MOTOR_SPEED = 255;
const unsigned long MAX_RUN_MS = 75000;
const unsigned long REVERSE_DELAY_MS = 300;

unsigned long motorStart = 0;
bool motorRunning = false;
int motorDir = 0;
int timeoutLockDir = 0;

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  motorRunning = false;
  motorDir = 0;
}

int startMotor(int dir) {
  if (dir == timeoutLockDir) return -1;
  if (motorRunning && motorDir == dir) return 0;

  if (motorRunning) {
    stopMotor();
    delay(REVERSE_DELAY_MS);
  }

  timeoutLockDir = 0;
  digitalWrite(IN1, dir > 0 ? HIGH : LOW);
  digitalWrite(IN2, dir > 0 ? LOW : HIGH);
  analogWrite(ENA, MOTOR_SPEED);
  motorDir = dir;
  motorStart = millis();
  motorRunning = true;
  return 1;
}

void handleMove(int dir, const char* name) {
  int r = startMotor(dir);
  Serial.print("ACK: ");
  Serial.print(name);
  if (r >= 0) {
    Serial.println("_SUCCESS");
  } else {
    Serial.println("_IGNORED_TIMEOUT_LOCK");
  }
}

// ---------------- 미세먼지 LED ----------------
void writeLedPin(int pin, int value) {
  analogWrite(pin, LED_COMMON_ANODE ? 255 - value : value);
}

void setLed(int r, int g, int b) {
  writeLedPin(LED_R_PIN, r);
  writeLedPin(LED_G_PIN, g);
  writeLedPin(LED_B_PIN, b);
}

void handleLed(const char* name, int r, int g, int b) {
  setLed(r, g, b);
  Serial.print("ACK: ");
  Serial.print(name);
  Serial.println("_SUCCESS");
}

// ---------------- 센서 측정 ----------------
int readDust() {
  long sum = 0;
  for (int i = 0; i < DUST_SAMPLES; i++) {
    digitalWrite(DUST_LED_PIN, LOW);
    delayMicroseconds(280);
    sum += analogRead(DUST_ANALOG_PIN);
    delayMicroseconds(40);
    digitalWrite(DUST_LED_PIN, HIGH);
    delay(10);
  }
  return (int)(sum / DUST_SAMPLES);
}

// NS-RSRM 릴레이 접점 읽기: 젖음(LOW) -> 1, 마름(HIGH) -> 0 반환
int readRain() {
  return (digitalRead(RAIN_RELAY_PIN) == LOW) ? 1 : 0;
}

void setup() {
  Serial.begin(9600);
  dht.begin();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(DUST_LED_PIN, OUTPUT);
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);

  // NS-RSRM 릴레이 접점용 내부 풀업 입력
  pinMode(RAIN_RELAY_PIN, INPUT_PULLUP);

  digitalWrite(DUST_LED_PIN, HIGH);
  stopMotor();
  setLed(0, 0, 0);
}

void loop() {
  if (motorRunning && (millis() - motorStart > MAX_RUN_MS)) {
    timeoutLockDir = motorDir;
    stopMotor();
    Serial.println("ACK: AUTO_STOP_TIMEOUT");
  }

  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "GET_SENSOR") {
      float h = validOrError(dht.readHumidity(), 0.0, 100.0);
      float t = validOrError(dht.readTemperature(), -40.0, 80.0);
      int rainVal = readRain();
      int dustVal = readDust();

      Serial.print("TEMP:");
      Serial.print(t);
      Serial.print(",HUMI:");
      Serial.print(h);
      Serial.print(",RAIN:");
      Serial.print(rainVal);
      Serial.print(",DUST:");
      Serial.println(dustVal);
    }
    else if (cmd == "WINDOW_OPEN") {
      handleMove(+1, "WINDOW_OPEN");
    }
    else if (cmd == "WINDOW_CLOSE") {
      handleMove(-1, "WINDOW_CLOSE");
    }
    else if (cmd == "WINDOW_STOP") {
      stopMotor();
      timeoutLockDir = 0;
      Serial.println("ACK: WINDOW_STOP_SUCCESS");
    }
    else if (cmd == "LED_BLUE")   { handleLed("LED_BLUE",   0,   0, 255); }
    else if (cmd == "LED_GREEN")  { handleLed("LED_GREEN",  0, 255,   0); }
    else if (cmd == "LED_ORANGE") { handleLed("LED_ORANGE", 255, 70,  0); }
    else if (cmd == "LED_RED")    { handleLed("LED_RED",    255,  0,  0); }
    else if (cmd == "LED_OFF")    { handleLed("LED_OFF",      0,  0,  0); }
    else {
      Serial.println("ACK: UNKNOWN_COMMAND");
    }
  }
}