#include <DHT.h>

#define DHTPIN 2       // AM2302 (DHT22) Sensor Pin (Digital 2)
#define DHTTYPE DHT22  // Sensor type
DHT dht(DHTPIN, DHTTYPE);

// L298 Motor Driver Pins (for window open/close)
const int ENA = 3;
const int IN1 = 4;
const int IN2 = 5;

// PM2.5 Dust Sensor Pins (GP2Y1023AU0F)
const int DUST_ANALOG_PIN = A0; // Dust sensor analog output
const int DUST_LED_PIN = 8;     // Dust sensor internal LED control pin
const int DUST_SAMPLES = 5;     // 미세먼지는 여러 번 측정해 평균을 보낸다 (잡음 감소)

// 미세먼지 단계 표시용 RGB LED (PWM 가능 핀 + 각 색마다 220Ω 저항)
// 서버(라즈베리파이)가 미세먼지 수치에 따라 LED_BLUE / LED_GREEN / LED_ORANGE / LED_RED 명령을 보낸다.
const int LED_R_PIN = 9;
const int LED_G_PIN = 10;
const int LED_B_PIN = 11;
const bool LED_COMMON_ANODE = false;  // 공통 애노드 RGB LED라면 true (공통 캐소드는 false)

// ---------------- 센서 오류 처리 ----------------
// DHT 읽기 실패/범위 이탈 시 TEMP/HUMI 자리에 보내는 값 (서버의 DHT_ERROR_THRESHOLD와 대응)
const float SENSOR_ERROR = -999.0;

// 값이 NaN이거나 물리적으로 말이 안 되는 범위면 오류값으로 바꾼다
float validOrError(float v, float lo, float hi) {
  if (isnan(v) || v < lo || v > hi) return SENSOR_ERROR;
  return v;
}

// ---------------- 모터 안전장치 ----------------
const int MOTOR_SPEED = 255;                    // 0~255
// 서버/라즈베리가 멈춰도 이 시간이 지나면 아두이노가 스스로 모터를 끈다.
// 서버의 MOVE_SECONDS(기본 54초)보다 최소 5초 이상 길게 설정할 것.
// (서버의 정지 명령이 재시도/재연결로 몇 초 늦어져도 이 상한이 먼저 걸리지 않도록)
const unsigned long MAX_RUN_MS = 75000;
// 이동 중 반대 방향 명령이 오면 먼저 멈추고 이 시간만큼 기다린 뒤 반전 (모터/L298 보호)
const unsigned long REVERSE_DELAY_MS = 300;

unsigned long motorStart = 0;
bool motorRunning = false;
int motorDir = 0;         // +1: 열기, -1: 닫기, 0: 정지
int timeoutLockDir = 0;   // 최대 시간 초과로 멈춘 방향. WINDOW_STOP 또는 반대 방향 명령이 오기 전까지
                          // 같은 방향 명령은 무시한다 (같은 명령을 반복 전송하는 프로그램이 있어도 구동 시간 제한 유지)

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  motorRunning = false;
  motorDir = 0;
}

// 반환: 1 = 새로 시작, 0 = 이미 같은 방향으로 동작 중 (타이머는 연장하지 않음), -1 = 타임아웃 잠금으로 무시
int startMotor(int dir) {
  if (dir == timeoutLockDir) return -1;          // 시간 초과로 멈춘 방향의 재명령은 무시
  if (motorRunning && motorDir == dir) return 0; // 같은 방향 반복 명령으로 타이머를 늘리지 않는다

  // 반대 방향으로 이미 돌고 있으면 급반전하지 않고 잠깐 멈췄다가 시작
  if (motorRunning) {
    stopMotor();
    delay(REVERSE_DELAY_MS);
  }

  timeoutLockDir = 0;  // 반대 방향 명령이므로 잠금 해제
  digitalWrite(IN1, dir > 0 ? HIGH : LOW);
  digitalWrite(IN2, dir > 0 ? LOW : HIGH);
  analogWrite(ENA, MOTOR_SPEED);
  motorDir = dir;
  motorStart = millis();
  motorRunning = true;
  return 1;
}

// 열기/닫기 명령 처리 후 응답 전송. 서버는 "<명령>_SUCCESS"를 기대한다.
void handleMove(int dir, const char* name) {
  int r = startMotor(dir);
  Serial.print("ACK: ");
  Serial.print(name);
  if (r >= 0) {
    Serial.println("_SUCCESS");
  } else {
    Serial.println("_IGNORED_TIMEOUT_LOCK");  // 시간 초과로 정지한 상태: STOP 또는 반대 명령 필요
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

// ---------------- 미세먼지 측정 ----------------
// 센서 LED를 280µs 켠 시점에 읽는 방식으로 여러 번 측정한 평균(ADC 원시값 0~1023)
int readDust() {
  long sum = 0;
  for (int i = 0; i < DUST_SAMPLES; i++) {
    digitalWrite(DUST_LED_PIN, LOW);  // Power on dust sensor LED
    delayMicroseconds(280);
    sum += analogRead(DUST_ANALOG_PIN);
    delayMicroseconds(40);
    digitalWrite(DUST_LED_PIN, HIGH); // Power off dust sensor LED
    delay(10);                        // 센서 측정 주기(약 10ms)를 맞춘다
  }
  return (int)(sum / DUST_SAMPLES);
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
  digitalWrite(DUST_LED_PIN, HIGH);  // 센서 LED는 평소 꺼 둔다
  stopMotor();  // 부팅 직후 모터가 도는 일이 없도록 정지 상태로 시작
  setLed(0, 0, 0);  // 서버가 미세먼지 단계를 알려 줄 때까지 표시 LED는 꺼 둔다
}

void loop() {
  // 최대 동작 시간 초과 시 자동 정지 (통신 두절 대비)
  if (motorRunning && (millis() - motorStart > MAX_RUN_MS)) {
    timeoutLockDir = motorDir;  // 이 방향의 재명령은 STOP/반대 명령이 올 때까지 무시
    stopMotor();
    Serial.println("ACK: AUTO_STOP_TIMEOUT");
  }

  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // Remove spaces and newline characters

    // 1. Request sensor data (Temp, Humidity, Dust)
    //    비 판단은 비 센서가 아니라 기상청 정보로 서버가 한다.
    if (cmd == "GET_SENSOR") {
      float h = validOrError(dht.readHumidity(), 0.0, 100.0);
      float t = validOrError(dht.readTemperature(), -40.0, 80.0);
      int dustVal = readDust();

      // DHT가 실패해도 미세먼지는 계속 보낸다 (실패한 값만 -999로 표시)
      Serial.print("TEMP:");
      Serial.print(t);
      Serial.print(",HUMI:");
      Serial.print(h);
      Serial.print(",DUST:");
      Serial.println(dustVal);
    }
    // 2. Open window (motor forward)
    else if (cmd == "WINDOW_OPEN") {
      handleMove(+1, "WINDOW_OPEN");
    }
    // 3. Close window (motor reverse)
    else if (cmd == "WINDOW_CLOSE") {
      handleMove(-1, "WINDOW_CLOSE");
    }
    // 4. Stop motor (타임아웃 잠금도 해제)
    else if (cmd == "WINDOW_STOP") {
      stopMotor();
      timeoutLockDir = 0;
      Serial.println("ACK: WINDOW_STOP_SUCCESS");
    }
    // 5. 미세먼지 단계 LED: 0~30 파랑, 31~80 초록, 81~150 주황, 151 이상 빨강
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
