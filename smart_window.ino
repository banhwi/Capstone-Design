#include <DHT.h>

#define DHTPIN 2       // AM2302 (DHT22) Sensor Pin (Digital 2)
#define DHTTYPE DHT22  // Sensor type
DHT dht(DHTPIN, DHTTYPE);

// L298 Motor Driver Pins (for window open/close)
const int ENA = 3;
const int IN1 = 4;
const int IN2 = 5;

// Rain Sensor Pin (Digital input from rain sensor relay module)
const int RAIN_PIN = 7;

// PM2.5 Dust Sensor Pins (GP2Y1023AU0F)
const int DUST_ANALOG_PIN = A0; // Dust sensor analog output
const int DUST_LED_PIN = 8;     // Dust sensor internal LED control pin

// ---------------- 센서 오류 처리 ----------------
// DHT 읽기 실패/범위 이탈 시 TEMP/HUMI 자리에 보내는 값 (서버의 DHT_ERROR_THRESHOLD와 대응)
const float SENSOR_ERROR = -999.0;

// 값이 NaN이거나 물리적으로 말이 안 되는 범위면 오류값으로 바꾼다
float validOrError(float v, float lo, float hi) {
  if (isnan(v) || v < lo || v > hi) return SENSOR_ERROR;
  return v;
}

// ---------------- 모터 안전장치 ----------------
const int MOTOR_SPEED = 200;                    // 0~255
// 서버/라즈베리가 멈춰도 이 시간이 지나면 아두이노가 스스로 모터를 끈다.
// 서버의 MOVE_SECONDS(기본 5초)보다 길게 설정할 것.
const unsigned long MAX_RUN_MS = 8000;
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

void setup() {
  Serial.begin(9600);
  dht.begin();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(RAIN_PIN, INPUT);
  pinMode(DUST_LED_PIN, OUTPUT);
  digitalWrite(DUST_LED_PIN, HIGH);  // 센서 LED는 평소 꺼 둔다
  stopMotor();  // 부팅 직후 모터가 도는 일이 없도록 정지 상태로 시작
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

    // 1. Request sensor data (Temp, Humidity, Rain, Dust)
    if (cmd == "GET_SENSOR") {
      float h = validOrError(dht.readHumidity(), 0.0, 100.0);
      float t = validOrError(dht.readTemperature(), -40.0, 80.0);
      int rainState = digitalRead(RAIN_PIN); // LOW = 비 감지 (릴레이 모듈 기준)

      // Read PM2.5 Dust Sensor value
      digitalWrite(DUST_LED_PIN, LOW); // Power on dust sensor LED
      delayMicroseconds(280);
      int dustVal = analogRead(DUST_ANALOG_PIN); // Read dust density
      delayMicroseconds(40);
      digitalWrite(DUST_LED_PIN, HIGH); // Power off dust sensor LED

      // DHT가 실패해도 비/미세먼지는 계속 보낸다 (실패한 값만 -999로 표시)
      Serial.print("TEMP:");
      Serial.print(t);
      Serial.print(",HUMI:");
      Serial.print(h);
      Serial.print(",RAIN:");
      Serial.print(rainState);
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
    else {
      Serial.println("ACK: UNKNOWN_COMMAND");
    }
  }
}
