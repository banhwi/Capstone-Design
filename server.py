from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS
import serial
import time
import threading
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

app = Flask(__name__)
CORS(app)

# -----------------------------------------------------------------
# 0. 설정값
# -----------------------------------------------------------------
PORT = '/dev/ttyACM0'      # 윈도우에서 테스트할 땐 'COM3' 등으로 변경
BAUD_RATE = 9600

SENSOR_INTERVAL = 3.0      # 센서 요청 주기(초). DHT22는 최소 2초 간격 필요
READ_TIMEOUT = 2.0         # GET_SENSOR 응답 대기 한도(초)
MOVE_SECONDS = 5.0         # 창문이 완전히 열리/닫히는 데 걸리는 시간(실측해서 조정!)
                           # 아두이노의 MAX_RUN_MS(기본 8000ms)보다 반드시 짧아야 함
FAIL_LIMIT = 3             # 연속 이 횟수만큼 실패하면 장애 알림
FAILSAFE_CLOSE_ON_FAULT = True  # 자동 모드에서 '통신 장애' 시 안전을 위해 창문 닫기

# 아두이노가 DHT 읽기 실패 시 TEMP/HUMI 자리에 보내는 값(-999). 이 이하이면 DHT 오류로 판단
DHT_ERROR_THRESHOLD = -990

# 미세먼지 원시값(ADC 0~1023) -> μg/m³ 근사 변환 계수
# Sharp 계열 센서 근사식: density(μg/m³) = (K * Vo - OFFSET) * 1000
# 센서/보정에 따라 오차가 크므로 실제 측정기와 비교해 K, OFFSET을 보정하세요.
ADC_REF_VOLTAGE = 5.0
DUST_K = 0.17
DUST_OFFSET = 0.1


def raw_to_ugm3(raw):
    voltage = raw * ADC_REF_VOLTAGE / 1024.0
    density = (DUST_K * voltage - DUST_OFFSET) * 1000.0
    return max(0, int(round(density)))


# -----------------------------------------------------------------
# 1. 시리얼 통신 설정
# -----------------------------------------------------------------
try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # 아두이노 부팅 대기
    print(">>> 아두이노 시리얼 통신 연결 성공!")
except Exception as e:
    ser = None
    print(f">>> [경고] 아두이노 연결 실패: {e} (가상 모드로 동작합니다)")

# 시리얼 포트 접근을 직렬화하는 락 (스레드 안전)
serial_lock = threading.Lock()
# 창문 상태/모터 타이머 접근용 락 (항상 state_lock -> serial_lock 순서로 획득)
state_lock = threading.RLock()

# -----------------------------------------------------------------
# 2. 시스템 전역 상태
# -----------------------------------------------------------------
system_data = {
    "indoor_temp": 0.0,
    "indoor_hum": 0.0,
    "outdoor_temp": 18.2,   # 실외 센서가 없다면 API나 고정값 사용
    "outdoor_hum": 62.0,
    "dust": 0,              # μg/m³ (변환값)
    "dust_raw": 0,          # ADC 원시값 (보정용)
    "rain_detected": False,
    "window_state": "UNKNOWN",  # 시작 시 실제 위치를 모르므로 첫 판단 때 반드시 명령을 보냄
    "moving": False,        # 모터 동작 중 여부
    "fault_alert": 0,
    "fault_message": "",
    "mode": "AUTO",
    "dust_threshold": 35,
}

move_end_time = 0.0

# 장애 원인을 종류별로 관리: comm = 통신 장애, dht = 온습도 센서 장애
fault_reasons = {"comm": None, "dht": None}
comm_fail_count = 0
dht_fail_count = 0


# -----------------------------------------------------------------
# 3. 시리얼 입출력 함수 (모든 시리얼 접근은 여기를 거친다)
# -----------------------------------------------------------------
def read_sensor_line():
    """버퍼를 비우고 요청한 뒤, TEMP: 로 시작하는 줄이 나올 때까지 읽는다.
    반환: (line, error_message)"""
    with serial_lock:
        ser.reset_input_buffer()  # 이전 ACK 등 잔여 데이터 제거
        ser.write(b"GET_SENSOR\n")
        deadline = time.time() + READ_TIMEOUT
        while time.time() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='ignore').strip()
            if line.startswith("TEMP:"):
                return line, None
            if line.startswith("ERROR"):
                return None, line
            # ACK 등 그 외 줄은 무시하고 계속 읽는다
    return None, "센서 응답 시간 초과"


def send_motor(cmd: bytes):
    """모터 명령 전송 후 ACK까지 읽어서 버퍼에 남기지 않는다."""
    with serial_lock:
        ser.reset_input_buffer()
        ser.write(cmd)
        ack = ser.readline().decode('utf-8', errors='ignore').strip()
    print(f">>> 아두이노 응답: {ack}")
    return ack


def parse_sensor_line(line):
    # 형식: TEMP:24.5,HUMI:60.0,RAIN:1,DUST:150
    kv = dict(part.split(':', 1) for part in line.split(','))
    return (
        float(kv["TEMP"]),
        float(kv["HUMI"]),
        int(kv["RAIN"]),   # 0이면 비 옴, 1이면 맑음
        int(kv["DUST"]),
    )


# -----------------------------------------------------------------
# 4. 창문 구동 (모터 정지 포함)
# -----------------------------------------------------------------
def move_window(target):
    """target: 'OPEN' 또는 'CLOSED'. 상태가 바뀔 때만 명령을 보낸다.
    시작 직후(UNKNOWN)에는 어떤 target이든 명령이 전송된다."""
    global move_end_time
    with state_lock:
        if system_data["window_state"] == target:
            return False  # 이미 그 방향 (동작 중이거나 완료) -> 중복 전송 안 함

        if ser:
            try:
                send_motor(b"WINDOW_OPEN\n" if target == "OPEN" else b"WINDOW_CLOSE\n")
            except Exception as e:
                print(f"모터 명령 전송 실패: {e}")
                return False

        system_data["window_state"] = target
        system_data["moving"] = True
        move_end_time = time.time() + MOVE_SECONDS
        print(f">>> 창문 {target} 동작 시작 ({MOVE_SECONDS}초 후 정지)")
        return True


def check_motor_stop():
    """동작 시간이 지나면 WINDOW_STOP 전송."""
    with state_lock:
        if system_data["moving"] and time.time() >= move_end_time:
            if ser:
                try:
                    send_motor(b"WINDOW_STOP\n")
                except Exception as e:
                    print(f"정지 명령 전송 실패: {e}")
                    return  # 다음 루프에서 재시도
            system_data["moving"] = False
            print(">>> 모터 정지")


# -----------------------------------------------------------------
# 5. 센서 갱신 + 자동 제어
# -----------------------------------------------------------------
def refresh_fault():
    """장애 원인들을 합쳐 fault_alert / fault_message 갱신."""
    msgs = [m for m in fault_reasons.values() if m]
    system_data["fault_alert"] = 1 if msgs else 0
    system_data["fault_message"] = " / ".join(msgs)


def comm_fault():
    return fault_reasons["comm"] is not None


def update_sensor():
    """센서를 읽어 system_data 갱신. 통신에 성공하면 True.
    DHT만 실패한 경우에도 비/미세먼지는 정상 갱신하고 True를 반환한다."""
    global comm_fail_count, dht_fail_count

    if not ser:
        fault_reasons["comm"] = "아두이노 미연결 (가상 모드)"
        refresh_fault()
        return False

    # --- 통신 및 파싱 ---
    try:
        line, err = read_sensor_line()
        if line is None:
            raise RuntimeError(err)
        temp, humi, rain, dust_raw = parse_sensor_line(line)
    except Exception as e:
        comm_fail_count += 1
        print(f"센서 읽기 실패 ({comm_fail_count}/{FAIL_LIMIT}): {e}")
        if comm_fail_count >= FAIL_LIMIT:
            fault_reasons["comm"] = f"통신 오류: {e}"
        refresh_fault()
        return False

    comm_fail_count = 0
    fault_reasons["comm"] = None

    # --- 비/미세먼지는 DHT와 무관하게 항상 갱신 ---
    system_data["dust_raw"] = dust_raw
    system_data["dust"] = raw_to_ugm3(dust_raw)
    system_data["rain_detected"] = (rain == 0)

    # --- 온습도(DHT)는 실패 시 이전 값을 유지하고 연속 실패 시에만 알림 ---
    if temp <= DHT_ERROR_THRESHOLD or humi <= DHT_ERROR_THRESHOLD:
        dht_fail_count += 1
        print(f"DHT 읽기 실패 ({dht_fail_count}/{FAIL_LIMIT})")
        if dht_fail_count >= FAIL_LIMIT:
            fault_reasons["dht"] = "온습도 센서(DHT) 오류 - 비/미세먼지 기준으로만 동작 중"
    else:
        dht_fail_count = 0
        fault_reasons["dht"] = None
        system_data["indoor_temp"] = temp
        system_data["indoor_hum"] = humi

    refresh_fault()
    return True


def auto_control():
    if comm_fault():
        # 통신 장애: 오래된 데이터로 판단하지 않고 안전하게 닫는다
        if FAILSAFE_CLOSE_ON_FAULT:
            move_window("CLOSED")
        return

    if system_data["rain_detected"]:
        move_window("CLOSED")       # 비: 최우선 방어
    elif system_data["dust"] > system_data["dust_threshold"]:
        move_window("CLOSED")       # 미세먼지 초과
    else:
        move_window("OPEN")


def hardware_control_loop():
    last_sensor = 0.0
    while True:
        check_motor_stop()

        now = time.time()
        if now - last_sensor >= SENSOR_INTERVAL:
            last_sensor = now
            ok = update_sensor()
            # 최신 데이터가 있거나, 통신 장애가 확정된 경우(페일세이프)에만 자동 제어
            if system_data["mode"] == "AUTO" and (ok or comm_fault()):
                auto_control()

        time.sleep(0.2)  # 모터 정지 타이밍을 정확히 잡기 위해 짧게 순환


# -----------------------------------------------------------------
# 6. Flask 라우트
# -----------------------------------------------------------------
@app.route('/')
def home():
    # 같은 폴더의 dashboard.html 제공
    return send_from_directory(BASE_DIR, 'dashboard.html')


@app.route('/status', methods=['GET'])
def get_status():
    return jsonify(system_data)


@app.route('/command', methods=['POST'])
def receive_command():
    data = request.get_json(silent=True) or {}
    print(f">>> 웹에서 수신된 명령: {data}")

    if "cmd" in data:
        cmd = data["cmd"]
        if cmd == "AUTO":
            system_data["mode"] = "AUTO"
            print(">>> 시스템이 '자동 모드'로 전환되었습니다.")
        elif cmd in ("OPEN", "CLOSE"):
            system_data["mode"] = "MANUAL"
            move_window("OPEN" if cmd == "OPEN" else "CLOSED")
            print(f">>> 수동 개입: 창문 {cmd}")
        else:
            return jsonify({"status": "error", "message": "알 수 없는 명령"}), 400

    if "set_dust" in data:
        try:
            value = int(data["set_dust"])
        except (TypeError, ValueError):
            return jsonify({"status": "error", "message": "잘못된 임계치"}), 400
        system_data["dust_threshold"] = value
        print(f">>> 미세먼지 허용 기준치가 {value} μg/m³ 로 업데이트 됨.")

    return jsonify({"status": "success"})


if __name__ == '__main__':
    # debug 리로더가 프로세스를 두 번 띄워 스레드/시리얼 포트가 중복되는 것을 방지
    threading.Thread(target=hardware_control_loop, daemon=True).start()
    print("통신 서버 가동 시작... (http://<라즈베리파이IP>:5000 접속)")
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
