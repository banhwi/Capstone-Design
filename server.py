"""
스마트 창문 제어 서버 (실행용)

- 라즈베리파이에서 이 파일 하나만 실행합니다. (라즈베리파이 단독 제어 스크립트와 동시 실행 금지)
- 아두이노와 시리얼로 통신하며 센서값 수집 / 자동 개폐 / 수동 제어 / 웹 대시보드를 제공합니다.
- 운영 서버(waitress), 로그 파일, 시리얼 자동 재연결, 설정 저장, /health 점검을 포함합니다.

환경변수(선택):
    SW_SERIAL_PORT   시리얼 포트 (기본 /dev/ttyACM0, 윈도우는 COM3 등)
    SW_HTTP_HOST     바인드 주소 (기본 0.0.0.0)
    SW_HTTP_PORT     웹 포트 (기본 5000)
    SW_MOVE_SECONDS  창문이 끝까지 열리/닫히는 시간(초, 기본 5) - 반드시 실측해서 설정
    SW_LOG_DIR       로그 폴더 (기본 ./logs)
    SW_SIMULATE      1이면 아두이노 없이 동작 (개발/화면 확인용)
"""
import atexit
import json
import logging
import os
import signal
import sys
import threading
import time
from logging.handlers import RotatingFileHandler

from flask import Flask, jsonify, request, send_from_directory
import serial

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


# -----------------------------------------------------------------
# 0. 설정값
# -----------------------------------------------------------------
def _env_float(name, default):
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return float(default)


SERIAL_PORT = os.environ.get("SW_SERIAL_PORT", "/dev/ttyACM0")
BAUD_RATE = 9600
HTTP_HOST = os.environ.get("SW_HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(_env_float("SW_HTTP_PORT", 5000))
SIMULATE = os.environ.get("SW_SIMULATE", "0") == "1"
LOG_DIR = os.environ.get("SW_LOG_DIR", os.path.join(BASE_DIR, "logs"))
SETTINGS_FILE = os.path.join(BASE_DIR, "settings.json")

SENSOR_INTERVAL = 3.0        # 정상일 때 센서 요청 주기(초). DHT22는 최소 2초 간격 필요
RETRY_INTERVAL = 2.1         # 센서 읽기 실패 시 재시도 간격(초). DHT22 최소 간격 이상으로 유지
READ_TIMEOUT = 2.0           # GET_SENSOR 응답 대기 한도(초)
FAIL_LIMIT = 3               # 연속 이 횟수만큼 실패하면 장애 알림
RECONNECT_INTERVAL = 5.0     # 시리얼 재연결 시도 최소 간격(초)

MOVE_SECONDS = _env_float("SW_MOVE_SECONDS", 5.0)  # 아두이노 MAX_RUN_MS(8000ms)보다 반드시 짧아야 함
REVERSE_DELAY = 0.3          # 이동 중 반대 방향으로 바꿀 때 정지 후 대기 시간(초)
MOTOR_RETRIES = 3            # 모터 명령 ACK를 못 받았을 때 재전송 횟수
SHUTDOWN_JOIN_TIMEOUT = 5.0  # 종료 시 제어 스레드가 끝나기를 기다리는 최대 시간(초)

FAILSAFE_CLOSE_ON_FAULT = True  # 자동 모드에서 '통신 장애' 시 안전을 위해 창문 닫기

DHT_ERROR_THRESHOLD = -990   # 아두이노가 DHT 실패 시 보내는 -999 판별 기준
DUST_MIN, DUST_MAX = 10, 100  # 대시보드 슬라이더 범위와 동일

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
# 로깅 (콘솔/systemd 저널 + 회전 로그 파일)
# -----------------------------------------------------------------
log = logging.getLogger("smart_window")
log.setLevel(logging.INFO)
if not log.handlers:
    _fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    _stream = logging.StreamHandler()
    _stream.setFormatter(_fmt)
    log.addHandler(_stream)
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        _file = RotatingFileHandler(os.path.join(LOG_DIR, "smart_window.log"),
                                    maxBytes=1_000_000, backupCount=3, encoding="utf-8")
        _file.setFormatter(_fmt)
        log.addHandler(_file)
    except OSError as _e:
        log.warning("로그 파일을 만들 수 없어 콘솔에만 기록합니다: %s", _e)
for _name in ("waitress", "werkzeug"):
    logging.getLogger(_name).setLevel(logging.WARNING)  # 1초마다 오는 /status 요청 로그 억제

app = Flask(__name__)

# -----------------------------------------------------------------
# 1. 공유 상태
# -----------------------------------------------------------------
ser = None  # 연결되면 serial.Serial 객체

# 시리얼 포트 접근을 직렬화하는 락 (스레드 안전)
serial_lock = threading.Lock()
# 창문 상태/모터 타이머 접근용 락 (항상 state_lock -> serial_lock 순서로 획득)
state_lock = threading.RLock()

# 종료 절차 시작을 알리는 이벤트: 켜지면 제어 루프가 끝나고 새 모터 명령이 거부된다
stop_event = threading.Event()
control_thread = None

system_data = {
    "indoor_temp": 0.0,
    "indoor_hum": 0.0,
    "outdoor_temp": 18.2,   # 실외 센서가 없다면 API나 고정값 사용
    "outdoor_hum": 62.0,
    "dust": 0,              # μg/m³ (변환값)
    "dust_raw": 0,          # ADC 원시값 (보정용)
    "rain_detected": False,
    "window_state": "UNKNOWN",  # 시작/재연결 직후에는 실제 위치를 모르므로 첫 판단 때 반드시 명령을 보냄
    "moving": False,        # 모터 동작 중 여부
    "fault_alert": 0,
    "fault_message": "",
    "mode": "AUTO",
    "dust_threshold": 35,
}

move_end_time = 0.0
last_sensor_ok = 0.0
last_reconnect = 0.0

# 장애 원인을 종류별로 관리: comm = 통신 장애, dht = 온습도 센서 장애, motor = 모터 명령 장애
fault_reasons = {"comm": None, "dht": None, "motor": None}
comm_fail_count = 0
dht_fail_count = 0


# -----------------------------------------------------------------
# 설정 저장 (재시작해도 미세먼지 임계치 유지)
# -----------------------------------------------------------------
def load_settings():
    try:
        with open(SETTINGS_FILE, encoding="utf-8") as f:
            value = int(json.load(f).get("dust_threshold", system_data["dust_threshold"]))
        if DUST_MIN <= value <= DUST_MAX:
            system_data["dust_threshold"] = value
            log.info("저장된 미세먼지 임계치 불러옴: %s μg/m³", value)
    except FileNotFoundError:
        pass
    except Exception as e:
        log.warning("설정 파일을 읽지 못해 기본값을 사용합니다: %s", e)


def save_settings():
    tmp = SETTINGS_FILE + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"dust_threshold": system_data["dust_threshold"]}, f)
        os.replace(tmp, SETTINGS_FILE)  # 원자적 교체: 전원이 나가도 파일이 깨지지 않음
    except OSError as e:
        log.warning("설정 저장 실패: %s", e)


# -----------------------------------------------------------------
# 2. 시리얼 연결 / 입출력 (모든 시리얼 접근은 여기를 거친다)
# -----------------------------------------------------------------
def open_serial():
    """리눅스(POSIX)에서는 독점(exclusive) 모드로 열어 다른 프로그램의 동시 접근을 막는다.
    (pyserial 3.3 이상 필요. 지원하지 않으면 일반 모드로 연다.)"""
    kwargs = {"exclusive": True} if os.name == "posix" else {}
    try:
        return serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1, **kwargs)
    except TypeError:
        return serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


def connect_serial():
    """시리얼을 (다시) 연결한다. 성공하면 True.
    연결 직후에는 창문 위치를 알 수 없으므로 상태를 UNKNOWN으로 되돌리고 모터 정지를 확인한다."""
    global ser
    with serial_lock:
        old, ser = ser, None
        if old is not None:
            try:
                old.close()
            except Exception:
                pass
        try:
            new = open_serial()
            time.sleep(2)  # 아두이노 부팅(자동 리셋) 대기. 이 동안 다른 시리얼 접근은 대기한다
            new.reset_input_buffer()
            ser = new
        except Exception as e:
            log.warning("아두이노 연결 실패 (%s): %s", SERIAL_PORT, e)
            return False

    with state_lock:
        system_data["window_state"] = "UNKNOWN"
        system_data["moving"] = False
    log.info("아두이노 시리얼 연결 성공 (%s)", SERIAL_PORT)
    if not send_motor_checked(b"WINDOW_STOP\n"):
        log.warning("연결 직후 모터 정지 확인 실패")
    return True


def maybe_reconnect():
    """너무 자주 시도하지 않도록 간격을 두고 재연결."""
    global last_reconnect
    if time.time() - last_reconnect < RECONNECT_INTERVAL:
        return False
    last_reconnect = time.time()
    return connect_serial()


def read_sensor_line():
    """버퍼를 비우고 요청한 뒤, TEMP: 로 시작하는 줄이 나올 때까지 읽는다.
    반환: (line, error_message)"""
    with serial_lock:
        if ser is None:
            raise serial.SerialException("시리얼 미연결")
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
    """모터 명령 한 번 전송하고 응답 한 줄을 돌려준다."""
    with serial_lock:
        if ser is None:
            raise serial.SerialException("시리얼 미연결")
        ser.reset_input_buffer()
        ser.write(cmd)
        return ser.readline().decode('utf-8', errors='ignore').strip()


def send_motor_checked(cmd: bytes):
    """모터 명령을 보내고, 기대한 ACK(예: WINDOW_OPEN_SUCCESS)가 올 때까지 재시도.
    성공하면 True, 끝내 실패하면 장애를 기록하고 False."""
    expected = cmd.decode().strip() + "_SUCCESS"
    for attempt in range(1, MOTOR_RETRIES + 1):
        try:
            ack = send_motor(cmd)
        except Exception as e:
            ack = ""
            log.warning("모터 명령 전송 예외: %s", e)
        if expected in ack:
            log.info("아두이노 응답: %s", ack)
            if fault_reasons["motor"]:
                fault_reasons["motor"] = None
                refresh_fault()
            return True
        log.warning("모터 명령 응답 이상 (%d/%d): '%s'", attempt, MOTOR_RETRIES, ack)
        time.sleep(0.1)

    fault_reasons["motor"] = "모터 명령 응답 없음 - 배선/시리얼 연결 확인 필요"
    refresh_fault()
    return False


_shutdown_lock = threading.Lock()
_shutdown_done = False


def shutdown_stop():
    """서버 종료 시: 제어 루프를 먼저 멈춘 뒤, 정지 명령을 ACK까지 확인하며 보낸다."""
    global _shutdown_done
    with _shutdown_lock:
        if _shutdown_done:
            return
        _shutdown_done = True

    stop_event.set()  # 새 모터 명령 차단 + 제어 루프 종료 신호
    if SIMULATE or ser is None:
        return

    t = control_thread
    if t is not None and t.is_alive() and t is not threading.current_thread():
        t.join(timeout=SHUTDOWN_JOIN_TIMEOUT)

    if t is None or not t.is_alive():
        # 시리얼을 쓰는 다른 스레드가 없으므로 응답을 안전하게 확인할 수 있다
        ok = send_motor_checked(b"WINDOW_STOP\n")
        log.info("종료 시 모터 정지 %s", "확인 완료" if ok else "확인 실패")
        return

    # 제어 스레드가 제때 끝나지 않은 경우: 안전이 우선이므로 정지 명령은 보내되, 응답은 확인하지 못한다
    log.warning("제어 스레드가 종료되지 않아 정지 명령을 응답 확인 없이 전송합니다")
    got = serial_lock.acquire(timeout=2)
    try:
        ser.write(b"WINDOW_STOP\n")
        ser.flush()
    except Exception as e:
        log.error("비상 정지 전송 실패: %s", e)
    finally:
        if got:
            serial_lock.release()


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
# 3. 창문 구동 (모터 정지 포함)
# -----------------------------------------------------------------
def move_window(target):
    """target: 'OPEN' 또는 'CLOSED'. 상태가 바뀔 때만 명령을 보낸다.
    시작/재연결 직후(UNKNOWN)에는 어떤 target이든 명령이 전송된다."""
    global move_end_time
    with state_lock:
        if stop_event.is_set():
            return False  # 종료 중에는 새 모터 명령을 내지 않는다
        if system_data["window_state"] == target:
            return False  # 이미 그 방향 (동작 중이거나 완료) -> 중복 전송 안 함

        if not SIMULATE:
            if ser is None:
                log.warning("아두이노 미연결 - 창문을 움직일 수 없습니다 (%s)", target)
                return False

            # 이동 중 반대 방향으로 바꿀 땐 먼저 정지하고 잠시 기다린다 (급반전 방지)
            stopped_first = False
            if system_data["moving"]:
                send_motor_checked(b"WINDOW_STOP\n")
                time.sleep(REVERSE_DELAY)
                stopped_first = True

            cmd = b"WINDOW_OPEN\n" if target == "OPEN" else b"WINDOW_CLOSE\n"
            if not send_motor_checked(cmd):
                if stopped_first:
                    # 멈춘 뒤 새 명령이 실패 -> 위치를 알 수 없으니 다음 판단 때 다시 시도
                    system_data["window_state"] = "UNKNOWN"
                    system_data["moving"] = False
                return False

        system_data["window_state"] = target
        system_data["moving"] = True
        move_end_time = time.time() + MOVE_SECONDS
        log.info("창문 %s 동작 시작 (%.1f초 후 정지)", target, MOVE_SECONDS)
        return True


def check_motor_stop():
    """동작 시간이 지나면 WINDOW_STOP 전송. ACK를 못 받으면 다음 루프에서 재시도."""
    with state_lock:
        if not (system_data["moving"] and time.time() >= move_end_time):
            return
        if not SIMULATE:
            if ser is None:
                # 연결이 끊김: 보낼 수 없다. 아두이노의 MAX_RUN_MS가 모터를 멈추고, 재연결 시 상태를 초기화한다
                system_data["moving"] = False
                return
            if not send_motor_checked(b"WINDOW_STOP\n"):
                return  # moving 유지 -> 다음 루프에서 재시도 (아두이노 MAX_RUN_MS가 최후 안전장치)
        system_data["moving"] = False
        log.info("모터 정지")


# -----------------------------------------------------------------
# 4. 센서 갱신 + 자동 제어
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
    global comm_fail_count, dht_fail_count, last_sensor_ok

    if SIMULATE:
        return True

    if ser is None:
        fault_reasons["comm"] = "아두이노 미연결 - 재연결 시도 중"
        refresh_fault()
        if not maybe_reconnect():
            return False
        # 재연결에 성공했으면 아래에서 바로 읽는다

    # --- 통신 및 파싱 ---
    try:
        line, err = read_sensor_line()
        if line is None:
            raise RuntimeError(err)
        temp, humi, rain, dust_raw = parse_sensor_line(line)
    except Exception as e:
        comm_fail_count += 1
        log.warning("센서 읽기 실패 (%d/%d): %s", comm_fail_count, FAIL_LIMIT, e)
        if comm_fail_count >= FAIL_LIMIT:
            fault_reasons["comm"] = f"통신 오류: {e}"
            refresh_fault()  # 재연결은 수 초 걸리므로 알림을 먼저 반영
            if maybe_reconnect():  # 케이블 재연결/아두이노 리셋 복구 시도
                comm_fail_count = 0
        refresh_fault()
        return False

    comm_fail_count = 0
    fault_reasons["comm"] = None
    last_sensor_ok = time.time()

    # --- 비/미세먼지는 DHT와 무관하게 항상 갱신 ---
    system_data["dust_raw"] = dust_raw
    system_data["dust"] = raw_to_ugm3(dust_raw)
    system_data["rain_detected"] = (rain == 0)

    # --- 온습도(DHT)는 실패 시 이전 값을 유지하고 연속 실패 시에만 알림 ---
    if temp <= DHT_ERROR_THRESHOLD or humi <= DHT_ERROR_THRESHOLD:
        dht_fail_count += 1
        log.warning("DHT 읽기 실패 (%d/%d)", dht_fail_count, FAIL_LIMIT)
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
    next_sensor = 0.0
    while not stop_event.is_set():
        try:
            check_motor_stop()

            if not stop_event.is_set() and time.time() >= next_sensor:
                ok = update_sensor()
                # 실패하면 정상 주기를 기다리지 않고 더 빨리 재시도 (일시적 오류 복구)
                next_sensor = time.time() + (SENSOR_INTERVAL if ok else RETRY_INTERVAL)
                # 최신 데이터가 있거나, 통신 장애가 확정된 경우(페일세이프)에만 자동 제어
                if system_data["mode"] == "AUTO" and (ok or comm_fault()):
                    auto_control()
        except Exception:
            # 예상 못 한 오류로 제어 스레드가 조용히 죽지 않도록 기록하고 계속 돈다
            log.exception("제어 루프 오류")
            next_sensor = time.time() + RETRY_INTERVAL

        stop_event.wait(0.2)  # 모터 정지 타이밍을 정확히 잡기 위해 짧게 순환 (종료 신호 시 즉시 깨어남)


# -----------------------------------------------------------------
# 5. Flask 라우트
# -----------------------------------------------------------------
@app.route('/')
def home():
    # 같은 폴더의 dashboard.html 제공
    return send_from_directory(BASE_DIR, 'dashboard.html')


@app.route('/status', methods=['GET'])
def get_status():
    return jsonify(system_data)


@app.route('/health', methods=['GET'])
def health():
    """점검용: 제어 스레드가 살아 있고 센서를 최근에 읽었는지 확인."""
    alive = control_thread is not None and control_thread.is_alive()
    age = round(time.time() - last_sensor_ok, 1) if last_sensor_ok else None
    body = {
        "control_thread_alive": alive,
        "serial_connected": SIMULATE or ser is not None,
        "simulate": SIMULATE,
        "last_sensor_age_sec": age,
        "fault_message": system_data["fault_message"],
    }
    return jsonify(body), (200 if alive else 503)


@app.route('/command', methods=['POST'])
def receive_command():
    data = request.get_json(silent=True) or {}
    log.info("웹에서 수신된 명령: %s", data)

    if "cmd" in data:
        cmd = data["cmd"]
        if cmd == "AUTO":
            system_data["mode"] = "AUTO"
            log.info("자동 모드로 전환")
        elif cmd in ("OPEN", "CLOSE"):
            system_data["mode"] = "MANUAL"
            move_window("OPEN" if cmd == "OPEN" else "CLOSED")
            log.info("수동 개입: 창문 %s", cmd)
        else:
            return jsonify({"status": "error", "message": "알 수 없는 명령"}), 400

    if "set_dust" in data:
        try:
            value = int(data["set_dust"])
        except (TypeError, ValueError):
            return jsonify({"status": "error", "message": "잘못된 임계치"}), 400
        if not (DUST_MIN <= value <= DUST_MAX):
            return jsonify({"status": "error",
                            "message": f"임계치는 {DUST_MIN}~{DUST_MAX} 사이여야 합니다"}), 400
        system_data["dust_threshold"] = value
        save_settings()
        log.info("미세먼지 허용 기준치가 %d μg/m³ 로 업데이트됨", value)

    return jsonify({"status": "success"})


# -----------------------------------------------------------------
# 6. 실행
# -----------------------------------------------------------------
def _on_sigterm(signum, frame):
    # systemd 등이 종료 신호(SIGTERM)를 보내면 정상 종료 -> atexit의 shutdown_stop이 실행됨
    sys.exit(0)


def main():
    global control_thread

    load_settings()
    atexit.register(shutdown_stop)
    signal.signal(signal.SIGTERM, _on_sigterm)

    if SIMULATE:
        log.warning("시뮬레이션 모드: 아두이노 없이 동작합니다 (실제 창문은 움직이지 않음)")
    else:
        # 시작 시 모터가 돌고 있지 않도록 정지를 확인한다. 실패해도 제어 루프가 계속 재연결을 시도한다
        connect_serial()

    control_thread = threading.Thread(target=hardware_control_loop, name="control", daemon=True)
    control_thread.start()

    log.info("서버 시작: http://%s:%d", HTTP_HOST, HTTP_PORT)
    try:
        from waitress import serve
    except ImportError:
        serve = None

    if serve is not None:
        serve(app, host=HTTP_HOST, port=HTTP_PORT, threads=4)
    else:
        log.warning("waitress가 설치되어 있지 않아 Flask 개발 서버로 실행합니다 "
                    "(pip install waitress 권장)")
        app.run(host=HTTP_HOST, port=HTTP_PORT, debug=False, use_reloader=False, threaded=True)


if __name__ == '__main__':
    main()
