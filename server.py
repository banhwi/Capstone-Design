from flask import Flask, jsonify, render_template_string, request
from flask_cors import CORS
import serial
import time
import threading

app = Flask(__name__)
CORS(app)

# -----------------------------------------------------------------
# 1. 하드웨어(아두이노) 시리얼 통신 설정 (구동팀 환경)
# -----------------------------------------------------------------
PORT = '/dev/ttyACM0'  # 윈도우에서 테스트할 땐 'COM3' 등으로 변경
BAUD_RATE = 9600
try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=2)
    time.sleep(2) # 아두이노 부팅 대기
    print(">>> 아두이노 시리얼 통신 연결 성공!")
except Exception as e:
    ser = None
    print(f">>> [경고] 아두이노 연결 실패: {e} (가상 모드로 동작합니다)")

# -----------------------------------------------------------------
# 2. 시스템 전역 상태 변수 (진짜 데이터 보관소)
# -----------------------------------------------------------------
system_data = {
    "indoor_temp": 0.0,
    "indoor_hum": 0.0,
    "outdoor_temp": 18.2, # 실외 센서가 없다면 API나 고정값 사용
    "outdoor_hum": 62.0,
    "dust": 0,
    "rain_detected": False,
    "window_state": "CLOSED",
    "fault_alert": 0,
    "mode": "AUTO",       # 현재 제어 모드 (AUTO or MANUAL)
    "dust_threshold": 35  # 미세먼지 허용 기준치
}

# -----------------------------------------------------------------
# 3. 백그라운드 하드웨어 제어 루프 (구동팀 로직 이식)
# -----------------------------------------------------------------
def hardware_control_loop():
    global system_data
    while True:
        if ser:
            try:
                # 1) 아두이노에 센서 데이터 요청
                ser.write(b"GET_SENSOR\n")
                time.sleep(0.1)
                
                # 2) 데이터 수신 및 파싱
                if ser.in_waiting > 0:
                    response = ser.readline().decode('utf-8').strip()
                    
                    if "TEMP:" in response:
                        # 형식 예시: TEMP:24.5,HUMI:60.0,RAIN:1,DUST:150
                        parts = response.split(',')
                        temp = float(parts[0].split(':')[1])
                        humi = float(parts[1].split(':')[1])
                        rain = int(parts[2].split(':')[1]) # 0이면 비 옴, 1이면 맑음
                        dust = int(parts[3].split(':')[1])
                        
                        # 웹 갱신용 데이터 덮어쓰기
                        system_data["indoor_temp"] = temp
                        system_data["indoor_hum"] = humi
                        system_data["dust"] = dust
                        system_data["rain_detected"] = (rain == 0)
                        
                        # 3) 자동 모드(AUTO)일 때만 구동팀의 엣지 제어 로직 개입
                        if system_data["mode"] == "AUTO":
                            if rain == 0:  # 비가 오면 즉시 닫음 (최우선 방어)
                                ser.write(b"WINDOW_CLOSE\n")
                                system_data["window_state"] = "CLOSED"
                            elif dust > system_data["dust_threshold"]: # 설정된 미세먼지 임계치 초과 시 닫음
                                ser.write(b"WINDOW_CLOSE\n")
                                system_data["window_state"] = "CLOSED"
                            else: # 안전한 상태면 염
                                ser.write(b"WINDOW_OPEN\n")
                                system_data["window_state"] = "OPEN"
                                
            except Exception as e:
                print(f"하드웨어 스레드 에러: {e}")
        
        # 5초마다 갱신 (모터가 움직일 시간을 충분히 줌)
        time.sleep(5)

# Flask 웹 서버 시작 전 백그라운드 스레드 실행
threading.Thread(target=hardware_control_loop, daemon=True).start()


# -----------------------------------------------------------------
# 4. 프론트엔드 HTML 템플릿 (네가 수정한 코드 그대로 적용)
# -----------------------------------------------------------------
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>스마트 창문 제어</title>
    <style>
        body { font-family: sans-serif; text-align: center; padding: 50px; }
        .card { border: 1px solid #ccc; padding: 20px; border-radius: 10px; margin-bottom: 20px; }
        .sensor-row { margin: 10px 0; font-size: 18px; }
        button { padding: 10px 20px; margin: 5px; cursor: pointer; font-size: 16px; }
        #alertBox { display: none; color: white; background-color: red; padding: 10px; font-weight: bold; margin-bottom: 20px; }
        .rain-warning { color: blue; font-weight: bold; } 
        input[type=range] { width: 60%; margin: 15px 0; }
    </style>
</head>
<body>
    <h2>스마트 홈 대시보드</h2>
    
    <div id="alertBox">⚠️ 긴급: 과부하 위험 감지! 시스템을 보호합니다.</div>

    <div class="card">
        <h3>실내외 온습도 및 환경 수치</h3>
        <div class="sensor-row">🏠 <strong>[실내]</strong> 온도: <span id="inTempDisplay">--</span> °C | 습도: <span id="inHumDisplay">--</span> %</div>
        <div class="sensor-row">☁️ <strong>[실외]</strong> 온도: <span id="outTempDisplay">--</span> °C | 습도: <span id="outHumDisplay">--</span> %</div>
        <div class="sensor-row">🌬️ 미세먼지: <span id="dustDisplay">--</span> μg/m³</div>
        <div class="sensor-row">🌧️ 강우 상태: <span id="rainDisplay">--</span></div>
        <br>
        <p>현재 창문 상태: <strong id="stateDisplay" style="font-size: 20px;">알 수 없음</strong> 
           <span style="font-size: 14px; color: gray;">(<span id="modeDisplay">AUTO</span> 모드)</span></p>
    </div>

    <div class="card">
        <h3>자동 개폐 임계치 튜닝</h3>
        <p>미세먼지 허용 기준: <strong id="thresholdView" style="color: green;">35</strong> μg/m³</p>
        <input type="range" id="dustSlider" min="10" max="100" step="5" value="35" oninput="updateSliderText(this.value)">
        <br>
        <button onclick="sendThreshold()">설정 적용</button>
    </div>

    <div class="card">
        <h3>수동 제어</h3>
        <button onclick="sendCommand('OPEN')">창문 열기</button>
        <button onclick="sendCommand('CLOSE')">창문 닫기</button>
        <button onclick="sendCommand('AUTO')">자동 모드</button>
    </div>

    <script>
        const SERVER_URL = "http://127.0.0.1:5000"; // 라즈베리파이 IP로 변경 필요

        async function fetchDashboard() {
            try {
                let response = await fetch(SERVER_URL + "/status");
                let data = await response.json();

                document.getElementById("inTempDisplay").innerText = data.indoor_temp;
                document.getElementById("inHumDisplay").innerText = data.indoor_hum;
                document.getElementById("outTempDisplay").innerText = data.outdoor_temp;
                document.getElementById("outHumDisplay").innerText = data.outdoor_hum;
                document.getElementById("dustDisplay").innerText = data.dust;
                
                let rainSpan = document.getElementById("rainDisplay");
                if (data.rain_detected === true) {
                    rainSpan.innerText = "비 감지됨 🌧️";
                    rainSpan.className = "rain-warning";
                } else {
                    rainSpan.innerText = "맑음 ☀️";
                    rainSpan.className = "";
                }

                document.getElementById("stateDisplay").innerText = data.window_state;
                document.getElementById("modeDisplay").innerText = data.mode;

                if (data.fault_alert === 1) {
                    document.getElementById("alertBox").style.display = "block";
                } else {
                    document.getElementById("alertBox").style.display = "none";
                }
            } catch (error) {
                console.error("서버 연결 실패:", error);
            }
        }

        async function sendCommand(cmd) {
            try {
                await fetch(SERVER_URL + "/command", {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ "cmd": cmd })
                });
                fetchDashboard();
            } catch (error) {
                alert("서버 통신 에러!");
            }
        }

        function updateSliderText(val) {
            document.getElementById("thresholdView").innerText = val;
        }

        async function sendThreshold() {
            let targetValue = document.getElementById("dustSlider").value;
            try {
                await fetch(SERVER_URL + "/command", {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ "set_dust": parseInt(targetValue) })
                });
                alert("임계치 " + targetValue + " μg/m³ 설정 완료!");
            } catch (error) {
                alert("서버 통신 에러!");
            }
        }

        setInterval(fetchDashboard, 1000);
        fetchDashboard();
    </script>
</body>
</html>
"""

@app.route('/')
def home():
    return render_template_string(HTML_TEMPLATE)

@app.route('/status', methods=['GET'])
def get_status():
    return jsonify(system_data)

@app.route('/command', methods=['POST'])
def receive_command():
    global system_data
    data = request.json
    print(f">>> 웹에서 수신된 명령: {data}")

    # 창문 제어 및 모드 변경 로직
    if "cmd" in data:
        command_str = data["cmd"]
        
        if command_str == "AUTO":
            system_data["mode"] = "AUTO"
            print(">>> 시스템이 '자동 모드'로 전환되었습니다.")
        else:
            # OPEN이나 CLOSE를 누르면 즉시 수동 모드로 강제 전환
            system_data["mode"] = "MANUAL"
            system_data["window_state"] = command_str
            
            # 아두이노로 물리적 신호 쏘기
            if ser:
                if command_str == "OPEN":
                    ser.write(b"WINDOW_OPEN\n")
                elif command_str == "CLOSE":
                    ser.write(b"WINDOW_CLOSE\n")
            print(f">>> 수동 개입: 창문을 {command_str} 상태로 변경합니다.")

    # 미세먼지 임계치 튜닝 로직
    if "set_dust" in data:
        system_data["dust_threshold"] = data["set_dust"]
        print(f">>> 미세먼지 허용 기준치가 {data['set_dust']} μg/m³ 로 업데이트 됨.")

    return jsonify({"status": "success"})

if __name__ == '__main__':
    print("통신 서버 가동 시작... (http://127.0.0.1:5000 접속)")
    app.run(host='0.0.0.0', port=5000, debug=True)