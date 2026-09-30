from flask import Flask, jsonify, render_template_string
from flask_cors import CORS
from flask import request  # <-- 명령(POST)을 받기 위해 이 줄이 꼭 필요했어!

app = Flask(__name__)
CORS(app)

mock_data = {
    "indoor_temp": 24.5,
    "indoor_hum": 48.0,
    "outdoor_temp": 18.2,
    "outdoor_hum": 62.0,
    "dust": 40,
    "rain_detected": False,
    "window_state": "CLOSED",
    "fault_alert": 0
}

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
        <p>현재 창문 상태: <strong id="stateDisplay" style="font-size: 20px;">알 수 없음</strong></p>
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
        const SERVER_URL = "http://127.0.0.1:5000";

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
    return jsonify(mock_data)

# 👉 이 부분이 빠져있어서 에러가 났던 것임! 명령어 수신 길목 추가
@app.route('/command', methods=['POST'])
def receive_command():
    data = request.json
    print("==================================")
    print("웹에서 날아온 통신 데이터:", data)
    print("==================================")

    if "cmd" in data:
        mock_data["window_state"] = data["cmd"]
    if "set_dust" in data:
        print(f">>> 펌웨어 업데이트: 미세먼지 임계치 {data['set_dust']} 설정됨!")

    return jsonify({"status": "success"})

if __name__ == '__main__':
    print("통신 서버 가동 시작... (http://127.0.0.1:5000 접속)")
    app.run(host='0.0.0.0', port=5000, debug=True)