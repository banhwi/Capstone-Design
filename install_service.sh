#!/usr/bin/env bash
# 스마트 창문 서버를 systemd 서비스로 등록합니다. (부팅 시 자동 시작, 죽으면 자동 재시작)
#
# 사용법: server.py 가 있는 폴더에서 실행
#     bash install_service.sh            # 등록하고 시작
#     bash install_service.sh --dry-run  # 만들어질 서비스 파일 내용만 미리 보기
#
# 폴더 위치와 사용자 이름은 자동으로 알아냅니다. (직접 경로를 고칠 필요 없음)
set -euo pipefail

# ---- 필요하면 여기 값만 고치세요 ----
SERIAL_PORT="/dev/ttyACM0"   # 아두이노 포트 (/dev/serial/by-id/... 경로를 쓰면 포트 번호가 바뀌어도 안전)
MOVE_SECONDS="18"            # 창문이 끝까지 열리/닫히는 실측 시간(초). 아두이노 MAX_RUN_MS(25초)보다 5초 이상 짧게
LAT="37.2636"                # 설치 장소 위도 (기상청 격자 변환용, 기본값은 수원)
LON="127.0286"               # 설치 장소 경도
# 기상청 API 키는 server.py 와 같은 폴더의 kma_key.txt 에 한 줄로 저장하세요 (서비스 파일에 넣지 않음)
# -------------------------------------

SERVICE_NAME="smart-window"
UNIT_PATH="/etc/systemd/system/${SERVICE_NAME}.service"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DRY_RUN=0
[[ "${1:-}" == "--dry-run" ]] && DRY_RUN=1

if [[ ! -f "$DIR/server.py" || ! -f "$DIR/dashboard.html" ]]; then
  echo "오류: $DIR 안에 server.py 와 dashboard.html 이 함께 있어야 합니다." >&2
  exit 1
fi

# 이 폴더에 가상환경(venv)이 있으면 그것을, 없으면 시스템 python3 를 사용
if [[ -x "$DIR/venv/bin/python" ]]; then
  PYTHON="$DIR/venv/bin/python"
else
  PYTHON="$(command -v python3)"
fi

# root가 아니면 sudo로 다시 실행 (원래 사용자 이름은 SUDO_USER 로 전달됨)
if [[ $DRY_RUN -eq 0 && $EUID -ne 0 ]]; then
  exec sudo bash "$0" "$@"
fi

RUN_USER="${SUDO_USER:-$(id -un)}"
if [[ "$RUN_USER" == "root" ]]; then
  echo "경고: root 계정으로 실행하도록 등록됩니다. 일반 사용자로 실행해 보세요." >&2
fi

print_unit() {
cat <<UNIT_EOF
[Unit]
Description=Smart Window Controller
After=network.target

[Service]
Type=simple
User=${RUN_USER}
SupplementaryGroups=dialout
WorkingDirectory=${DIR}
ExecStart=${PYTHON} ${DIR}/server.py
Environment=SW_SERIAL_PORT=${SERIAL_PORT}
Environment=SW_MOVE_SECONDS=${MOVE_SECONDS}
Environment=SW_LAT=${LAT}
Environment=SW_LON=${LON}
Environment=PYTHONUNBUFFERED=1
Restart=always
RestartSec=5
# 종료 시 모터 정지 확인(shutdown_stop)을 할 시간을 준다
KillSignal=SIGTERM
TimeoutStopSec=20

[Install]
WantedBy=multi-user.target
UNIT_EOF
}

if [[ $DRY_RUN -eq 1 ]]; then
  echo "# 아래 내용으로 ${UNIT_PATH} 가 만들어집니다"
  print_unit
  exit 0
fi

print_unit > "$UNIT_PATH"
systemctl daemon-reload
systemctl enable "$SERVICE_NAME"
systemctl restart "$SERVICE_NAME"
sleep 3
systemctl --no-pager --full status "$SERVICE_NAME" || true

echo
echo "등록 완료. 로그 확인:  journalctl -u ${SERVICE_NAME} -f"
echo "중지:  sudo systemctl stop ${SERVICE_NAME}    재시작:  sudo systemctl restart ${SERVICE_NAME}"
