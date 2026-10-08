#!/usr/bin/env bash
# Coordinate the package CLI, using the AgileX controller launcher layout.
set -Eeuo pipefail

sdk_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
xcore=(uv run --locked --project "$sdk_dir" xcore-sdk-python)
robot_ip="${XCORE_ROBOT_IP:-192.168.2.160}"
local_ip="${XCORE_LOCAL_IP:-192.168.2.100}"
gello_port="${XCORE_GELLO_PORT:-/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0}"
calib="$sdk_dir/config/cr7_calib.json"
server_port=6001
hz=50
max_speed_deg=3
enable_motion=false
assume_yes=false
skip_prepare=false
calibrate_zero=false
prepare_speed=50
prepare_motion_timeout=600
prepare_max_step_deg=180
prepare_pid=""
prepare_gripper_options=()
server_pid=""
client_pid=""
session_dir=""
gripper_options=()

usage() {
    cat <<'EOF'
用法：./start_gello_follow.sh [选项]

默认只读预览；--enable-motion 才进入六轴真机跟随。
  --ip IP              CR7 IP，默认 192.168.2.160
  --local-ip IP        本机有线 IP，默认 192.168.2.100（必须已配置）
  --gello-port PATH    GELLO 串口
  --calib FILE         现场标定 JSON，默认 xcore-sdk-python/config/cr7_calib.json
  --port PORT          本机 ZMQ 端口，默认 6001
  --hz HZ              主臂读取／目标发送频率，默认 50 Hz
  --max-speed-deg V    CR7 跟随关节速度上限，默认 3 °/s
  --enable-motion      启用实际跟随（否则 dry-run）
                       默认先低速对齐到 GELLO 当前姿态，再进入实时跟随
  --skip-prepare       已手动对齐时跳过移动准备，仍校验启动姿态
  --calibrate-zero     首次标定：GELLO 保持六轴零位，CR7 归零后保存偏移
                       已有标定不能覆盖；正常启动不归零、不重新标定
  --prepare-speed V    启动对齐 MoveAbsJ 速度，默认 50 mm/s
  --prepare-motion-timeout S 每段准备运动等待时间，默认 600 s
  --prepare-max-step-deg V   每轴准备运动最大角度差，默认 180°
  --yes                跳过启用运动的交互确认
  --gripper-host HOST  同时跟随外接夹爪；同机服务使用 127.0.0.1
  --gripper-port PORT  夹爪 TCP 端口，默认 5005
  --gripper-id ID      GELLO 扳机 ID，默认 7
  --gripper-open-deg V / --gripper-close-deg V   扳机角度端点，默认 194.8 / 153
  --gripper-open-pos V / --gripper-closed-pos V  实际夹爪行程端点，默认 0 / 255
  --gripper-hz HZ      夹爪更新频率，默认 5 Hz（六轴仍为 50 Hz）
  --gripper-speed V / --gripper-force V         夹爪速度／力度，默认 150 / 0
  --gripper-timeout V / --gripper-stale-timeout V 请求超时／断流超时，默认 0.75 / 1.5 s
  --raw-data-root PATH  启用 raw episode 记录（需 --enable-motion 和 --gripper-host）
  --task TEXT          记录任务描述
  --start-recording    对齐后立即开始记录，否则使用 R/S/D/P/H 单键
  --session-path-file PATH / --record-queue-size N / --record-feedback-max-age S
  -h, --help           显示帮助
EOF
}

fail() { echo "错误：$*" >&2; exit 1; }

stop_group() {
    local task_pid="$1"
    [[ -n "$task_pid" ]] || return 0
    if kill -0 -- "-$task_pid" 2>/dev/null; then
        kill -TERM -- "-$task_pid" 2>/dev/null || true
        for ((attempt=0; attempt<150; attempt++)); do
            kill -0 -- "-$task_pid" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 -- "-$task_pid" 2>/dev/null; then
            echo "进程 $task_pid 未及时结束，正在终止；请核对示教器状态。" >&2
            kill -KILL -- "-$task_pid" 2>/dev/null || true
        fi
    fi
    wait "$task_pid" 2>/dev/null || true
}

cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM
    # Stop leader targets first, then allow the RT server to finish its loop.
    stop_group "$client_pid"
    stop_group "$server_pid"
    stop_group "$prepare_pid"
    [[ -z "$session_dir" ]] || echo "服务端日志：$session_dir/server.log"
    exit "$exit_code"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

while (( $# > 0 )); do
    case "$1" in
        --ip) robot_ip="${2:?--ip 缺少地址}"; shift 2 ;;
        --local-ip) local_ip="${2:?--local-ip 缺少地址}"; shift 2 ;;
        --gello-port) gello_port="${2:?--gello-port 缺少路径}"; shift 2 ;;
        --calib) calib="${2:?--calib 缺少路径}"; shift 2 ;;
        --port) server_port="${2:?--port 缺少端口}"; shift 2 ;;
        --hz) hz="${2:?--hz 缺少数值}"; shift 2 ;;
        --max-speed-deg) max_speed_deg="${2:?--max-speed-deg 缺少数值}"; shift 2 ;;
        --enable-motion) enable_motion=true; shift ;;
        --yes) assume_yes=true; shift ;;
        --skip-prepare) skip_prepare=true; shift ;;
        --calibrate-zero) calibrate_zero=true; shift ;;
        --prepare-speed) prepare_speed="${2:?缺少准备速度}"; shift 2 ;;
        --prepare-motion-timeout) prepare_motion_timeout="${2:?缺少等待时间}"; shift 2 ;;
        --prepare-max-step-deg) prepare_max_step_deg="${2:?缺少角度差上限}"; shift 2 ;;
        --start-recording) gripper_options+=("$1"); shift ;;
        --raw-data-root|--task|--session-path-file|--record-queue-size|--record-feedback-max-age)
            [[ $# -ge 2 ]] || fail "$1 缺少参数"
            gripper_options+=("$1" "$2"); shift 2 ;;
        --gripper-host|--gripper-port|--gripper-timeout)
            [[ $# -ge 2 ]] || fail "$1 缺少参数"
            prepare_gripper_options+=("$1" "$2")
            gripper_options+=("$1" "$2"); shift 2 ;;
        --gripper-id|--gripper-open-deg|--gripper-close-deg|--gripper-open-pos|--gripper-closed-pos|--gripper-hz|--gripper-speed|--gripper-force|--gripper-stale-timeout)
            [[ $# -ge 2 ]] || fail "$1 缺少参数"
            gripper_options+=("$1" "$2"); shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; fail "未知参数：$1" ;;
    esac
done

for task_command in uv flock setsid; do
    command -v "$task_command" >/dev/null || fail "未安装 $task_command"
done
if [[ "$calibrate_zero" == true ]]; then
    [[ "$enable_motion" == true && "$skip_prepare" == false ]] || fail "--calibrate-zero 需要 --enable-motion，不能与 --skip-prepare 一起使用"
    [[ ! -e "$calib" ]] || fail "标定文件已存在：$calib；重标定请用 --calib 指定新文件"
else
    [[ -r "$calib" ]] || fail "缺少标定：$calib；主臂保持零位后加 --enable-motion --calibrate-zero 首次标定"
fi
[[ -r "$gello_port" && -w "$gello_port" ]] || fail "GELLO 串口不存在或无读写权限：$gello_port"

# Only the launcher owns the descriptor; its child processes must not inherit it.
exec 9>"$sdk_dir/.follow.lock"
flock -n 9 || fail "已有跟随启动流程正在运行"

# Parse all motion/network options before connecting to CR7 or opening the serial port.
uv run --locked --project "$sdk_dir" python - "$robot_ip" "$local_ip" "$server_port" "$hz" "$max_speed_deg" "$enable_motion" "$prepare_speed" "$prepare_motion_timeout" "$prepare_max_step_deg" "${gripper_options[@]}" <<'PY'
import sys
from xcore_sdk_python.cli import parser, validate
root = parser()
server = root.parse_args(["follow-server", "--ip", sys.argv[1], "--local-ip", sys.argv[2],
                         "--port", sys.argv[3], "--max-speed-deg", sys.argv[5]])
validate(server, root)
client = root.parse_args(["follow", "--port", sys.argv[3], "--hz", sys.argv[4], *sys.argv[10:]]
                         + ([] if sys.argv[6] == "true" else ["--dry-run"]))
validate(client, root)
prepare = root.parse_args(["follow-prepare", "--speed", sys.argv[7],
                           "--motion-timeout", sys.argv[8], "--max-step-deg", sys.argv[9]])
validate(prepare, root)
PY

echo "[1/3] 检查 SDK、标定与 GELLO 只读反馈"
"${xcore[@]}" doctor 9>&-
if [[ "$calibrate_zero" != true ]]; then
    "${xcore[@]}" follow-check --serial "$gello_port" --calib "$calib" 9>&-
fi
if [[ "$enable_motion" == true && "$assume_yes" != true ]]; then
    if [[ "$calibrate_zero" == true ]]; then
        echo "首次标定要求 GELLO 处于六轴 0° 姿态；CR7 将先归到六轴 0°。"
    fi
    read -r -p "将准备对齐并启用跟随；准备期间保持 GELLO 不动，确认运动范围后输入 y：" answer
    [[ "$answer" == y || "$answer" == yes ]] || { echo "已取消。"; exit 0; }
fi

mkdir -p "$sdk_dir/logs"
session_dir="$(mktemp -d "$sdk_dir/logs/follow-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
if [[ "$enable_motion" == true && "$skip_prepare" == false ]]; then
    prepare_options=()
    [[ "$calibrate_zero" != true ]] || prepare_options=(--calibrate-zero)
    echo "[准备] 按标定低速移动到 GELLO 当前目标；保持主臂不动，单段最多等待 ${prepare_motion_timeout}s。"
    PYTHONUNBUFFERED=1 setsid "${xcore[@]}" follow-prepare \
        --ip "$robot_ip" --local-ip "$local_ip" --serial "$gello_port" --calib "$calib" \
        --speed "$prepare_speed" --motion-timeout "$prepare_motion_timeout" \
        --max-step-deg "$prepare_max_step_deg" --output "$session_dir/preparation.json" \
        "${prepare_options[@]}" "${prepare_gripper_options[@]}" 9>&- &
    prepare_pid=$!
    wait "$prepare_pid"
    prepare_pid=""
    echo "[准备] 对齐完成，准备会话已关闭；现在启动实时跟随。"
fi
server_options=()
client_options=(--dry-run)
if [[ "$enable_motion" == true ]]; then
    server_options=(--enable-motion --yes)
    client_options=(--yes)
fi

echo "[2/3] 启动独占 CR7 SDK 会话的 ZMQ 服务端"
PYTHONUNBUFFERED=1 setsid "${xcore[@]}" follow-server \
    --ip "$robot_ip" --local-ip "$local_ip" --port "$server_port" \
    --max-speed-deg "$max_speed_deg" --quiet "${server_options[@]}" \
    9>&- >"$session_dir/server.log" 2>&1 &
server_pid=$!
server_ready=false
for ((attempt=0; attempt<300; attempt++)); do
    if ! kill -0 "$server_pid" 2>/dev/null; then
        cat "$session_dir/server.log" >&2
        fail "CR7 服务端启动失败"
    fi
    if grep -Fq 'CR7 follow server:' "$session_dir/server.log"; then
        server_ready=true
        break
    fi
    sleep 0.1
done
[[ "$server_ready" == true ]] || { cat "$session_dir/server.log" >&2; fail "服务端 30 秒内未就绪"; }
cat "$session_dir/server.log"

echo "[3/3] 启动 GELLO 客户端；按 Ctrl+C 停止客户端和服务端"
PYTHONUNBUFFERED=1 setsid "${xcore[@]}" follow --port "$server_port" \
    --serial "$gello_port" --calib "$calib" --hz "$hz" \
    "${client_options[@]}" "${gripper_options[@]}" 9>&- <&0 &
client_pid=$!
wait "$client_pid"
