#!/usr/bin/env bash
# dog_monitor.sh —— 独立于 opencode 的轻量运行监控。
#
# 用法：
#   tools/dog_monitor.sh start [RUN_DIR]   启动后台监控 daemon（每 60s 采样一次）
#   tools/dog_monitor.sh stop              停止监控 daemon
#   tools/dog_monitor.sh status [RUN_DIR]  一次性查看当前状态（不启动 daemon）
#   tools/dog_monitor.sh tail [RUN_DIR]    查看最近监控快照
#
# 输出（均在 RUN_DIR 内）：
#   dog_monitor.log    每 60s 一条快照（追加）
#   dog_monitor.status 最新状态（覆盖）
#   dog_monitor.alert  卡死/冻结告警（追加；同时打印到 stdout）
#
# 卡死判据（保守，避免把探索停驻误报）：
#   1) sim_frozen：sim elapsed 连续 4 次（约 4 分钟）不推进 → 仿真冻结；
#   2) pose_stuck：EXPLORE/CHANGE/RETURN_HOME 期间连续 6 分钟位移 < 0.05m
#      且 stuck/recover 计数在增长 → 可能卡死；
#   3) 终态/换层结果直接记录。
set -u

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_RUN_DIR="$(ls -dt /tmp/seed8127_p1_8m_* /tmp/seed8127_closed_loop_* 2>/dev/null | head -1)"
RUN_DIR="${2:-$DEFAULT_RUN_DIR}"
if [[ -z "$RUN_DIR" || ! -d "$RUN_DIR" ]]; then
  echo "错误：未找到运行目录，请显式传入 RUN_DIR" >&2
  exit 1
fi
MISSION_LOG="$RUN_DIR/mission.log"
STATUS="$RUN_DIR/dog_monitor.status"
ALERT="$RUN_DIR/dog_monitor.alert"
SNAP="$RUN_DIR/dog_monitor.log"
PID_FILE="/tmp/dog_monitor_$(echo "$RUN_DIR" | md5sum | cut -c1-8).pid"

monitor_once() {
  python3 - "$MISSION_LOG" "$SNAP" "$STATUS" <<'PYEOF'
import json, math, os, sys, time

path, snap, status = sys.argv[1], sys.argv[2], sys.argv[3]
last = None
for line in open(path, errors="replace"):
    j = line.find("{")
    if j < 0:
        continue
    try:
        o = json.loads(line[j:])
    except Exception:
        continue
    if o.get("event") == "mission_runtime":
        last = o
if last is None:
    print("无 mission_runtime 数据", flush=True)
    sys.exit(0)

m = last.get("mapper") or {}
c = last.get("controller") or {}
pose = last.get("pose") or [0, 0, 0]
entry = {
    "wall": time.strftime("%H:%M:%S"),
    "sim": last.get("elapsed"),
    "state": last.get("state"),
    "floor": last.get("current_floor"),
    "target": last.get("target_floor"),
    "cov": last.get("coverage_ratio"),
    "visit": m.get("visited_traversable_ratio"),
    "lowcand": m.get("low_candidate_core_count"),
    "hard": m.get("hard_core_count"),
    "stuck": c.get("stuck_count"),
    "recover": c.get("recover_count"),
    "path": c.get("path_active"),
    "x": round(pose[0], 2),
    "y": round(pose[1], 2),
    "term": last.get("terminal_outcome"),
    "rhc": last.get("return_home_completed"),
}
with open(snap, "a") as f:
    f.write(json.dumps(entry, ensure_ascii=False) + "\n")

# 读历史做卡死判定
hist = []
try:
    with open(snap) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    hist.append(json.loads(line))
                except Exception:
                    pass
except OSError:
    pass
hist = hist[-12:]
cur = entry
alerts = []
if cur["term"] != "running":
    alerts.append("任务终态：term=%s rhc=%s" % (cur["term"], cur["rhc"]))
if len(hist) >= 4:
    sims = [h["sim"] for h in hist[-4:]]
    if len(set(sims)) == 1:
        alerts.append("sim 冻结告警：elapsed 连续 4 次未推进 (%.1f)" % cur["sim"])
if len(hist) >= 6:
    pts = hist[-6:]
    disp = max(
        math.hypot(pts[i]["x"] - pts[0]["x"], pts[i]["y"] - pts[0]["y"])
        for i in range(1, len(pts))
    )
    active_states = ("EXPLORE_FLOOR", "CHANGE_FLOOR", "RETURN_HOME")
    stuck_growth = cur["stuck"] is not None and (
        cur["stuck"] > (pts[0]["stuck"] or 0)
        or cur["recover"] > (pts[0]["recover"] or 0)
    )
    if cur["state"] in active_states and disp < 0.05 and stuck_growth:
        alerts.append(
            "疑似卡死：6 分钟内位移<0.05m 且 stuck=%s recover=%s state=%s pose=(%.2f,%.2f)"
            % (cur["stuck"], cur["recover"], cur["state"], cur["x"], cur["y"])
        )

with open(status, "w") as f:
    f.write(json.dumps(entry, ensure_ascii=False, indent=1) + "\n")
if alerts:
    with open(os.path.join(os.path.dirname(snap), "dog_monitor.alert"), "a") as f:
        f.write("[%s] %s\n" % (time.strftime("%H:%M:%S"), "; ".join(alerts)))
    for a in alerts:
        print("ALERT: %s" % a, flush=True)
PYEOF
}

start() {
  if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "监控已在运行：pid=$(cat "$PID_FILE")"
    return 0
  fi
  # 双 fork 脱离当前会话，保证与 opencode/终端关闭无关
  nohup setsid bash -c "
    while true; do
      $0 once '$RUN_DIR'
      sleep 60
    done
  " > "$RUN_DIR/dog_monitor.out" 2>&1 < /dev/null &
  disown
  echo "$!" > "$PID_FILE"
  echo "监控已启动：pid=$(cat "$PID_FILE") run=$RUN_DIR"
  echo "查看状态：tools/dog_monitor.sh status $RUN_DIR"
}

stop() {
  if [[ -f "$PID_FILE" ]]; then
    kill "$(cat "$PID_FILE")" 2>/dev/null
    rm -f "$PID_FILE"
    echo "监控已停止"
  else
    echo "监控未运行"
  fi
}

live_once() {
  python3 - "$MISSION_LOG" <<'PYEOF'
import json, os, sys, time

path = sys.argv[1]
last = None
try:
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        f.seek(max(0, size - 262144))
        tail = f.read().decode(errors="replace")
    for line in tail.splitlines():
        j = line.find("{")
        if j < 0:
            continue
        try:
            o = json.loads(line[j:])
        except Exception:
            continue
        if o.get("event") == "mission_runtime":
            last = o
except OSError:
    pass
if last is None:
    print("[%s] 无 mission_runtime 数据" % time.strftime("%H:%M:%S"), flush=True)
    sys.exit(0)
m = last.get("mapper") or {}
c = last.get("controller") or {}
pose = last.get("pose") or [0, 0, 0]
print(
    "[%s] sim=%5.1f %-12s floor=%s->%s cov=%.3f visit=%.3f lowcand=%s hard=%s "
    "stuck=%s rec=%s path=%s pose=(%.2f,%.2f) term=%s rhc=%s"
    % (
        time.strftime("%H:%M:%S"), last.get("elapsed"), last.get("state"),
        last.get("current_floor"), last.get("target_floor"), last.get("coverage_ratio"),
        m.get("visited_traversable_ratio"), m.get("low_candidate_core_count"),
        m.get("hard_core_count"), c.get("stuck_count"), c.get("recover_count"),
        c.get("path_active"), pose[0], pose[1], last.get("terminal_outcome"),
        last.get("return_home_completed"),
    ),
    flush=True,
)
PYEOF
}

live_loop() {
  local interval="${2:-3}"
  while true; do
    live_once
    sleep "$interval"
  done
}

case "${1:-status}" in
  start) start ;;
  stop) stop ;;
  once) monitor_once ;;
  status)
    if [[ -f "$STATUS" ]]; then
      echo "=== 最新状态（$STATUS）==="
      cat "$STATUS"
    else
      echo "尚无状态文件（monitor 未运行或 run 未开始）"
    fi
    if [[ -f "$ALERT" ]]; then
      echo "=== 告警记录 ==="
      tail -5 "$ALERT"
    fi
    ;;
  tail)
    if [[ -f "$SNAP" ]]; then
      tail -10 "$SNAP"
    else
      echo "无快照"
    fi
    ;;
  live)
    if [[ "${3:-}" == "--loop" ]]; then
      live_loop "${4:-3}"
    elif [[ "${2:-}" == "--loop" ]]; then
      live_loop "${3:-3}"
    else
      live_once
    fi
    ;;
  *) echo "用法: $0 {start|stop|status|tail|live [--loop]} [RUN_DIR]" >&2; exit 1 ;;
esac
