#!/bin/bash
# recv_selftest.sh - the robot receiver's REFUSAL paths, on the Jetson, on a private ROS master (port 11511).
# Checks: (1) refuses to listen on all addresses; (2) refuses limits above its ceilings; (3) refuses a sender with a
# different run name (E frame, connection closed); (4) with the right run name but NO joystick messages, a "go"
# command is NOT published (deadman); (5) with deadman_button -1 (sim only) the same command IS published, clamped
# to 0.30 / 0.40, and negative forward speed becomes 0; (6) after the sender goes silent, zeros then silence.
# Writes <out>/recv_selftest.txt. Stops everything it started by PID.
set -u
OUT="${1:?usage: recv_selftest.sh <out_dir>}"; mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11511 ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1 ROS_LOG_DIR="$OUT/roslog_selftest"
R="$OUT/recv_selftest.txt"; : > "$R"
setsid nohup roscore -p 11511 > "$OUT/selftest_roscore.log" 2>&1 < /dev/null & CORE=$!
for i in $(seq 1 20); do timeout 3 rostopic list > /dev/null 2>&1 && break; sleep 1; done
say() { echo "$*" | tee -a "$R"; }
python3 "$HERE/nav_cmd_recv.py" __name:=t1 _run:=t _bind:=0.0.0.0 > "$OUT/st1.log" 2>&1
grep -q "REFUSED: bind" "$OUT/st1.log" && say "1 PASS refuses bind 0.0.0.0" || say "1 FAIL bind 0.0.0.0 not refused"
python3 "$HERE/nav_cmd_recv.py" __name:=t2 _run:=t _bind:=127.0.0.1 _max_vx:=0.9 > "$OUT/st2.log" 2>&1
grep -q "REFUSED: limits" "$OUT/st2.log" && say "2 PASS refuses max_vx 0.9" || say "2 FAIL limits not refused"
start_recv() {
    setsid nohup python3 "$HERE/nav_cmd_recv.py" __name:=nav_cmd_recv _run:=t _bind:=127.0.0.1 _port:=8119 \
        _deadman_button:="$1" _jobs_dir:="$OUT" > "$OUT/st_recv_$1.log" 2>&1 < /dev/null &
    RECV=$!; sleep 3
}
client() {   # client RUN VX WZ SECONDS -> prints what came back
python3 - "$@" <<'EOF'
import json, socket, struct, sys, time
run, vx, wz, secs = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
F, C = struct.Struct("<cI"), struct.Struct("<IqffBB")
s = socket.create_connection(("127.0.0.1", 8119), timeout=3)
b = json.dumps({"proto": 1, "run": run, "sender": "selftest"}).encode(); s.sendall(F.pack(b"H", len(b)) + b)
k, n = F.unpack(s.recv(5)); body = s.recv(n)
print("reply", k.decode(), body.decode()[:120])
if k != b"W": sys.exit(0)
t0 = time.time(); i = 0
while time.time() - t0 < secs:
    i += 1; c = C.pack(i, time.time_ns(), vx, wz, 1, 0); s.sendall(F.pack(b"C", len(c)) + c); time.sleep(0.05)
s.close()
EOF
}
start_recv 7
(timeout 6 rostopic echo -n 3 /cmd_vel > "$OUT/st_cmd_deadman.txt" 2>&1 &)
client wrong 0.2 0 1 > "$OUT/st3.txt"; grep -q "reply E" "$OUT/st3.txt" && say "3 PASS wrong run name refused: $(cut -c1-90 "$OUT/st3.txt")" || say "3 FAIL: $(cat "$OUT/st3.txt")"
client t 0.2 0 3 > /dev/null; sleep 3
grep -q "linear" "$OUT/st_cmd_deadman.txt" && say "4 FAIL moved with no joystick" || say "4 PASS no /cmd_vel published without the joystick button"
kill -INT "$RECV"; sleep 2
start_recv -1
(timeout 8 rostopic echo /cmd_vel/linear/x > "$OUT/st_vx.txt" 2>&1 &)
(timeout 8 rostopic echo /cmd_vel/angular/z > "$OUT/st_wz.txt" 2>&1 &)
client t 0.9 1.5 2 > /dev/null
client t -0.5 -1.5 1 > /dev/null
sleep 5
MAXVX=$(grep -E "^-?[0-9][0-9.]*$" "$OUT/st_vx.txt" | sort -g | tail -1); MINVX=$(grep -E "^-?[0-9][0-9.]*$" "$OUT/st_vx.txt" | sort -g | head -1)
MAXWZ=$(grep -E "^-?[0-9][0-9.]*$" "$OUT/st_wz.txt" | sort -g | tail -1); MINWZ=$(grep -E "^-?[0-9][0-9.]*$" "$OUT/st_wz.txt" | sort -g | head -1)
say "5 published forward speed range [$MINVX, $MAXVX] (want [0, ~0.3]), turn range [$MINWZ, $MAXWZ] (want [-0.4, 0.4])"
NZ=$(grep -c "STOP: " "$OUT/st_recv_-1.log"); say "6 stop events after the sender went silent: $NZ (want >= 1); last: $(grep 'STOP:' "$OUT/st_recv_-1.log" | tail -1 | cut -d' ' -f3-)"
kill -INT "$RECV"; sleep 2; kill -INT "$CORE"; sleep 2
ps -o pid= -p "$RECV" > /dev/null 2>&1 && kill -KILL "$RECV"
ps -o pid= -p "$CORE" > /dev/null 2>&1 && kill -KILL -- "-$CORE"
cat "$R"
