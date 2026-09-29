#!/bin/bash
# Installs wifi_roam_helper.py as a root systemd service (needs `sudo -v` first; no package installed).
# Jetson:  bash install_wifi_roam_helper.sh
# Robot:   bash install_wifi_roam_helper.sh "/media/administrator/USB Drive/slam_series2/tools/wifi_roam_helper.py" /home/administrator/jobs
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../../run/lib/paths.sh"
set -e
PY="${1:-$TOOLS_DIR/wifi_roam_helper.py}"
JOBS="${2:-$JOBS_DIR}"
mkdir -p "$JOBS"
cat > /tmp/wifi-roam-helper.service <<UNIT
[Unit]
Description=WiFi roaming helper - switch access point on weak signal, keep power saving off (SLAM series 2)
After=NetworkManager.service
[Service]
ExecStart=/usr/bin/python3 "$PY" "$JOBS"
Restart=always
RestartSec=5
[Install]
WantedBy=multi-user.target
UNIT
sudo -n install -m 644 /tmp/wifi-roam-helper.service /etc/systemd/system/wifi-roam-helper.service
sudo -n systemctl daemon-reload
sudo -n systemctl enable --now wifi-roam-helper.service
sleep 5; systemctl is-active wifi-roam-helper.service; cat "$JOBS/wifi_roam.progress"
