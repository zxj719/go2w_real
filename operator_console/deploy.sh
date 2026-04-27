#!/usr/bin/env bash
# deploy.sh — Deploy operator_console to the GO2W robot
#
# Usage:
#   ./deploy.sh                    # deploy to the default GO2W IP
#   ./deploy.sh 192.168.1.100      # deploy to a specific IP
#   ./deploy.sh --check            # dry-run: print the deployment plan
#
# Prerequisites on the robot:
#   - Ubuntu 20.04 + ROS 2 Foxy
#   - python3-yaml (sudo apt install python3-yaml)
#   - SSH access as unitree user
#   - ros-foxy-foxglove-bridge (installed automatically if missing)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
GO2W_USER="${GO2W_USER:-unitree}"
GO2W_IP="${1:-192.168.123.18}"
GO2W_DEST="/home/${GO2W_USER}/ros_ws/src/go2w_real/operator_console"
SCRIPTS_DEST="/home/${GO2W_USER}/ros_ws/src/go2w_real/scripts"
CHECK_ONLY=false

if [[ "${1:-}" == "--check" ]]; then
    CHECK_ONLY=true
    GO2W_IP="<GO2W_IP>"
fi

log() { printf '[deploy] %s\n' "$*"; }

if [[ "$CHECK_ONLY" == true ]]; then
    log "=== Dry-run: deployment plan ==="
    log ""
    log "1. Copy operator_console/ to ${GO2W_USER}@${GO2W_IP}:${GO2W_DEST}/"
    log "2. Copy scripts/start_navigation_no_server.sh to ${SCRIPTS_DEST}/"
    log "3. Install ros-foxy-foxglove-bridge (apt)"
    log "4. Install python3-yaml (apt)"
    log "5. Install systemd services:"
    log "   sudo cp ${GO2W_DEST}/systemd/foxglove-bridge.service /etc/systemd/system/"
    log "   sudo cp ${GO2W_DEST}/systemd/launch-manager.service /etc/systemd/system/"
    log "   sudo systemctl daemon-reload"
    log "   sudo systemctl enable --now foxglove-bridge launch-manager"
    log "6. Verify:"
    log "   curl http://${GO2W_IP}:8080/api/health"
    log "   Open http://${GO2W_IP}:8080 on phone"
    log "   Open Foxglove Web, connect to ws://${GO2W_IP}:8765"
    exit 0
fi

log "Deploying to ${GO2W_USER}@${GO2W_IP}:${GO2W_DEST}/"

# Step 1: sync operator_console
log "Step 1/5: Copying operator_console/"
ssh "${GO2W_USER}@${GO2W_IP}" "mkdir -p ${GO2W_DEST} ${SCRIPTS_DEST}"
rsync -avz --delete \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    "${SCRIPT_DIR}/" \
    "${GO2W_USER}@${GO2W_IP}:${GO2W_DEST}/"

# Step 2: sync the new bringup script
log "Step 2/5: Copying scripts/start_navigation_no_server.sh"
rsync -avz \
    "${SCRIPT_DIR}/../scripts/start_navigation_no_server.sh" \
    "${GO2W_USER}@${GO2W_IP}:${SCRIPTS_DEST}/"
ssh "${GO2W_USER}@${GO2W_IP}" "chmod +x ${SCRIPTS_DEST}/start_navigation_no_server.sh"

# Step 3: dependencies
log "Step 3/5: Installing dependencies..."
ssh "${GO2W_USER}@${GO2W_IP}" "dpkg -l ros-foxy-foxglove-bridge >/dev/null 2>&1 || \
    (echo 'Installing foxglove_bridge...' && sudo apt update -qq && sudo apt install -y -qq ros-foxy-foxglove-bridge)"
ssh "${GO2W_USER}@${GO2W_IP}" "python3 -c 'import yaml' 2>/dev/null || \
    (echo 'Installing python3-yaml...' && sudo apt install -y -qq python3-yaml)"

# Step 4: install systemd services
log "Step 4/5: Installing systemd services..."
ssh "${GO2W_USER}@${GO2W_IP}" "sudo cp ${GO2W_DEST}/systemd/foxglove-bridge.service /etc/systemd/system/ && \
    sudo cp ${GO2W_DEST}/systemd/launch-manager.service /etc/systemd/system/ && \
    sudo systemctl daemon-reload && \
    sudo systemctl enable foxglove-bridge launch-manager"

# Step 5: start / restart
log "Step 5/5: Starting services..."
ssh "${GO2W_USER}@${GO2W_IP}" "sudo systemctl restart foxglove-bridge launch-manager"

log ""
log "=== Deployment complete ==="
log ""
log "Control panel: http://${GO2W_IP}:8080"
log "Health check:  curl http://${GO2W_IP}:8080/api/health"
log "POI list:      curl http://${GO2W_IP}:8080/api/waypoints"
log "Foxglove:      Open https://app.foxglove.dev, connect ws://${GO2W_IP}:8765"
log ""
log "Service status:"
ssh "${GO2W_USER}@${GO2W_IP}" "systemctl --no-pager status foxglove-bridge launch-manager" || true
