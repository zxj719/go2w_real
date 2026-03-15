#!/bin/bash
# Manually configure and activate Nav2 nodes on Foxy.
# Foxy's lifecycle_manager has a short service_call_timeout that can't be
# changed via parameter, causing "Failed to change state" on slow hardware.
# This script waits and retries.

set -e

NODES="controller_server planner_server recoveries_server bt_navigator"

echo "[activate_nav2] Waiting 5s for nodes to start..."
sleep 5

for node in $NODES; do
    echo "[activate_nav2] Configuring $node..."
    for attempt in 1 2 3 4 5; do
        if ros2 lifecycle set /$node configure 2>&1 | grep -q "Transitioning"; then
            echo "[activate_nav2] $node configured (attempt $attempt)"
            break
        fi
        echo "[activate_nav2] $node configure attempt $attempt failed, retrying in 2s..."
        sleep 2
    done
done

sleep 1

for node in $NODES; do
    echo "[activate_nav2] Activating $node..."
    for attempt in 1 2 3 4 5; do
        if ros2 lifecycle set /$node activate 2>&1 | grep -q "Transitioning"; then
            echo "[activate_nav2] $node activated (attempt $attempt)"
            break
        fi
        echo "[activate_nav2] $node activate attempt $attempt failed, retrying in 2s..."
        sleep 2
    done
done

echo "[activate_nav2] All Nav2 nodes should be active now."
echo "[activate_nav2] Verify: ros2 lifecycle get /controller_server"
