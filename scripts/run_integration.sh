#!/usr/bin/env bash
# Run only inside an installed ROS 2 Jazzy / Gazebo Harmonic Linux environment.
set -euo pipefail

for required_command in ros2 setsid; do
    if ! command -v "$required_command" >/dev/null 2>&1; then
        echo "Missing $required_command. Source ROS and the workspace, or use the Docker image." >&2
        exit 2
    fi
done

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
log_dir="${SWARM_TEST_LOG_DIR:-${project_root}/artifacts/integration/$(date -u +%Y%m%dT%H%M%SZ)-$$}"
mkdir -p "$log_dir"
log_dir="$(cd "$log_dir" && pwd)"
# Keep this test's simulator discovery separate from other local Gazebo sessions.
export GZ_PARTITION="swarm_integration_$$"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((20 + $$ % 200))}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONUNBUFFERED=1

test_mode="${SWARM_TEST_MODE:-communication}"
case "$test_mode" in
    communication)
        launch=(ros2 launch drone_swarm_simulator simulation.launch.py gui:=false auto_demo:=false)
        probe=(ros2 run drone_swarm_simulator communication_smoke_test)
        ;;
    mission)
        launch=(ros2 launch drone_swarm_simulator simulation.launch.py gui:=false auto_demo:=true)
        probe=(python3 "$project_root/scripts/test_mission_live.py")
        ;;
    fleet|show)
        default_count=5
        [[ "$test_mode" == show ]] && default_count=20
        count="${SWARM_TEST_COUNT:-$default_count}"
        if ! [[ "$count" =~ ^[1-9][0-9]{0,2}$ ]] || ((count > 400)); then
            echo "SWARM_TEST_COUNT must be an integer between 1 and 400, got: $count" >&2
            exit 2
        fi
        launch=(ros2 launch drone_swarm_simulator swarm.launch.py "count:=$count" gui:=false auto_start:=true)
        if [[ "$test_mode" == show ]]; then
            probe=(python3 "$project_root/scripts/test_show_live.py" --ros-args -p "count:=$count")
        else
            probe=(python3 "$project_root/scripts/test_fleet_live.py" --ros-args -p "count:=$count")
        fi
        ;;
    *)
        echo "SWARM_TEST_MODE must be communication, mission, fleet, or show, got: $test_mode" >&2
        exit 2
        ;;
esac

launch_pid=""
smoke_pid=""
cleanup() {
    local result=$?
    trap - EXIT INT TERM
    for child_pid in "$smoke_pid" "$launch_pid"; do
        if [[ -n "$child_pid" ]]; then
            kill -TERM -- "-$child_pid" 2>/dev/null || true
        fi
    done
    # A launch process may already have exited while its Gazebo children remain.
    # Probe the entire new process group, not just the launch parent's PID.
    for attempt in {1..50}; do
        local alive=0
        for child_pid in "$smoke_pid" "$launch_pid"; do
            if [[ -n "$child_pid" ]] && kill -0 -- "-$child_pid" 2>/dev/null; then
                alive=1
            fi
        done
        [[ "$alive" == 0 ]] && break
        sleep 0.1
    done
    for child_pid in "$smoke_pid" "$launch_pid"; do
        if [[ -n "$child_pid" ]]; then
            kill -KILL -- "-$child_pid" 2>/dev/null || true
            wait "$child_pid" 2>/dev/null || true
        fi
    done
    echo "Integration logs: $log_dir"
    if [[ "$result" != 0 ]]; then
        echo "Integration FAILED (exit $result). Recent launch output:" >&2
        tail -n 80 "$log_dir/launch.log" >&2 || true
    fi
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Starting headless Gazebo; test=$test_mode ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Integration logs: $log_dir"
setsid "${launch[@]}" >"$log_dir/launch.log" 2>&1 &
launch_pid=$!

setsid "${probe[@]}" "$@" \
    >"$log_dir/smoke.log" 2>&1 &
smoke_pid=$!

while kill -0 "$smoke_pid" 2>/dev/null; do
    if ! kill -0 "$launch_pid" 2>/dev/null; then
        echo "Simulation launch exited before the $test_mode test completed." >&2
        cat "$log_dir/smoke.log"
        exit 1
    fi
    sleep 0.2
done

result=0
wait "$smoke_pid" || result=$?
cat "$log_dir/smoke.log"
exit "$result"
