#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
mkdir -p artifacts
mode=desktop
count=400
while [[ $# -gt 0 ]]; do
    case "$1" in
        --browser) mode=browser; shift ;;
        --headless) mode=headless; shift ;;
        --count) count="${2:?Supply a count from 1 to 400}"; shift 2 ;;
        *) echo "Usage: scripts/start.sh [--browser|--headless] [--count 1..400]" >&2; exit 2 ;;
    esac
done
if [[ ! "$count" =~ ^[1-9][0-9]{0,2}$ ]] || (( count > 400 )); then
    echo "Drone count must be an integer from 1 to 400." >&2; exit 2
fi
if ! command -v docker >/dev/null || ! docker info >/dev/null 2>&1; then
    echo "Start Docker Desktop or Colima, then retry. See TROUBLESHOOTING.md." >&2; exit 2
fi
if [[ "$mode" == desktop ]] && ! .venv/bin/python -c 'from PySide6.QtWebEngineWidgets import QWebEngineView' >/dev/null 2>&1; then
    echo "Install the desktop environment first: bash scripts/install_desktop.sh" >&2; exit 2
fi
if ! docker image inspect drone-swarm:latest >/dev/null 2>&1; then
    docker build -t drone-swarm:latest -f docker/Dockerfile .
fi
if docker container inspect drone-swarm-app >/dev/null 2>&1; then
    managed="$(docker inspect -f '{{index .Config.Labels "drone_swarm.managed"}}' drone-swarm-app)"
    if [[ "$managed" != true ]]; then
        echo "Container name drone-swarm-app is already used by another application." >&2; exit 2
    fi
    if [[ "$(docker inspect -f '{{.State.Running}}' drone-swarm-app)" != true ]]; then
        docker logs drone-swarm-app >artifacts/last-session.log 2>&1 || true
        docker rm drone-swarm-app >/dev/null
    elif [[ "$(docker inspect -f '{{.Image}}' drone-swarm-app)" != "$(docker image inspect -f '{{.Id}}' drone-swarm:latest)" ]]; then
        echo "The image has been rebuilt. Run bash scripts/stop.sh, then start again to use it." >&2; exit 2
    else
        echo "Reusing the running simulator. Stop it before changing launch options."
    fi
fi
if ! docker container inspect drone-swarm-app >/dev/null 2>&1; then
    docker run -d --name drone-swarm-app --label drone_swarm.managed=true \
        -p 127.0.0.1:8765:8765 drone-swarm:latest \
        ros2 launch drone_swarm_simulator full_system.launch.py \
        count:="$count" gui:=false auto_start:=false >/dev/null
fi
ready=false
for attempt in {1..120}; do
    if curl --silent --fail --max-time 1 http://127.0.0.1:8765/api/state | python3 -c 'import json,sys; sys.exit(0 if json.load(sys.stdin).get("connected") else 1)' 2>/dev/null; then
        ready=true; break
    fi
    if [[ "$(docker inspect -f '{{.State.Running}}' drone-swarm-app 2>/dev/null || true)" != true ]]; then
        docker logs drone-swarm-app >artifacts/last-session.log 2>&1 || true
        tail -n 30 artifacts/last-session.log
        echo "Simulator stopped during startup. See artifacts/last-session.log. For 400 drones, we recommend 4 CPUs and 6 GiB allocated to Docker." >&2; exit 1
    fi
    sleep .5
done
if [[ "$ready" != true ]]; then
    docker logs drone-swarm-app >artifacts/last-session.log 2>&1 || true
    tail -n 60 artifacts/last-session.log
    echo "Gazebo did not report a connected fleet within 60 seconds. See artifacts/last-session.log." >&2; exit 1
fi
echo "Simulator: http://127.0.0.1:8765 — stop with: bash scripts/stop.sh"
case "$mode" in
    desktop) exec .venv/bin/python -m drone_swarm.gui.main_window --url http://127.0.0.1:8765 ;;
    browser)
        if [[ "$(uname -s)" == Darwin ]]; then open http://127.0.0.1:8765
        else xdg-open http://127.0.0.1:8765; fi ;;
esac
