#!/usr/bin/env bash
set -euo pipefail
if ! docker container inspect drone-swarm-app >/dev/null 2>&1; then
    echo "The application container is already stopped."; exit 0
fi
managed="$(docker inspect -f '{{index .Config.Labels "drone_swarm.managed"}}' drone-swarm-app)"
if [[ "$managed" != true ]]; then
    echo "Refusing to stop a container not created by this application." >&2; exit 2
fi
docker stop --time 10 drone-swarm-app
