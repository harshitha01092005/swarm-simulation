#!/usr/bin/env python3
"""Verify actual Gazebo LED materials through the local application's HTTP API.

Run after other acceptance tests have finished, with the swarm stationary.
Scene info resolves owned entity IDs; full ECS state supplies current material
bytes because Gazebo Harmonic's scene graph may retain creation-time materials.
Only lighting settings are changed, and their original values are restored.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen


# Run inside the existing simulator container, using its installed protobufs.
# All subprocess calls use argument vectors; no user value becomes shell code.
GAZEBO_READER = r'''
import hashlib
from functools import lru_cache
import json
import subprocess
import sys

sys.path.insert(0, "/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python")
from google.protobuf import text_format
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.serialized_map_pb2 import SerializedStepMap
from gz.msgs10.material_pb2 import Material

def query(service, message):
    result = subprocess.run([
        "gz", "service", "-s", service,
        "--reqtype", "gz.msgs.Empty", "--reptype", message.DESCRIPTOR.full_name,
        "--timeout", "3000", "--req", "",
    ], capture_output=True, text=True, timeout=8, check=False)
    if result.returncode or not result.stdout.strip():
        raise RuntimeError(f"{service}: {result.stderr[:500]} {result.stdout[:500]}")
    if len(result.stdout) > 64 * 1024 * 1024:
        raise RuntimeError("Gazebo response exceeds 64 MiB")
    text_format.Parse(result.stdout, message)
    return message

def only_named(items, name):
    matches = [item for item in items if item.name.rsplit("::", 1)[-1] == name]
    if len(matches) != 1:
        raise RuntimeError(f"Expected exactly one owned {name}, found {len(matches)}")
    return matches[0]

def rgba(color):
    return [color.r, color.g, color.b, color.a]

@lru_cache(maxsize=16)
def component_type(name):
    # gz::common::hash64 (FNV-1a), used by components::Factory for the exact
    # registered component type name. SerializedComponent.type is uint64.
    value = 0xcbf29ce484222325
    for byte in ("gz_sim_components." + name).encode("ascii"):
        value = ((value ^ byte) * 0x100000001b3) & ((1 << 64) - 1)
    return value

def material_type():
    return component_type("Material")

def component(entity, name, required=True):
    matches = [c for c in entity.components.values() if c.type == component_type(name) and not c.remove]
    if len(matches) == 1 and not entity.remove:
        return matches[0].component
    if not matches and not required:
        return None
    raise RuntimeError(f"Entity {entity.id} does not have exactly one live {name} component")

def verify_owner(state, entity_id, name, kind, parent_id=None):
    if entity_id not in state.state.entities:
        raise RuntimeError(f"Owned {name} entity {entity_id} is absent from live ECS state")
    entity = state.state.entities[entity_id]
    if component(entity, "Name").decode("utf-8") != name:
        raise RuntimeError(f"Entity {entity_id} does not have the expected live name {name}")
    component(entity, kind)
    if parent_id is not None and int(component(entity, "ParentEntity")) != parent_id:
        raise RuntimeError(f"Entity {entity_id} is not owned by expected parent {parent_id}")
    return entity

def observe():
    scene = query("/world/fleet_world/scene/info", Scene())
    # SceneInfoService converts sdf::Scene (render settings), not sdf::World;
    # Gazebo 8.11 leaves the optional scene name unset. Verify the world and
    # full ownership chain from live ECS components instead of assuming a name.
    if scene.name and scene.name != "fleet_world":
        raise RuntimeError(f"Unexpected scene: {scene.name!r}")
    model = only_named(scene.model, "drone_001")
    link = only_named(model.link, "base_link")
    visual = only_named(link.visual, "show_light")
    state = query("/world/fleet_world/state", SerializedStepMap())
    worlds = [entity for entity in state.state.entities.values()
              if not entity.remove and component(entity, "World", required=False) is not None]
    worlds = [entity for entity in worlds if component(entity, "Name") == b"fleet_world"]
    if len(worlds) != 1:
        raise RuntimeError("Expected exactly one live fleet_world entity in the queried ECS state")
    world = worlds[0]
    verify_owner(state, model.id, "drone_001", "Model", world.id)
    verify_owner(state, link.id, "base_link", "Link", model.id)
    entity = verify_owner(state, visual.id, "show_light", "Visual", link.id)
    if visual.parent_id != link.id:
        raise RuntimeError("Scene visual parent does not match its live ECS owner")
    components = [c for c in entity.components.values() if c.type == material_type() and not c.remove]
    if entity.remove or len(components) != 1:
        raise RuntimeError("Owned visual does not have exactly one live Material component")
    material = Material()
    material.ParseFromString(components[0].component)
    if not material.HasField("emissive"):
        raise RuntimeError("Live Material component has no emissive color")
    return {
        "source": "/world/fleet_world/state", "identity_source": "/world/fleet_world/scene/info",
        "world": "fleet_world", "world_id": world.id, "scene_name": scene.name,
        "ownership_verified": True, "model": model.name, "model_id": model.id,
        "link": link.name, "link_id": link.id, "visual": visual.name, "visual_id": visual.id,
        "material_type_id": material_type(),
        "emissive_rgba": rgba(material.emissive),
        "diffuse_rgba": rgba(material.diffuse),
        "scene_cached_emissive_rgba": rgba(visual.material.emissive),
        "material_sha256": hashlib.sha256(components[0].component).hexdigest(),
        "simulation_time": state.stats.sim_time.sec + state.stats.sim_time.nsec * 1e-9,
    }

print(json.dumps(observe(), allow_nan=False))
'''


class LEDAcceptance:
    def __init__(self, args):
        self.args = args
        self.url = args.url.rstrip("/")
        parsed = urlsplit(self.url)
        if parsed.scheme != "http" or parsed.hostname not in {"127.0.0.1", "localhost", "::1"}:
            raise ValueError("Use an HTTP URL on localhost, 127.0.0.1, or ::1")
        if parsed.username or parsed.password or parsed.query or parsed.fragment or parsed.path not in {"", "/"}:
            raise ValueError("URL must be the local application's base URL")
        if not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_.-]{0,127}", args.container):
            raise ValueError("Invalid Docker container name")
        if not 1 <= args.timeout <= 120:
            raise ValueError("timeout must be between 1 and 120 seconds per color")
        self.original = None
        self.changed = False
        self.report = {
            "started_at": datetime.now(timezone.utc).isoformat(), "container": args.container,
            "url": self.url, "passed": False, "checks": [], "restored": False,
            "evidence": "Gazebo ECS Material bytes; controller RGB is used only as a command acknowledgement",
        }

    def request(self, path, body=None):
        data = None if body is None else json.dumps(body, allow_nan=False).encode()
        request = Request(self.url + path, data=data, headers={"Content-Type": "application/json"})
        try:
            with urlopen(request, timeout=20 if body is not None else 5) as response:
                encoded = response.read(2 * 1024 * 1024 + 1)
        except HTTPError as error:
            raise RuntimeError(f"HTTP {error.code}: {error.read(1000).decode(errors='replace')}") from error
        if len(encoded) > 2 * 1024 * 1024:
            raise RuntimeError("HTTP response exceeds 2 MiB")
        return json.loads(encoded)

    def configure(self, settings, label):
        result = self.request("/api/command", {
            "action": "configure", "settings": settings, "request_id": "gazebo-leds-" + label,
        })
        if result.get("ok") is not True or result.get("accepted") is not True:
            raise RuntimeError(f"Lighting command was not acknowledged: {result}")

    def read_material(self):
        result = subprocess.run([
            "docker", "exec", self.args.container,
            "/ws/src/drone_swarm_simulator/docker/entrypoint.sh", "python3", "-c", GAZEBO_READER,
        ], capture_output=True, text=True, timeout=22, check=False)
        if result.returncode:
            raise RuntimeError(f"Gazebo material query failed: {result.stderr[-1800:]} {result.stdout[-500:]}")
        evidence = json.loads(result.stdout)
        if not all(isinstance(x, (int, float)) and math.isfinite(x) and 0 <= x <= 1
                   for x in evidence["emissive_rgba"]):
            raise AssertionError("Gazebo returned a nonfinite or out-of-range emissive color")
        return evidence

    @staticmethod
    def settings_match(state, expected):
        return all(state.get("config", {}).get(key) == value for key, value in expected.items())

    def verify_color(self, label, rgb):
        expected = {"color_mode": "single", "color": rgb, "brightness": 1.0}
        self.changed = True  # The request may apply even if the HTTP reply times out.
        self.configure(expected, label)
        deadline = time.monotonic() + self.args.timeout
        last = None
        while time.monotonic() < deadline:
            state = self.request("/api/state")
            if state.get("state") == "ERROR" or not state.get("connected"):
                raise AssertionError(f"Runtime lost connection during LED test: {state.get('error')}")
            if not self.settings_match(state, expected):
                time.sleep(0.1)
                continue
            last = self.read_material()
            if all(abs(actual - target) <= 1e-5 for actual, target in zip(last["emissive_rgba"], [*rgb, 1.0])):
                self.report["checks"].append({"name": label, "expected_rgb": rgb, "passed": True, **last})
                print(f"PASS: Gazebo {label} emissive={last['emissive_rgba']} entity={last['visual_id']}", flush=True)
                return
            time.sleep(0.2)
        raise AssertionError(f"Gazebo LED did not become {label}: last measured material={last}")

    def run(self):
        initial = self.request("/api/state")
        if not initial.get("connected") or initial.get("layout_pending"):
            raise RuntimeError("Wait for a connected, settled Gazebo fleet before running the LED test")
        if initial.get("state") not in {"IDLE", "STOPPED", "FORMATION_COMPLETE", "ANIMATING", "PAUSED"}:
            raise RuntimeError("Run the LED test after movement and other acceptance tests have finished")
        if initial.get("pending_config") or initial.get("pending_formation") or initial.get("image_processing"):
            raise RuntimeError("Wait for pending commands and image processing to finish")
        self.original = {key: initial["config"][key] for key in ("color_mode", "color", "brightness")}
        self.report["original_lighting"] = self.original
        self.report["initial_material"] = self.read_material()
        self.verify_color("red", [1.0, 0.0, 0.0])
        self.verify_color("blue", [0.0, 0.0, 1.0])
        red, blue = self.report["checks"]
        if red["visual_id"] != blue["visual_id"] or red["material_sha256"] == blue["material_sha256"]:
            raise AssertionError("Expected different material bytes on the same owned Gazebo visual")
        self.report["passed"] = True

    def restore(self):
        if not self.changed:
            self.report["restored"] = True
            return
        self.configure(self.original, "restore")
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            if self.settings_match(self.request("/api/state"), self.original):
                self.report["restored"] = True
                print("PASS: original lighting settings restored", flush=True)
                return
            time.sleep(0.1)
        raise RuntimeError("Original lighting settings were not restored within eight seconds")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--container", default="drone-swarm-app")
    parser.add_argument("--url", default="http://127.0.0.1:8765")
    parser.add_argument("--report", default="artifacts/gazebo-leds.json")
    parser.add_argument("--timeout", type=float, default=15, help="Wall-time limit per commanded color")
    args = parser.parse_args()
    try:
        test = LEDAcceptance(args)
    except ValueError as error:
        parser.error(str(error))
    started = time.monotonic()
    try:
        test.run()
    except (Exception, KeyboardInterrupt) as error:
        test.report["error"] = f"{type(error).__name__}: {error}"
        print(f"FAIL: {test.report['error']}", file=sys.stderr, flush=True)
    finally:
        try:
            test.restore()
        except (Exception, KeyboardInterrupt) as error:
            test.report["passed"] = False
            test.report["restore_error"] = f"{type(error).__name__}: {error}"
            print(f"FAIL: original lighting restoration: {error}", file=sys.stderr, flush=True)
        test.report["wall_seconds"] = round(time.monotonic() - started, 3)
        target = Path(args.report).expanduser()
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(test.report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print(f"Report: {target}", flush=True)
    return 0 if test.report["passed"] and test.report["restored"] else 1


if __name__ == "__main__":
    sys.exit(main())
