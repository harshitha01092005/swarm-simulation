#!/usr/bin/env python3
"""Reproducible CPU benchmarks, distinct from Gazebo and rendered FPS."""
import argparse
from pathlib import Path
import json
import platform
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from drone_swarm.swarm.fleet import FleetController
from drone_swarm.utils.fleet_config import FleetConfig


def benchmark(count):
    fleet = FleetController(FleetConfig(count=count))
    start = time.perf_counter()
    fleet.load_image((Path(__file__).resolve().parents[1] / 'assets/sample.png').read_bytes())
    processing = time.perf_counter() - start
    fleet.start()
    while fleet.state != 'FORMATION_COMPLETE':
        fleet.step(1/30)
    start = time.perf_counter()
    fleet.apply_formation('image')
    planning = time.perf_counter() - start
    certificate = fleet.snapshot()['certified_minimum_separation_m']
    samples = []
    while fleet.state != 'FORMATION_COMPLETE':
        start = time.perf_counter()
        fleet.step(1/30)
        samples.append(time.perf_counter() - start)
    snapshots = []
    for _ in range(100):
        start = time.perf_counter()
        json.dumps(fleet.snapshot(), allow_nan=False)
        snapshots.append(time.perf_counter() - start)
    return {'count': count, 'image_processing_ms': processing*1000,
            'image_assignment_planning_ms': planning*1000,
            'controller_step_p95_ms': float(np.percentile(samples,95))*1000,
            'snapshot_json_p95_ms': float(np.percentile(snapshots,95))*1000,
            'certified_separation_m': certificate, 'depth_layers':fleet.depth_layers}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='artifacts/core-performance.json')
    args = parser.parse_args()
    report = {'platform':platform.platform(), 'python':platform.python_version(),
              'numpy':np.__version__, 'measurement':'CPU core only; excludes Gazebo, ROS and renderer',
              'results':[benchmark(n) for n in (100,200,400)]}
    path=Path(args.output)
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__ == '__main__':
    main()
