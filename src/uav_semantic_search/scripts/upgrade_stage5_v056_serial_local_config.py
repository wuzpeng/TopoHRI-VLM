#!/usr/bin/env python3
"""Upgrade Stage-5 cloud VLM configuration for serial local-image inference.

This utility preserves backend.api_key, backend.base_url, backend.model and all
user-supplied target-query fields.  It only fixes the coordinator/scheduler
settings needed by Stage-5 v0.5.6.
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import yaml


def as_float(value, default):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=None, help='Path to vlm_semantic_search.yaml')
    parser.add_argument('--no-backup', action='store_true')
    args = parser.parse_args()

    default = Path(__file__).resolve().parent.parent / 'config' / 'vlm_semantic_search.yaml'
    path = Path(args.config).expanduser() if args.config else default
    if not path.is_file():
        raise SystemExit('Missing config: %s' % path)

    data = yaml.safe_load(path.read_text()) or {}
    root = data.setdefault('vlm_semantic_search', {})
    if not args.no_backup:
        backup = path.with_suffix(path.suffix + '.before_v056.bak')
        if not backup.exists():
            shutil.copy2(str(path), str(backup))
            print('Backup:', backup)

    backend = root.setdefault('backend', {})
    backend_timeout = as_float(backend.get('timeout_sec', 45.0), 45.0)
    # Do not silently shorten a user-raised HTTP timeout.  The per-robot
    # coordinator wait must remain slightly longer than that timeout.
    local_wait = max(as_float(root.get('local_response_timeout_sec', 0.0), 0.0), backend_timeout + 5.0)
    central_wait = max(as_float(root.get('central_response_timeout_sec', 0.0), 0.0), backend_timeout + 5.0)

    root['pause_gazebo_during_epoch'] = False
    root['local_response_timeout_sec'] = local_wait
    root['central_response_timeout_sec'] = central_wait
    root['validator_timeout_sec'] = as_float(root.get('validator_timeout_sec', 6.0), 6.0)
    root['skip_central_on_local_timeout'] = True
    root['skip_central_on_local_failure'] = True

    dispatch = root.setdefault('local_dispatch', {})
    dispatch.update({
        'mode': 'sequential',
        'participant_order': dispatch.get('participant_order', ['uav0', 'uav1', 'ugv0']),
        'per_robot_response_timeout_sec': max(
            as_float(dispatch.get('per_robot_response_timeout_sec', 0.0), 0.0), backend_timeout + 5.0),
        'inter_request_delay_sec': as_float(dispatch.get('inter_request_delay_sec', 0.30), 0.30),
    })

    scheduler = root.setdefault('scheduler', {})
    scheduler.update({
        'min_trigger_interval_sec': max(as_float(scheduler.get('min_trigger_interval_sec', 20.0), 20.0), 20.0),
        'backend_failure_cooldown_sec': max(as_float(scheduler.get('backend_failure_cooldown_sec', 60.0), 60.0), 60.0),
        'trigger_on_goal_reached': True,
        'trigger_on_robot_blocked': True,
        'trigger_on_target_query_change': True,
        'trigger_on_distance_in_uncertain_space': False,
        'trigger_on_new_observation_sector': False,
        'trigger_on_visual_novelty': False,
        'trigger_on_map_free_space_expanded': False,
        'trigger_on_topology_cue_changed': False,
    })

    validator = root.setdefault('validator', {})
    validator['fallback_mode'] = 'hold'
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False, default_flow_style=False))
    print('Updated Stage-5 v0.5.6 serial-local-VLM configuration:', path)
    print('Preserved backend.api_key/base_url/model and target-query fields.')
    print('HTTP timeout = %.1fs | per-robot local wait = %.1fs | dispatch = sequential' % (
        backend_timeout, dispatch['per_robot_response_timeout_sec']))


if __name__ == '__main__':
    main()
