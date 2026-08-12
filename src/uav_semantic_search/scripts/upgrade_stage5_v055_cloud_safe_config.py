#!/usr/bin/env python3
"""Upgrade Stage-5 VLM config for cloud API operation without erasing API credentials.

This utility changes only scheduling/recovery keys. Existing backend.base_url,
backend.model, backend.api_key and backend.api_key_env remain untouched.
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import yaml


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
        backup = path.with_suffix(path.suffix + '.before_v055.bak')
        if not backup.exists():
            shutil.copy2(str(path), str(backup))
            print('Backup:', backup)
    # Preserve backend credentials and model settings exactly as the user supplied them.
    root['pause_gazebo_during_epoch'] = False
    root['local_response_timeout_sec'] = 22.0
    root['central_response_timeout_sec'] = 22.0
    root['validator_timeout_sec'] = 6.0
    root['skip_central_on_local_timeout'] = True
    root['skip_central_on_local_failure'] = True

    scheduler = root.setdefault('scheduler', {})
    scheduler.update({
        'min_trigger_interval_sec': 20.0,
        'backend_failure_cooldown_sec': 60.0,
        'trigger_on_goal_reached': True,
        'trigger_on_robot_blocked': True,
        'trigger_on_target_query_change': True,
        # Cloud-safe defaults: semantic calls are reserved for discrete mission
        # events; re-enable these one by one after endpoint latency is stable.
        'trigger_on_distance_in_uncertain_space': False,
        'trigger_on_new_observation_sector': False,
        'trigger_on_visual_novelty': False,
        'trigger_on_map_free_space_expanded': False,
        'trigger_on_topology_cue_changed': False,
    })
    validator = root.setdefault('validator', {})
    validator['fallback_mode'] = 'hold'
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False, default_flow_style=False))
    print('Updated cloud-safe configuration:', path)
    print('Preserved backend.api_key/base_url/model fields.')


if __name__ == '__main__':
    main()
