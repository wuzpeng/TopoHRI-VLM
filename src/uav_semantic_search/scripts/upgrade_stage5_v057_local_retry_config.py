#!/usr/bin/env python3
"""Enable bounded identical-snapshot retry for Stage-5 local VLM calls.

The script preserves backend.api_key/base_url/model and target query fields.
Retries resend the exact same epoch snapshot and prompt.  No image resizing,
JPEG-quality reduction, token reduction, or fresh ROS image capture occurs.
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


def as_int(value, default):
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)


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
        backup = path.with_suffix(path.suffix + '.before_v057.bak')
        if not backup.exists():
            shutil.copy2(str(path), str(backup))
            print('Backup:', backup)

    # Keep the prior cloud-safe non-pausing and sequential dispatch policy.
    root['pause_gazebo_during_epoch'] = False
    root['skip_central_on_local_timeout'] = True
    root['skip_central_on_local_failure'] = True

    backend = root.setdefault('backend', {})
    backend_timeout = as_float(backend.get('timeout_sec', 45.0), 45.0)
    retry = root.setdefault('local_retry', {})
    # One normal request plus up to one identical retry is the safe default.
    # It avoids a multi-minute stale epoch while still recovering from a
    # transient queued/timeout response.
    retry.update({
        'enabled': True,
        'max_retries': max(1, as_int(retry.get('max_retries', 1), 1)),
        'attempt_timeout_sec': min(backend_timeout, max(30.0, as_float(retry.get('attempt_timeout_sec', 35.0), 35.0))),
        'total_deadline_sec': max(75.0, as_float(retry.get('total_deadline_sec', 75.0), 75.0)),
        'initial_backoff_sec': max(0.0, as_float(retry.get('initial_backoff_sec', 1.5), 1.5)),
        'backoff_multiplier': max(1.0, as_float(retry.get('backoff_multiplier', 2.0), 2.0)),
        'retry_on_timeout': True,
        'retry_on_connection_errors': True,
        'retry_on_http_429_5xx': True,
        'snapshot_policy': 'identical_epoch_snapshot',
    })

    dispatch = root.setdefault('local_dispatch', {})
    dispatch['mode'] = 'sequential'
    dispatch['participant_order'] = dispatch.get('participant_order', ['uav0', 'uav1', 'ugv0'])
    # Must exceed the full retry budget, otherwise the coordinator could mark a
    # robot missing while its retry worker is still active.
    local_wait = max(as_float(dispatch.get('per_robot_response_timeout_sec', 0.0), 0.0),
                     float(retry['total_deadline_sec']) + 5.0)
    dispatch['per_robot_response_timeout_sec'] = local_wait
    dispatch['inter_request_delay_sec'] = as_float(dispatch.get('inter_request_delay_sec', 0.30), 0.30)
    root['local_response_timeout_sec'] = max(as_float(root.get('local_response_timeout_sec', 0.0), 0.0), local_wait)

    scheduler = root.setdefault('scheduler', {})
    scheduler['backend_failure_cooldown_sec'] = max(
        60.0, as_float(scheduler.get('backend_failure_cooldown_sec', 60.0), 60.0))
    scheduler['min_trigger_interval_sec'] = max(
        20.0, as_float(scheduler.get('min_trigger_interval_sec', 20.0), 20.0))

    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False, default_flow_style=False))
    print('Updated Stage-5 v0.5.7 bounded local retry configuration:', path)
    print('Preserved backend.api_key/base_url/model and target-query fields.')
    print('Same-snapshot retry: max_retries=%d | attempt_timeout=%.1fs | total_deadline=%.1fs | per_robot_wait=%.1fs' % (
        retry['max_retries'], retry['attempt_timeout_sec'], retry['total_deadline_sec'], dispatch['per_robot_response_timeout_sec']))


if __name__ == '__main__':
    main()
