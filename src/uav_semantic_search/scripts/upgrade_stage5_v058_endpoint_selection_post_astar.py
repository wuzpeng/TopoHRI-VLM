#!/usr/bin/env python3
"""Merge the v0.5.8 corrected endpoint-selection / post-selection A* settings."""
from __future__ import annotations

import os
import shutil
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH = os.path.join(ROOT, 'config', 'vlm_semantic_search.yaml')
BACKUP = PATH + '.before_v058_endpoint_post_astar.bak'


def main() -> None:
    with open(PATH, 'r') as handle:
        data = yaml.safe_load(handle) or {}
    root = data.setdefault('vlm_semantic_search', {})
    if not os.path.exists(BACKUP):
        shutil.copy2(PATH, BACKUP)

    # Preserve backend.api_key/base_url/model, local_retry, and local_dispatch.
    root['pause_gazebo_during_epoch'] = False
    fallback = root.setdefault('backend_fallback', {})
    fallback.setdefault('enabled', True)
    fallback.setdefault('response_timeout_sec', 12.0)
    fallback.setdefault('minimum_goal_displacement_m', 1.0)
    route = root.setdefault('route_planner', {})
    route.setdefault('request_topic', '/vlm/route_request')
    route.setdefault('result_topic', '/vlm/route_result')
    route.setdefault('recompute_from_current_pose', True)
    route.setdefault('recompute_from_current_map', True)
    validator = root.setdefault('validator', {})
    validator.setdefault('recheck_astar', True)
    validator.setdefault('reject_duplicate_candidate_assignment', True)
    validator.setdefault('fallback_mode', 'safe_nearest_candidate')
    validator.setdefault('assign_unassigned_robots', True)
    validator.setdefault('require_initial_uav_explore', True)

    with open(PATH, 'w') as handle:
        yaml.safe_dump(data, handle, default_flow_style=False, allow_unicode=True, sort_keys=False)
    print('Backup:', BACKUP)
    print('Updated v0.5.8 endpoint-only selection / post-selection A* configuration:', PATH)
    print('Preserved backend.api_key/base_url/model and the existing local retry settings.')


if __name__ == '__main__':
    main()
