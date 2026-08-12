#!/usr/bin/env python3
"""Aggregate manual Human--AI VLM trial summaries."""
from __future__ import annotations

import argparse
import csv
import json
import os
from typing import Any, Dict, List

import numpy as np


def _mean(values: List[float]) -> float:
    return float(np.mean(values)) if values else float('nan')


def _std(values: List[float]) -> float:
    return float(np.std(values, ddof=1)) if len(values) > 1 else 0.0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'results_dir',
        nargs='?',
        default=os.path.expanduser('~/harp_sar_ws/experiment_results'),
    )
    parser.add_argument(
        '--output-prefix',
        default='aggregate_metrics',
    )
    args = parser.parse_args()

    summaries: List[Dict[str, Any]] = []
    for root, _dirs, files in os.walk(os.path.expanduser(args.results_dir)):
        if 'summary.json' not in files:
            continue
        path = os.path.join(root, 'summary.json')
        try:
            with open(path, 'r', encoding='utf-8') as stream:
                item = json.load(stream)
            item['_summary_path'] = path
            summaries.append(item)
        except (OSError, ValueError) as exc:
            print('Skipped %s: %s' % (path, exc))

    if not summaries:
        raise SystemExit('No summary.json files found under %s' % args.results_dir)

    rows = []
    map_ids = sorted(set(str(item.get('map_id', 'unknown')) for item in summaries))
    for map_id in map_ids + ['ALL']:
        selected = (
            summaries
            if map_id == 'ALL'
            else [item for item in summaries if str(item.get('map_id')) == map_id]
        )
        successful = [item for item in selected if int(item.get('success', 0)) == 1]
        motion = [
            float(item['task_motion_time_sec']) for item in successful
            if item.get('task_motion_time_sec') is not None
        ]
        route = [
            float(item['team_route_length_m']) for item in successful
            if item.get('team_route_length_m') is not None
        ]
        coverage = [
            float(item['coverage_at_success_percent']) for item in successful
            if item.get('coverage_at_success_percent') is not None
        ]
        rcr = [
            float(item['region_conflict_rate_percent']) for item in selected
            if item.get('region_conflict_rate_percent') is not None
        ]
        cvr = [
            float(item['constraint_violation_rate_percent']) for item in selected
            if item.get('constraint_violation_rate_percent') is not None
        ]
        rcr_conflicts = sum(int(item.get('rcr_conflicting_pairs', 0)) for item in selected)
        rcr_pairs = sum(int(item.get('rcr_eligible_pairs', 0)) for item in selected)
        cvr_violations = sum(int(item.get('cvr_violating_decisions', 0)) for item in selected)
        cvr_decisions = sum(int(item.get('cvr_raw_decisions', 0)) for item in selected)
        rows.append({
            'map_id': map_id,
            'trials': len(selected),
            'successful_trials': len(successful),
            'success_rate_percent': 100.0 * len(successful) / len(selected),
            'successful_task_motion_time_mean_sec': _mean(motion),
            'successful_task_motion_time_std_sec': _std(motion),
            'successful_team_route_length_mean_m': _mean(route),
            'successful_team_route_length_std_m': _std(route),
            'coverage_at_success_mean_percent': _mean(coverage),
            'coverage_at_success_std_percent': _std(coverage),
            # Trial-level mean/std are the paper-facing statistics. Pooled
            # counts are retained as an auditable descriptive supplement.
            'rcr_valid_trials': len(rcr),
            'region_conflict_rate_mean_percent': _mean(rcr),
            'region_conflict_rate_std_percent': _std(rcr),
            'rcr_pooled_conflicting_pairs': rcr_conflicts,
            'rcr_pooled_eligible_pairs': rcr_pairs,
            'rcr_pooled_percent': (
                100.0 * rcr_conflicts / rcr_pairs if rcr_pairs else float('nan')
            ),
            'cvr_valid_trials': len(cvr),
            'constraint_violation_rate_mean_percent': _mean(cvr),
            'constraint_violation_rate_std_percent': _std(cvr),
            'cvr_pooled_violating_decisions': cvr_violations,
            'cvr_pooled_raw_decisions': cvr_decisions,
            'cvr_pooled_percent': (
                100.0 * cvr_violations / cvr_decisions
                if cvr_decisions else float('nan')
            ),
        })

    output_dir = os.path.expanduser(args.results_dir)
    csv_path = os.path.join(output_dir, args.output_prefix + '.csv')
    json_path = os.path.join(output_dir, args.output_prefix + '.json')
    with open(csv_path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    with open(json_path, 'w', encoding='utf-8') as stream:
        json.dump(rows, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print('Wrote %s' % csv_path)
    print('Wrote %s' % json_path)


if __name__ == '__main__':
    main()
