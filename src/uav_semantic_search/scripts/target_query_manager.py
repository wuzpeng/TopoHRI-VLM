#!/usr/bin/env python3
"""Online target-query manager for open-vocabulary VLM search.

Every accepted update receives a new ``query_version`` and matching ``query_id``.
The latter is intentionally overwritten rather than ``setdefault``-ed: retaining
Q0 after a human update makes debugging and query-specific semantic evidence
ambiguous.
"""
from __future__ import annotations

import os
import sys
import threading
import time
from typing import Any, Dict

import rospy
from std_msgs.msg import String

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from vlm_common import compact_json, safe_json_loads


class TargetQueryManager:
    def __init__(self):
        root = rospy.get_param('/vlm_semantic_search')
        self.lock = threading.RLock()
        self.query: Dict[str, Any] = dict(root.get('target_query', {}))
        version = int(self.query.get('query_version', 0))
        self.query['query_version'] = version
        self.query['query_id'] = 'Q%d' % version
        self.query.setdefault('source', 'system_default')
        self.pub = rospy.Publisher('/vlm/target_query', String, queue_size=3, latch=True)
        # Target switching is a task-level reset.  Executors and the route planner
        # subscribe to this topic to drop old routes immediately instead of
        # continuing stale verification/search commands while the new query epoch
        # is being prepared.
        self.cancel_pub = rospy.Publisher('/vlm/query_switch_cancel', String, queue_size=10)
        rospy.Subscriber('/vlm/set_target_query', String, self._set_cb, queue_size=10)
        self.pub.publish(compact_json(self.query))
        rospy.loginfo('Target query initialized: %s v%d %s', self.query['query_id'], version,
                      self.query.get('query_text', ''))

    def _set_cb(self, msg: String):
        update = safe_json_loads(msg.data, None)
        if not isinstance(update, dict) or not str(update.get('query_text', '')).strip():
            rospy.logwarn('Ignored invalid /vlm/set_target_query payload; query_text is required.')
            return
        with self.lock:
            old_version = int(self.query.get('query_version', 0))
            merged = dict(self.query)
            merged.update(update)
            new_version = old_version + 1
            merged['query_version'] = new_version
            merged['query_id'] = 'Q%d' % new_version
            merged['query_text'] = str(merged['query_text']).strip()
            merged['query_updated_wall_time'] = round(time.time(), 3)
            cancel_notice = {
                'reason': 'TARGET_QUERY_CHANGE',
                'old_query_version': old_version,
                'new_query_version': new_version,
                'new_query_id': merged['query_id'],
                'new_query_text': str(merged.get('query_text', '')).strip(),
                # A query switch invalidates all active semantic assignments.
                # The next TARGET_QUERY_CHANGE epoch will produce fresh routes.
                'cancel_task_types': [
                    'AERIAL_INSPECT',
                    'GROUND_VERIFY',
                    'HRI_REGION_SEARCH',
                    'QUERY_RESCAN',
                    'EXPLORE',
                    'INSPECT',
                    'HOVER_AND_SCAN',
                    'SCAN_IN_PLACE',
                ],
                'wall_time': round(time.time(), 3),
            }
            self.query = merged
            self.pub.publish(compact_json(self.query))
            self.cancel_pub.publish(compact_json(cancel_notice))
        rospy.logwarn('Target query switched to %s: %s', self.query['query_id'], self.query.get('query_text'))


def main():
    rospy.init_node('target_query_manager')
    TargetQueryManager()
    rospy.spin()


if __name__ == '__main__':
    main()
