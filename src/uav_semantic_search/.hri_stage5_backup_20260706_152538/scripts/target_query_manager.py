#!/usr/bin/env python3
"""Online target-query manager for open-vocabulary VLM search."""
from __future__ import annotations

import os
import sys
import threading
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
        self.query.setdefault('query_version', 0)
        self.pub = rospy.Publisher('/vlm/target_query', String, queue_size=3, latch=True)
        rospy.Subscriber('/vlm/set_target_query', String, self._set_cb, queue_size=5)
        self.pub.publish(compact_json(self.query))
        rospy.loginfo('Target query initialized: v%d %s', self.query['query_version'], self.query.get('query_text', ''))

    def _set_cb(self, msg):
        update = safe_json_loads(msg.data, None)
        if not isinstance(update, dict) or not str(update.get('query_text', '')).strip():
            rospy.logwarn('Ignored invalid /vlm/set_target_query payload; query_text is required.')
            return
        with self.lock:
            old_version = int(self.query.get('query_version', 0))
            merged = dict(self.query)
            merged.update(update)
            merged['query_version'] = old_version + 1
            merged.setdefault('query_id', 'Q%d' % merged['query_version'])
            self.query = merged
            self.pub.publish(compact_json(self.query))
        rospy.logwarn('Target query switched to v%d: %s', self.query['query_version'], self.query.get('query_text'))


def main():
    rospy.init_node('target_query_manager')
    TargetQueryManager()
    rospy.spin()


if __name__ == '__main__':
    main()
