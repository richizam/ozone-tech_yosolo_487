# -*- coding: utf-8 -*-
"""Minimal in-process pub/sub. Topic names mirror the ROS 2-shaped interface
we would use in an industrial deployment (see report, integration section):
  item_spawned, item_classified, item_settled, routing_cmd, cell_event, item_delivered
"""
from collections import defaultdict


class Bus:
    def __init__(self):
        self._subs = defaultdict(list)
        self.history = []              # every message, for the event log

    def subscribe(self, topic, fn):
        self._subs[topic].append(fn)

    def publish(self, topic, **msg):
        self.history.append((topic, dict(msg)))
        for fn in self._subs[topic]:
            fn(**msg)
