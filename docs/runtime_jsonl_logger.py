"""Lightweight JSONL runtime logger.

Writes one JSON line per event to ``~/.ros/runtime_logs/<node_name>_<timestamp>.jsonl``.
"""

import json
import os
import time
from pathlib import Path


class RuntimeJsonlLogger:
    """Append-only JSONL logger used by production ROS 2 nodes."""

    def __init__(self, node_name: str):
        log_dir = Path.home() / ".ros" / "runtime_logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self._path = log_dir / f"{node_name}_{stamp}.jsonl"
        # create the file
        self._path.touch()

    @property
    def path(self) -> Path:
        return self._path

    def log(self, event: str, **kwargs):
        """Append a single JSON line with *event* tag and arbitrary fields."""
        record = {"t": time.time(), "event": event, **kwargs}
        try:
            with open(self._path, "a") as f:
                f.write(json.dumps(record, default=str) + "\n")
        except Exception:
            pass  # never crash the node because of logging
