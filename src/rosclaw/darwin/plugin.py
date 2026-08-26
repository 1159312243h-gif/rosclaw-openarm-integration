"""DarwinPlugin — Runtime plugin entry point."""

from __future__ import annotations

import logging
from typing import Any

from rosclaw.core.lifecycle import LifecycleMixin

from .engine import DarwinEngine

logger = logging.getLogger("rosclaw.darwin.plugin")


class DarwinPlugin(LifecycleMixin):
    """Runtime plugin for rosclaw-darwin."""

    name = "rosclaw-darwin"
    version = "1.0.0"

    def __init__(
        self,
        config: dict[str, Any] | None = None,
        event_bus: Any | None = None,
        seekdb_client: Any | None = None,
    ):
        super().__init__()
        self.config = config or {}
        self.engine: DarwinEngine | None = None
        self._event_bus = event_bus
        self._seekdb_client = seekdb_client

    def _do_initialize(self) -> None:
        self.engine = DarwinEngine(
            event_bus=self._event_bus,
            seekdb_client=self._seekdb_client,
            default_seeds=self.config.get("default_seeds", 10),
            default_episodes=self.config.get("default_episodes", 50),
        )
        logger.info("DarwinPlugin: initialized")

    def _do_start(self) -> None:
        logger.info("DarwinPlugin: started")

    def _do_stop(self) -> None:
        logger.info("DarwinPlugin: stopped")

    def health(self) -> dict[str, Any]:
        return {
            "plugin": self.name,
            "version": self.version,
            "engine_ready": self.engine is not None,
            "status": "healthy" if self.engine else "not_ready",
        }
