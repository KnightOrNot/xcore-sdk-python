"""Private process boundary so native SDK calls cannot hang the CLI forever."""

from __future__ import annotations

import json
import signal
import sys

from .commands import execute

RESULT_PREFIX = "XCORE_RESULT:"


def _interrupt(signum: int, frame: object) -> None:
    # Group termination can also be forwarded by the parent runner. A second
    # signal must not interrupt the robot's stop/idle/power restoration.
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    raise KeyboardInterrupt


def main() -> None:
    signal.signal(signal.SIGTERM, _interrupt)
    options = json.load(sys.stdin)
    try:
        result = {"ok": True, "command": options["command"], "result": execute(options)}
    except KeyboardInterrupt:
        result = {"ok": False, "command": options["command"], "error": "Interrupted"}
    except Exception as exc:
        result = {"ok": False, "command": options["command"], "error": str(exc)}
    print(
        RESULT_PREFIX + json.dumps(result, ensure_ascii=False, allow_nan=False),
        flush=True,
    )


if __name__ == "__main__":
    main()
