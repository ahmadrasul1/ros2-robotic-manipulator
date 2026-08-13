#!/usr/bin/env python3

"""Wait until the current PDO bridge run publishes its readiness file."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", default="/tmp/rascl_pdo_ready")
    parser.add_argument("--run-id", default="")
    parser.add_argument("--poll", type=float, default=0.05)
    args = parser.parse_args()

    path = Path(args.path)
    print(
        f"Waiting for PDO bridge readiness: {path}, run_id={args.run_id!r}",
        flush=True,
    )
    while True:
        try:
            content = path.read_text(encoding="utf-8")
        except (FileNotFoundError, OSError, UnicodeError):
            time.sleep(max(args.poll, 0.01))
            continue

        values: dict[str, str] = {}
        for line in content.splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                values[key.strip()] = value.strip()

        run_matches = not args.run_id or values.get("run_id") == args.run_id

        if content.startswith("PDO_READY\n") and run_matches:
            print(content.strip(), flush=True)
            return

        if content.startswith("PDO_FAILED\n") and run_matches:
            reason = values.get("reason", "PDO bridge startup failed")
            print(
                f"PDO bridge did not become ready: {reason}",
                file=sys.stderr,
                flush=True,
            )
            raise SystemExit(1)

        time.sleep(max(args.poll, 0.01))


if __name__ == "__main__":
    main()
