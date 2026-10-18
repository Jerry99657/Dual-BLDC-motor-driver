#!/usr/bin/env python3
"""Install the standard editable KK_UI Demo only into a new target directory."""

import argparse
import shutil
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("target", type=Path, help="new project-owned Demo directory")
    args = parser.parse_args()
    # Check before resolve() so even a dangling symlink is never followed.
    target = args.target.absolute()
    if target.exists() or target.is_symlink():
        print(f"refusing to overwrite existing target: {target}", file=sys.stderr)
        return 2
    source = Path(__file__).resolve().parent.parent / "assets" / "kk-ui-demo"
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, target)
    print(f"installed editable standard KK_UI Demo: {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
