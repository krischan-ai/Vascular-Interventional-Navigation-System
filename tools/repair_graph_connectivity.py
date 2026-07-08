from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from services.graph_connectivity import connected_components, repair_graph_file


def main() -> int:
    parser = argparse.ArgumentParser(description="Connect graph.json components with virtual edges.")
    parser.add_argument("graph", type=Path, help="Path to graph.json")
    parser.add_argument("--check", action="store_true", help="Only report component count")
    args = parser.parse_args()

    raw = json.loads(args.graph.read_text(encoding="utf-8"))
    before = connected_components(raw)
    print(f"before: {len(before)} component(s), sizes={[len(c) for c in before[:10]]}")

    if args.check:
        return 0 if len(before) == 1 else 1

    edges = repair_graph_file(args.graph)
    raw_after = json.loads(args.graph.read_text(encoding="utf-8"))
    after = connected_components(raw_after)
    print(f"added: {len(edges)} virtual edge pair(s)")
    print(f"after: {len(after)} component(s), sizes={[len(c) for c in after[:10]]}")
    return 0 if len(after) == 1 else 1


if __name__ == "__main__":
    raise SystemExit(main())
