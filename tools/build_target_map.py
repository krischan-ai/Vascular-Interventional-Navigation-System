from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


def read_fcsv(path: Path) -> list[dict[str, Any]]:
    points: list[dict[str, Any]] = []
    columns: list[str] | None = None

    with path.open("r", encoding="utf-8", newline="") as file_obj:
        for raw_line in file_obj:
            line = raw_line.strip()
            if not line:
                continue
            if line.startswith("# columns ="):
                columns = [item.strip() for item in line.split("=", 1)[1].split(",")]
                continue
            if line.startswith("#"):
                continue

            row = next(csv.reader([raw_line], skipinitialspace=True))
            if columns is None:
                raise ValueError(f"FCSV columns header not found: {path}")

            data = {columns[index]: row[index] for index in range(min(len(columns), len(row)))}
            points.append(
                {
                    "id": data.get("id", ""),
                    "label": data.get("label", ""),
                    "position_lps": [
                        float(data["x"]),
                        float(data["y"]),
                        float(data["z"]),
                    ],
                }
            )

    return points


def build_targets(case_dir: Path) -> Path:
    endpoints_path = case_dir / "raw/Endpoints.fcsv"
    if not endpoints_path.is_file():
        raise FileNotFoundError(f"Endpoints file not found: {endpoints_path}")

    endpoints = read_fcsv(endpoints_path)
    targets = {
        "case_id": case_dir.name,
        "coordinate_system": "LPS",
        "unit": "mm",
        "aliases": {
            "root": None,
            "bca": None,
            "lcca": None,
        },
        "endpoints": {
            point["label"]: {
                "id": point["id"],
                "position_lps": point["position_lps"],
            }
            for point in endpoints
        },
        "notes": [
            "Aliases are intentionally unset until BCA/LCCA/root are confirmed in a viewer.",
            "Endpoint labels come directly from the source FCSV file.",
        ],
    }

    output_path = case_dir / "derived/targets.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(targets, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return output_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build targets.json from VPP FCSV files.")
    parser.add_argument(
        "case_dir",
        nargs="?",
        type=Path,
        default=Path("data") / "vpp_assets" / "case_001",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_path = build_targets(args.case_dir.resolve())
    print(f"Wrote target map: {output_path}")


if __name__ == "__main__":
    main()
