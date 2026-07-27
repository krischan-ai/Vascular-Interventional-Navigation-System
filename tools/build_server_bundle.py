"""Build a standalone CathSim backend/simulation/RL server distribution."""

from __future__ import annotations

import argparse
import fnmatch
import hashlib
import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = ROOT / "server" / "manifest.json"


def load_manifest(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError(f"Unsupported manifest schema in {path}")
    return data


def _relative_matches(relative: Path, patterns: list[str]) -> bool:
    value = relative.as_posix()
    return any(
        fnmatch.fnmatch(value, pattern) or fnmatch.fnmatch(relative.name, pattern)
        for pattern in patterns
    )


def iter_source_files(source: Path, manifest: dict) -> list[Path]:
    if source.is_file():
        return [source]
    excluded_names = set(manifest["exclude_names"])
    excluded_globs = list(manifest["exclude_globs"])
    files: list[Path] = []
    for path in source.rglob("*"):
        relative = path.relative_to(ROOT)
        if any(part in excluded_names for part in relative.parts):
            continue
        if _relative_matches(relative, excluded_globs):
            continue
        if path.is_file():
            files.append(path)
    return sorted(files)


def copy_entry(source_rel: str, output: Path, manifest: dict) -> int:
    source = ROOT / source_rel
    copied = 0
    for path in iter_source_files(source, manifest):
        target = output / path.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        copied += 1
    return copied


def file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file_obj:
        for chunk in iter(lambda: file_obj.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_build_manifest(output: Path, definition: dict) -> dict:
    manifest_path = output / "SERVER_BUILD.json"
    files = []
    total_bytes = 0
    for path in sorted(output.rglob("*")):
        if not path.is_file() or path == manifest_path:
            continue
        size = path.stat().st_size
        total_bytes += size
        files.append({
            "path": path.relative_to(output).as_posix(),
            "size": size,
            "sha256": file_digest(path),
        })
    result = {
        "schema_version": 1,
        "name": definition["name"],
        "built_at": datetime.now(timezone.utc).isoformat(),
        "file_count": len(files),
        "total_bytes": total_bytes,
        "files": files,
    }
    manifest_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return result


def validate_output(output: Path, manifest: dict) -> None:
    missing = [entry for entry in manifest["required"] if not (output / entry).exists()]
    if missing:
        raise RuntimeError("Bundle is missing required paths: " + ", ".join(missing))
    leaked = [
        entry for entry in manifest["forbidden_top_level"] if (output / entry).exists()
    ]
    if leaked:
        raise RuntimeError("Forbidden paths leaked into bundle: " + ", ".join(leaked))


def build(output: Path, definition: dict) -> dict:
    if output.exists():
        raise FileExistsError(
            f"Output already exists: {output}. Choose another path or remove it explicitly."
        )
    output.mkdir(parents=True)
    try:
        for source_rel in definition["required"]:
            source = ROOT / source_rel
            if not source.exists():
                raise FileNotFoundError(f"Required server path does not exist: {source_rel}")
            copy_entry(source_rel, output, definition)
        for source_rel in definition["optional"]:
            source = ROOT / source_rel
            if source.exists():
                copy_entry(source_rel, output, definition)
        validate_output(output, definition)
        return write_build_manifest(output, definition)
    except Exception:
        shutil.rmtree(output)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output", type=Path, default=ROOT / "dist" / "cathsim-server")
    parser.add_argument("--zip", action="store_true", dest="make_zip")
    args = parser.parse_args()

    definition = load_manifest(args.manifest.resolve())
    output = args.output.resolve()
    result = build(output, definition)
    archive = None
    if args.make_zip:
        archive_path = output.with_suffix(".zip")
        if archive_path.exists():
            raise FileExistsError(f"Archive already exists: {archive_path}")
        archive = shutil.make_archive(str(output), "zip", output.parent, output.name)

    summary = {
        "status": "ok",
        "output": str(output),
        "archive": archive,
        "file_count": result["file_count"],
        "total_bytes": result["total_bytes"],
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"build_server_bundle: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
