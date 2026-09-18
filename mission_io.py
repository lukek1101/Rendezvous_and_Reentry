"""Shared serialization and case-index storage for the Python design tools.

Hash formatting is intentionally compatible with existing optimizer archives.
Each JSON replacement is atomic; concurrent index writers are not supported.
"""
import hashlib
import json
import os
from pathlib import Path
import tempfile
from datetime import datetime, timezone

import numpy as np


def to_jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {str(k): to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(v) for v in value]
    return value


def canonical_json_text(value):
    return json.dumps(to_jsonable(value), sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def short_hash(value, length=12):
    return hashlib.sha256(canonical_json_text(value).encode("utf-8")).hexdigest()[:length]


def safe_slug(value):
    slug = "".join(ch if ch.isalnum() or ch in "_-." else "-"
                   for ch in str(value).strip().lower()).strip("-")
    while "--" in slug:
        slug = slug.replace("--", "-")
    return slug or "case"


def relative_posix_path(path, base_dir):
    path, base_dir = Path(path).resolve(), Path(base_dir).resolve()
    try:
        return path.relative_to(base_dir).as_posix()
    except ValueError:
        return path.as_posix()


def write_json_atomic(path, value):
    """Do not leave a truncated latest alias/index if serialization fails."""
    text = json.dumps(to_jsonable(value), indent=2, sort_keys=True)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                     prefix=path.name + ".", suffix=".tmp", delete=False) as stream:
        temporary = Path(stream.name)
        try:
            stream.write(text)
        except BaseException:
            stream.close()
            temporary.unlink(missing_ok=True)
            raise
    try:
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def update_case_index(index_path, entry, source):
    """Upsert one case without silently discarding an unreadable index."""
    index_path = Path(index_path)
    index = json.loads(index_path.read_text(encoding="utf-8")) if index_path.exists() else {}
    if not isinstance(index, dict):
        raise ValueError(f"Invalid case index object: {index_path}")
    cases = index.get("cases", [])
    if not isinstance(cases, list) or any(not isinstance(case, dict) for case in cases):
        raise ValueError(f"Invalid cases list: {index_path}")
    cases = [case for case in cases if case.get("case_id") != entry["case_id"]]
    cases.append(entry)
    cases.sort(key=lambda case: str(case.get("created_at", "")))
    write_json_atomic(index_path, {
        "schema_version": 1, "source": source,
        "updated_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "latest_case_id": entry["case_id"], "cases": cases,
    })
