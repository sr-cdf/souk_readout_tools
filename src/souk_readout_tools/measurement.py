"""On-disk record for a measurement run: a manifest plus saved data files.

A run is a directory containing ``measurement.json`` (the manifest) alongside
the data/plots/analysis files it points at.  The manifest records, for each
step, the swept parameters, the values read back from the hardware, free-form
metadata, and pointers to the saved artifacts.

The data model is plain dataclasses:

    MeasurementRun                one run (kind, parameters, metadata, steps)
      └ MeasurementStep           one point on the swept axis
          └ MeasurementArtifact   a saved file (sweep ``.npz``, plot ``.png``, ...)

``MeasurementStore`` reads and writes the files; it does not orchestrate the
measurement.  The acquisition loop lives with the measurement that uses it
(see :func:`souk_readout_tools.power_sweep.run_power_sweep`).
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

import numpy as np


MANIFEST_FILE = "measurement.json"
SCHEMA_VERSION = 1


class RunStatus:
    PENDING = "pending"
    RUNNING = "running"
    SUCCESS = "success"
    FAILED = "failed"


class ArtifactKind:
    SWEEP = "sweep"
    TIMESTREAM = "timestream"
    ACCUMULATOR_SNAPSHOT = "accumulator_snapshot"
    ADC_SNAPSHOT = "adc_snapshot"
    DAC_SNAPSHOT = "dac_snapshot"
    FIT_RESULTS = "fit_results"
    SUMMARY_TABLE = "summary_table"
    PLOT = "plot"
    CONFIG_SNAPSHOT = "config_snapshot"
    SYSTEM_INFO = "system_info"
    EVENT_LOG = "event_log"
    OTHER = "other"


class ArtifactRole:
    RAW = "raw"
    DATA = "data"
    ANALYSIS = "analysis"
    DIAGNOSTIC = "diagnostic"
    PROVENANCE = "provenance"
    PLOT = "plot"


def timestamp() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S %z")


def _jsonify(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _jsonify(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonify(item) for item in value]
    if isinstance(value, float):
        return value if np.isfinite(value) else None
    return value


def _relative_to(path: Path, root: Path) -> str:
    try:
        return str(path.resolve().relative_to(root.resolve()))
    except ValueError:
        return str(path)


@dataclass
class MeasurementArtifact:
    name: str
    kind: str
    path: str
    format: str
    role: str = ArtifactRole.DATA
    metadata: Dict[str, Any] = field(default_factory=dict)
    created: str = field(default_factory=timestamp)

    def absolute_path(self, root: Path | str) -> Path:
        """Absolute path to the artifact file, resolving a relative ``path``
        under the run directory ``root``."""
        path = Path(self.path)
        return path if path.is_absolute() else Path(root) / path

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "kind": self.kind,
            "path": self.path,
            "format": self.format,
            "role": self.role,
            "metadata": _jsonify(self.metadata),
            "created": self.created,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "MeasurementArtifact":
        """Rebuild an artifact from its :meth:`to_dict` mapping ``data``."""
        return cls(
            name=data["name"],
            kind=data["kind"],
            path=data["path"],
            format=data.get("format", ""),
            role=data.get("role", ArtifactRole.DATA),
            metadata=dict(data.get("metadata", {})),
            created=data.get("created", timestamp()),
        )


@dataclass
class MeasurementStep:
    index: int
    axis: Dict[str, Any] = field(default_factory=dict)
    readback: Dict[str, Any] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)
    artifacts: List[MeasurementArtifact] = field(default_factory=list)
    status: str = RunStatus.PENDING
    started: Optional[str] = None
    finished: Optional[str] = None
    error: Optional[str] = None

    def add_artifact(self, artifact: MeasurementArtifact) -> MeasurementArtifact:
        """Attach ``artifact`` to this step and return it."""
        self.artifacts.append(artifact)
        return artifact

    def artifact(
        self,
        kind: str,
        name: Optional[str] = None,
    ) -> Optional[MeasurementArtifact]:
        """First artifact of the given ``kind`` (and ``name`` if given), or None."""
        for artifact in self.artifacts:
            if artifact.kind == kind and (name is None or artifact.name == name):
                return artifact
        return None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "index": int(self.index),
            "axis": _jsonify(self.axis),
            "readback": _jsonify(self.readback),
            "metadata": _jsonify(self.metadata),
            "artifacts": [artifact.to_dict() for artifact in self.artifacts],
            "status": self.status,
            "started": self.started,
            "finished": self.finished,
            "error": self.error,
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "MeasurementStep":
        """Rebuild a step from its :meth:`to_dict` mapping ``data``."""
        return cls(
            index=int(data["index"]),
            axis=dict(data.get("axis", data.get("parameters", {}))),
            readback=dict(data.get("readback", {})),
            metadata=dict(data.get("metadata", {})),
            artifacts=[
                MeasurementArtifact.from_dict(item)
                for item in data.get("artifacts", [])
            ],
            status=data.get("status", RunStatus.PENDING),
            started=data.get("started"),
            finished=data.get("finished"),
            error=data.get("error"),
        )


@dataclass
class MeasurementRun:
    kind: str
    root: Path | str
    parameters: Dict[str, Any] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)
    steps: List[MeasurementStep] = field(default_factory=list)
    artifacts: List[MeasurementArtifact] = field(default_factory=list)
    provenance: Dict[str, Any] = field(default_factory=dict)
    schema_version: int = SCHEMA_VERSION
    created: str = field(default_factory=timestamp)
    finished: Optional[str] = None
    status: str = RunStatus.PENDING
    error: Optional[str] = None

    def __post_init__(self) -> None:
        self.root = Path(self.root).resolve()

    @property
    def manifest_path(self) -> Path:
        return self.root / MANIFEST_FILE

    def store(self) -> "MeasurementStore":
        return MeasurementStore(self.root)

    def add_step(self, step: MeasurementStep) -> MeasurementStep:
        """Append ``step`` to the run and return it."""
        self.steps.append(step)
        return step

    def add_artifact(self, artifact: MeasurementArtifact) -> MeasurementArtifact:
        """Attach a run-level ``artifact`` (not tied to a step) and return it."""
        self.artifacts.append(artifact)
        return artifact

    def to_dict(self) -> Dict[str, Any]:
        return {
            "kind": self.kind,
            "schema_version": self.schema_version,
            "root": str(self.root),
            "created": self.created,
            "finished": self.finished,
            "status": self.status,
            "error": self.error,
            "parameters": _jsonify(self.parameters),
            "metadata": _jsonify(self.metadata),
            "provenance": _jsonify(self.provenance),
            "artifacts": [artifact.to_dict() for artifact in self.artifacts],
            "steps": [step.to_dict() for step in self.steps],
        }

    def write_manifest(self) -> Path:
        self.root.mkdir(parents=True, exist_ok=True)
        with self.manifest_path.open("w", encoding="utf-8") as handle:
            json.dump(self.to_dict(), handle, indent=2)
        return self.manifest_path

    @classmethod
    def from_dict(
        cls,
        data: Mapping[str, Any],
        root: Optional[Path | str] = None,
    ) -> "MeasurementRun":
        """Rebuild a run from its :meth:`to_dict` mapping ``data``.

        ``root`` overrides the run directory recorded in ``data`` (used when
        loading a manifest that was moved).
        """
        return cls(
            kind=data["kind"],
            root=root or data.get("root", "."),
            parameters=dict(data.get("parameters", {})),
            metadata=dict(data.get("metadata", {})),
            steps=[
                MeasurementStep.from_dict(item)
                for item in data.get("steps", [])
            ],
            artifacts=[
                MeasurementArtifact.from_dict(item)
                for item in data.get("artifacts", [])
            ],
            provenance=dict(data.get("provenance", {})),
            schema_version=int(data.get("schema_version", SCHEMA_VERSION)),
            created=data.get("created", timestamp()),
            finished=data.get("finished"),
            status=data.get("status", RunStatus.PENDING),
            error=data.get("error"),
        )

    @classmethod
    def load(cls, path: Path | str) -> "MeasurementRun":
        """Load a run from ``path`` (a run directory or a manifest file)."""
        path = Path(path).resolve()
        manifest = path / MANIFEST_FILE if path.is_dir() else path
        with manifest.open("r", encoding="utf-8") as handle:
            return cls.from_dict(json.load(handle), root=manifest.parent)


class MeasurementStore:
    """Read and write the files that make up a run directory."""

    def __init__(self, root: Path | str):
        self.root = Path(root).resolve()

    def ensure_layout(self) -> None:
        for dirname in ("data", "analysis", "plots", "logs"):
            (self.root / dirname).mkdir(parents=True, exist_ok=True)

    def path(self, path: Path | str) -> Path:
        """Resolve ``path`` against the store root if it is relative."""
        path = Path(path)
        return path if path.is_absolute() else self.root / path

    def save_json(self, path: Path | str, data: Any) -> str:
        """Write ``data`` as JSON to ``path``; return the path relative to root.

        ``path`` may be absolute or relative to the store root; ``data`` is
        run through :func:`_jsonify` so numpy values serialise.
        """
        path = self.path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as handle:
            json.dump(_jsonify(data), handle, indent=2)
        return _relative_to(path, self.root)

    def load_json(self, path: Path | str) -> Any:
        """Load JSON from ``path`` (absolute or relative to the store root)."""
        with self.path(path).open("r", encoding="utf-8") as handle:
            return json.load(handle)

    def save_npz(self, path: Path | str, data: Mapping[str, Any]) -> str:
        """Save mapping ``data`` to ``.npz`` at ``path``: array values go in
        directly, everything else is JSON-encoded into a single
        ``_metadata_json`` entry.  Returns the path relative to the root."""
        path = self.path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        arrays = {}
        metadata = {}
        for key, value in data.items():
            if isinstance(value, np.ndarray):
                arrays[str(key)] = value
            elif isinstance(value, np.generic):
                metadata[str(key)] = value.item()
            else:
                metadata[str(key)] = _jsonify(value)
        arrays["_metadata_json"] = np.array(json.dumps(metadata))
        np.savez_compressed(path, **arrays)
        return _relative_to(path, self.root)

    def load_npz(self, path: Path | str) -> Dict[str, Any]:
        """Load an ``.npz`` written by :meth:`save_npz` from ``path``, merging
        the arrays and the decoded ``_metadata_json`` into one dict."""
        with np.load(self.path(path), allow_pickle=False) as archive:
            data = {}
            if "_metadata_json" in archive:
                data.update(json.loads(str(archive["_metadata_json"].item())))
            for key in archive.files:
                if key != "_metadata_json":
                    data[key] = archive[key].copy()
        return data

    def make_artifact(
        self,
        *,
        name: str,
        kind: str,
        relative_path: Path | str,
        format: str,
        role: str = ArtifactRole.DATA,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> MeasurementArtifact:
        """Build a :class:`MeasurementArtifact` record (writes nothing).

        Parameters
        ----------
        name : str
            Short identifier for the artifact.
        kind : str
            One of the :class:`ArtifactKind` values (e.g. ``"sweep"``).
        relative_path : str or Path
            Path to the file, relative to the run root.
        format : str
            File format tag, e.g. ``"npz"`` or ``"json"``.
        role : str, optional
            One of the :class:`ArtifactRole` values (default ``"data"``).
        metadata : dict, optional
            Extra free-form metadata stored alongside the pointer.
        """
        return MeasurementArtifact(
            name=name,
            kind=kind,
            path=str(relative_path),
            format=format,
            role=role,
            metadata=dict(metadata or {}),
        )

    def save_step_npz_artifact(
        self,
        run: MeasurementRun,
        step: MeasurementStep,
        *,
        name: str,
        kind: str,
        relative_path: Path | str,
        data: Mapping[str, Any],
        role: str = ArtifactRole.DATA,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> MeasurementArtifact:
        """Save ``data`` as ``.npz`` and record it as an artifact on ``step``.

        ``run`` is accepted for symmetry (the artifact is attached to ``step``,
        which already belongs to the run); ``name``/``kind``/``relative_path``/
        ``role``/``metadata`` are passed through to :meth:`make_artifact`, and
        ``data`` is the mapping written via :meth:`save_npz`.
        """
        saved_path = self.save_npz(relative_path, data)
        artifact = self.make_artifact(
            name=name,
            kind=kind,
            relative_path=saved_path,
            format="npz",
            role=role,
            metadata=metadata,
        )
        step.add_artifact(artifact)
        return artifact

    def load_artifact_data(self, artifact: MeasurementArtifact) -> Any:
        """Load and return the data for ``artifact`` (``.npz`` or ``.json``)."""
        if artifact.format == "npz":
            return self.load_npz(artifact.path)
        if artifact.format == "json":
            return self.load_json(artifact.path)
        raise ValueError(f"Unsupported artifact format {artifact.format!r}.")


def save_system_info(run, store, client, label, sections="all"):
    """Save a ``client.get_info()`` snapshot as a provenance artifact.

    Best-effort: a client without ``get_info``, or a failure fetching it, is
    recorded as a warning in ``run.metadata`` rather than raised, so it never
    aborts a measurement.

    Parameters
    ----------
    run : MeasurementRun
        Run to attach the artifact and provenance entry to.
    store : MeasurementStore
        Store used to write the JSON file.
    client : object or None
        Anything with a ``get_info(sections)`` method; ``None`` is a no-op.
    label : str
        Tag distinguishing snapshots, e.g. ``"start"`` or ``"end"``.
    sections : optional
        Forwarded to ``client.get_info`` to select sections (default
        ``"all"``).
    """
    if client is None or not hasattr(client, "get_info"):
        return
    try:
        info = client.get_info(sections)
    except Exception as exc:  # best-effort provenance only
        run.metadata.setdefault("warnings", []).append(
            f"Could not capture system info {label}: {exc}"
        )
        return
    rel_path = store.save_json(f"system_info_{label}.json", info)
    artifact = store.make_artifact(
        name=f"system_info_{label}",
        kind=ArtifactKind.SYSTEM_INFO,
        relative_path=rel_path,
        format="json",
        role=ArtifactRole.PROVENANCE,
        metadata={"label": label, "sections": sections},
    )
    run.add_artifact(artifact)
    run.provenance[f"system_info_{label}"] = rel_path


__all__ = [
    "MANIFEST_FILE",
    "SCHEMA_VERSION",
    "RunStatus",
    "ArtifactKind",
    "ArtifactRole",
    "MeasurementArtifact",
    "MeasurementStep",
    "MeasurementRun",
    "MeasurementStore",
    "save_system_info",
    "timestamp",
]
