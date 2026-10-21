"""JSON configuration with the provenance of every value attached to it.

Python 3.10 here has neither `tomllib` nor `pyyaml`, so the files are plain JSON
and each leaf carries its own source instead of a comment:

    {"value": 0.058, "unit": "m", "source": "site_measurement", "note": "..."}

`estimated` and `idealised` leaves are listed at startup and recorded in the
report, so a value without evidence cannot quietly become a result.
"""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCES = {
    "site_measurement",
    "official_urdf",
    "measured_in_sim",
    "design_choice",
    "estimated",
    "idealised",
}
# Sources that have to announce themselves.
UNEVIDENCED = ("estimated", "idealised")


class Config:
    """Dotted-path access to one JSON config, with its unevidenced entries."""

    def __init__(self, path):
        self.path = Path(path)
        self.entries = {}
        self._walk(json.loads(self.path.read_text()), ())

    def _walk(self, node, prefix):
        if isinstance(node, dict) and "value" in node:
            source = node.get("source")
            if source not in SOURCES:
                raise ValueError(
                    f"{self.path.name}:{'.'.join(prefix)} has source {source!r}, "
                    f"expected one of {sorted(SOURCES)}"
                )
            self.entries[".".join(prefix)] = node
            return
        if not isinstance(node, dict):
            raise ValueError(
                f"{self.path.name}:{'.'.join(prefix)} is a bare {type(node).__name__}; "
                "every leaf needs value/unit/source/note"
            )
        for key, child in node.items():
            self._walk(child, prefix + (key,))

    def __getitem__(self, path):
        try:
            return self.entries[path]["value"]
        except KeyError:
            raise KeyError(f"{self.path.name} has no entry {path!r}") from None

    def unit(self, path):
        return self.entries[path].get("unit")

    def override(self, **values):
        """A copy with some values replaced, for sensitivity runs.

        The replacement keeps the original source and note and appends the
        override, so a swept value cannot be mistaken for the configured one.
        """
        import copy

        clone = copy.copy(self)
        clone.entries = {k: dict(v) for k, v in self.entries.items()}
        for path, value in values.items():
            dotted = path.replace("__", ".")
            if dotted not in clone.entries:
                raise KeyError(f"{self.path.name} has no entry {dotted!r}")
            entry = clone.entries[dotted]
            entry["note"] = f"{entry.get('note', '')} [overridden: {value}]".strip()
            entry["value"] = value
        return clone

    def unevidenced(self):
        """Entries whose value has no measurement behind it, worst first."""
        return [
            {"path": path, "value": e["value"], "unit": e.get("unit"),
             "source": e["source"], "note": e.get("note", "")}
            for path, e in sorted(self.entries.items())
            if e["source"] in UNEVIDENCED
        ]


def load(name):
    return Config(ROOT / "config" / f"{name}.json")
