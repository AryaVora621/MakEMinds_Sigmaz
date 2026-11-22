"""Shared pytest fixtures and markers for the field_sim test suite.

Model directory resolution: env FIELD_SIM_MODEL_DIR if set, otherwise
sim.loader.default_model_dir().  sim is imported lazily so that collection
still works while the package is being written.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SYNTHETIC_SOURCE = "synthetic"


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "real_model: needs the real (non-synthetic) field model; skipped when the model source is 'synthetic'",
    )
    config.addinivalue_line("markers", "slow: slow test (e.g. loads every mesh)")


def _resolve_model_dir() -> Path:
    env = os.environ.get("FIELD_SIM_MODEL_DIR")
    if env:
        return Path(env)
    from sim.loader import default_model_dir

    return Path(default_model_dir())


def _declared_source(model_dir: Path):
    """Return the 'source' string declared in model.json, or None."""
    model_json = Path(model_dir) / "model.json"
    if not model_json.is_file():
        return None
    with open(model_json, encoding="utf-8") as fh:
        return json.load(fh).get("source")


def pytest_runtest_setup(item):
    if item.get_closest_marker("real_model") is None:
        return
    if _declared_source(_resolve_model_dir()) == SYNTHETIC_SOURCE:
        pytest.skip("real_model test skipped: model source is 'synthetic'")


@pytest.fixture(scope="session")
def model_dir() -> Path:
    return _resolve_model_dir()


@pytest.fixture(scope="session")
def model(model_dir):
    from sim.loader import load_model

    return load_model(model_dir)


@pytest.fixture(scope="session")
def model_source(model, model_dir):
    """Declared model source: model.meta['source'] if present, else model.json."""
    meta = getattr(model, "meta", None) or {}
    return meta.get("source") or _declared_source(model_dir)


@pytest.fixture
def field(model):
    # Function-scoped: Field holds mutable hive state.
    from sim.field import Field

    return Field(model)
