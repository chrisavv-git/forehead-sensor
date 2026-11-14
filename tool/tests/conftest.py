"""Fixtures shared by the tool tests: the simulator is built once, and every test gets its own config and data dir."""
import os
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'tool'))
from fhs import store  # noqa: E402


@pytest.fixture(scope='session', autouse=True)
def build_sim():
    subprocess.run(['make', '-s', '-C', os.path.join(ROOT, 'firmware', 'host'), 'fhsim'], check=True)


@pytest.fixture
def env(tmp_path, monkeypatch):
    e = dict(os.environ, FHS_CONFIG=str(tmp_path / 'cfg.json'), FHS_DATA=str(tmp_path / 'data'),
             PYTHONPATH=os.path.join(ROOT, 'tool'))
    monkeypatch.setattr(store, 'CONFIG', e['FHS_CONFIG'])
    monkeypatch.setattr(store, 'DATA', e['FHS_DATA'])
    return e
