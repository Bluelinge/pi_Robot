from pathlib import Path

from pi_robot.storage import load_json, save_json


def test_save_and_load_json(tmp_path: Path) -> None:
    path = tmp_path / "payload.json"
    payload = {"ok": True, "value": 3}
    save_json(path, payload)
    assert load_json(path, {}) == payload
