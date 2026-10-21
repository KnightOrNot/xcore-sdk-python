from __future__ import annotations

from pathlib import Path

import pytest

from xcore_sdk_python.cli import config_path, parser
from xcore_sdk_python.configuration import load_config


def write_config(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "default.toml"
    path.write_text(text)
    return path


def test_default_config_matches_previous_cli_defaults() -> None:
    config = load_config()
    assert config.robot.ip == "192.168.2.160"
    assert config.server.max_speed_deg == 3.0
    assert config.gripper.port == 5005
    assert config.follow.hz == 50.0
    assert config.gello.signs == (1, 1, 1, 1, 1, 1)
    for command in ("follow-prepare", "return-pose"):
        assert parser(config).parse_args([command]).speed == 1000.0


def test_config_overrides_cli_defaults(tmp_path: Path) -> None:
    path = write_config(
        tmp_path,
        """
[robot]
ip = "10.0.0.2"
timeout_s = 3.5

[server]
max_speed_deg = 42.0

[gello]
signs = [-1, 1, -1, 1, -1, 1]
""",
    )
    config = load_config(path)
    assert config.robot.ip == "10.0.0.2"
    assert config.server.max_speed_deg == 42.0
    assert config.gello.signs == (-1, 1, -1, 1, -1, 1)
    args = parser(config).parse_args(["follow-server"])
    assert args.ip == "10.0.0.2"
    assert args.timeout == 3.5
    assert args.max_speed_deg == 42.0


def test_explicit_cli_value_wins(tmp_path: Path) -> None:
    path = write_config(tmp_path, '[robot]\nip = "10.0.0.2"\n')
    args = parser(load_config(path)).parse_args(["status", "--ip", "10.9.9.9"])
    assert args.ip == "10.9.9.9"


def test_environment_still_overrides_config(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setenv("XCORE_ROBOT_IP", "10.1.1.1")
    path = write_config(tmp_path, '[robot]\nip = "10.0.0.2"\n')
    assert parser(load_config(path)).parse_args(["status"]).ip == "10.1.1.1"


def test_unknown_section_and_option_are_rejected(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="unknown configuration sections"):
        load_config(write_config(tmp_path, "[nope]\nx = 1\n"))
    with pytest.raises(ValueError, match="unknown robot options"):
        load_config(write_config(tmp_path, "[robot]\nnope = 1\n"))


def test_invalid_values_are_rejected(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="gello.signs"):
        load_config(write_config(tmp_path, "[gello]\nsigns = [1, 0, 1, 1, 1, 1]\n"))
    with pytest.raises(ValueError, match="max_speed_deg"):
        load_config(write_config(tmp_path, "[server]\nmax_speed_deg = 80\n"))
    with pytest.raises(ValueError, match="positive"):
        load_config(write_config(tmp_path, "[follow]\nhz = 0\n"))


def test_config_path_scans_both_forms() -> None:
    assert config_path(["--config", "a.toml", "status"]) == Path("a.toml")
    assert config_path(["status", "--config=b.toml"]) == Path("b.toml")
    assert config_path(["status"]) is None
    assert config_path(["--config", "a.toml", "status", "--config=b.toml"]) == Path(
        "b.toml"
    )


@pytest.mark.parametrize(
    "arguments",
    [
        ["--config", "config/default.toml", "doctor"],
        ["doctor", "--config", "config/default.toml"],
    ],
)
def test_config_option_before_or_after_command(arguments):
    args = parser().parse_args(arguments)
    assert args.config == Path("config/default.toml")


@pytest.mark.parametrize(
    "text",
    [
        "[server]\nport = true\n",
        "[gripper]\nhz = nan\n",
        "[calibration]\nsamples = 2.5\n",
        "[robot]\nip = 123\n",
        "[recording]\ntask = []\n",
    ],
)
def test_invalid_config_types_fail_cleanly(tmp_path, text):
    with pytest.raises(ValueError):
        load_config(write_config(tmp_path, text))
