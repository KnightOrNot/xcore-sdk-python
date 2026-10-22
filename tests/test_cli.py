import json
import subprocess

import pytest

from xcore_sdk_python import cli


@pytest.mark.parametrize(
    "arguments",
    [
        ["status", "--ip", "invalid"],
        ["status", "--timeout", "0"],
        ["movej", "--joints", "0", "0", "nan", "0", "0", "0"],
        ["move-joint", "--joint", "7", "--delta-deg", "1"],
        ["move-joint", "--joint", "1", "--delta-deg", "20"],
        ["monitor", "--duration", "100"],
        ["network", "configure"],
    ],
)
def test_invalid_parameters_never_connect(monkeypatch, arguments):
    monkeypatch.setattr(cli, "isolated", lambda _: pytest.fail("must not connect"))
    with pytest.raises(SystemExit) as exc:
        cli.main(arguments)
    assert exc.value.code == 2


def test_output_is_machine_readable_and_saved_without_overwrite(
    monkeypatch, tmp_path, capsys
):
    monkeypatch.setattr(
        cli, "isolated", lambda _: {"ok": True, "result": {"sdk": "0.7.1"}}
    )
    output = tmp_path / "result.json"
    assert cli.main(["doctor", "--output", str(output)]) == 0
    assert json.loads(output.read_text()) == json.loads(capsys.readouterr().out)
    with pytest.raises(SystemExit):
        cli.main(["doctor", "--output", str(output)])


def test_default_non_realtime_local_ip_is_empty(monkeypatch):
    monkeypatch.delenv("XCORE_LOCAL_IP", raising=False)

    def inspect(options):
        assert options["local_ip"] == ""
        return {"ok": True}

    monkeypatch.setattr(cli, "isolated", inspect)
    assert cli.main(["doctor"]) == 0


def test_timeout_returns_124_and_does_not_save_success(monkeypatch, tmp_path):
    def timeout(_):
        raise subprocess.TimeoutExpired("worker", 1)

    monkeypatch.setattr(cli, "isolated", timeout)
    target = tmp_path / "report.json"
    assert cli.main(["status", "--output", str(target)]) == 124
    assert not target.exists()


def test_query_error_returns_one(monkeypatch, capsys):
    monkeypatch.setattr(
        cli, "isolated", lambda _: {"ok": False, "error": "not connected"}
    )
    assert cli.main(["status"]) == 1
    assert json.loads(capsys.readouterr().err)["ok"] is False


@pytest.mark.parametrize(
    "arguments",
    [
        ["movej", "--joints", "0", "0", "0", "0", "0", "0"],
        ["move-joint", "--joint", "6", "--delta-deg", "1"],
    ],
)
@pytest.mark.parametrize("extra, expected", [([], 1000), (["--speed", "50"], 50)])
def test_motion_cli_default_and_explicit_speed(monkeypatch, arguments, extra, expected):
    def inspect(options):
        assert options["speed"] == expected
        return {"ok": True}

    monkeypatch.setattr(cli, "isolated", inspect)
    assert cli.main(arguments + extra) == 0


def test_worker_timeout_leaves_no_background_process(monkeypatch):
    real_popen = subprocess.Popen
    children = []

    def sleeping_worker(command, **kwargs):
        child = real_popen(
            [cli.sys.executable, "-c", "import time; time.sleep(5)"], **kwargs
        )
        children.append(child)
        return child

    monkeypatch.setattr(cli.subprocess, "Popen", sleeping_worker)
    with pytest.raises(subprocess.TimeoutExpired):
        cli.isolated({"command": "doctor", "timeout": 0.05})
    assert all(child.poll() is not None for child in children)


def test_capture_pose_writes_schema_one_without_motion(monkeypatch, tmp_path):
    monkeypatch.setattr(
        cli,
        "isolated",
        lambda options: {
            "ok": True,
            "result": {"rad": [0.1] * 6, "deg": [5.0] * 6},
        },
    )
    path = tmp_path / "session" / "exit_pose.json"
    assert cli.main(["capture-pose", "--save", str(path)]) == 0
    data = json.loads(path.read_text())
    assert data["schema_version"] == 1
    assert data["robot_ip"] == "192.168.2.160"
    assert data["joints_rad"] == [0.1] * 6
    assert data["joints_deg"] == [5.0] * 6
    assert "captured_at" in data
    with pytest.raises(SystemExit) as exc:
        cli.main(["capture-pose", "--save", str(path)])
    assert exc.value.code == 2


def test_capture_pose_zero_does_not_connect(monkeypatch, tmp_path):
    monkeypatch.setattr(cli, "isolated", lambda _: pytest.fail("must not connect"))
    path = tmp_path / "zero.json"
    assert cli.main(["capture-pose", "--save", str(path), "--zero"]) == 0
    data = json.loads(path.read_text())
    assert data["joints_rad"] == [0.0] * 6
    assert data["joints_deg"] == [0.0] * 6
