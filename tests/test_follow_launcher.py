from __future__ import annotations

import fcntl
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest


@pytest.fixture
def launcher(tmp_path: Path) -> tuple[list[str], dict[str, str], Path]:
    """Exercise real shell process ownership with fake hardware commands."""
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    source = Path(__file__).resolve().parents[1] / "scripts/start_gello_follow.sh"
    shutil.copy(source, scripts)
    serial = tmp_path / "serial"
    serial.touch()
    calib = tmp_path / "calib.json"
    calib.write_text(json.dumps({"joint_offsets": [0] * 6, "joint_signs": [1] * 6}))
    uv = tmp_path / "uv"
    uv.write_text(
        f"#!{sys.executable}\n"
        "import json, os, signal, sys, time\n"
        "args = sys.argv[1:]\n"
        "args = args[args.index('--project') + 2:]\n"
        "if args[0] == 'python':\n"
        "    os.execv(sys.executable, [sys.executable, *args[1:]])\n"
        "command = args[1]\n"
        "with open(os.environ['CALLS'], 'a') as f:\n"
        "    f.write(json.dumps({'args': args, 'pid': os.getpid()}) + '\\n')\n"
        "if command in ('doctor', 'follow-check'):\n"
        "    print('{\"ok\": true}')\n"
        "elif command == 'gripper-check':\n"
        "    if os.environ.get('FAIL_GRIPPER'): sys.exit(8)\n"
        "    print('{\"ok\": true}')\n"
        "elif command == 'follow-prepare':\n"
        "    if os.environ.get('FAIL_PREPARE'): sys.exit(9)\n"
        "    if os.environ.get('HOLD_PREPARE'):\n"
        "        signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))\n"
        "        while True: time.sleep(.02)\n"
        "    output = args[args.index('--output')+1]\n"
        "    with open(output, 'w') as f: f.write('{}')\n"
        "    print('{\"ok\": true}')\n"
        "elif command == 'follow-server':\n"
        "    if os.environ.get('FAIL_SERVER'):\n"
        "        sys.exit(7)\n"
        "    print('CR7 follow server: fake', flush=True)\n"
        "    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))\n"
        "    while True: time.sleep(.02)\n"
        "elif command == 'follow':\n"
        "    if os.environ.get('CHECK_STDIN'):\n"
        "        assert sys.stdin.read(1) == 'r', 'recording stdin lost'\n"
        "    if os.environ.get('HOLD_CLIENT'):\n"
        "        signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))\n"
        "        while True: time.sleep(.02)\n"
        "    sys.exit(int(os.environ.get('CLIENT_EXIT', '0')))\n"
    )
    uv.chmod(0o755)
    calls = tmp_path / "calls.jsonl"
    env = {
        **os.environ,
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "CALLS": str(calls),
    }
    env.pop("XCORE_FOLLOW_MAX_SPEED_DEG", None)
    env.pop("XCORE_PREPARE_SPEED", None)
    env.pop("XCORE_GRIPPER_HOST", None)
    cmd = [
        "bash",
        str(scripts / source.name),
        "--gello-port",
        str(serial),
        "--calib",
        str(calib),
    ]
    return cmd, env, calls


def read_calls(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines()]


def assert_children_stopped(calls: list[dict]) -> None:
    for call in calls:
        if call["args"][1] in ("follow-prepare", "follow-server", "follow"):
            with pytest.raises(ProcessLookupError):
                os.kill(call["pid"], 0)


@pytest.mark.parametrize("motion", [False, True])
def test_launcher_owns_and_cleans_sessions(launcher, motion: bool) -> None:
    cmd, env, path = launcher
    if motion:
        cmd += ["--enable-motion", "--yes"]
    result = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, result.stderr + result.stdout
    calls = read_calls(path)
    assert [c["args"][1] for c in calls] == (
        ["doctor", "follow-check"]
        + (["follow-prepare"] if motion else [])
        + ["follow-server", "follow"]
    )
    server = next(c for c in calls if c["args"][1] == "follow-server")
    client = next(c for c in calls if c["args"][1] == "follow")
    assert ("--enable-motion" in server["args"]) is motion
    assert ("--dry-run" in client["args"]) is not motion
    assert_children_stopped(calls)


@pytest.mark.parametrize(
    "configured, options, expected",
    [
        (None, [], "75"),
        ("10", [], "10"),
        ("10", ["--max-speed-deg", "20"], "20"),
        (None, ["--max-speed-deg", "75"], "75"),
    ],
)
def test_follow_speed_reaches_server_and_is_displayed(
    launcher, configured, options, expected
) -> None:
    cmd, env, path = launcher
    if configured is not None:
        env["XCORE_FOLLOW_MAX_SPEED_DEG"] = configured
    result = subprocess.run(
        cmd + options, env=env, capture_output=True, text=True, timeout=8
    )
    assert result.returncode == 0, result.stdout + result.stderr
    server = next(
        c["args"] for c in read_calls(path) if c["args"][1] == "follow-server"
    )
    assert server[server.index("--max-speed-deg") + 1] == expected
    assert f"速度上限 {expected} °/s" in result.stdout
    assert_children_stopped(read_calls(path))


@pytest.mark.parametrize("configured", ["0", "-1", "76", "nan", "abc"])
def test_invalid_follow_speed_fails_before_hardware_commands(launcher, configured):
    cmd, env, path = launcher
    result = subprocess.run(
        cmd,
        env={**env, "XCORE_FOLLOW_MAX_SPEED_DEG": configured},
        capture_output=True,
        timeout=8,
    )
    assert result.returncode != 0
    assert not path.exists()


def test_client_failure_preserved_and_server_stopped(launcher) -> None:
    cmd, env, path = launcher
    result = subprocess.run(
        cmd, env={**env, "CLIENT_EXIT": "3"}, capture_output=True, timeout=8
    )
    assert result.returncode == 3
    assert_children_stopped(read_calls(path))


def test_server_failure_prevents_client_start(launcher) -> None:
    cmd, env, path = launcher
    result = subprocess.run(
        cmd, env={**env, "FAIL_SERVER": "1"}, capture_output=True, timeout=8
    )
    assert result.returncode == 1
    calls = read_calls(path)
    assert "follow" not in [call["args"][1] for call in calls]
    assert_children_stopped(calls)


@pytest.mark.parametrize(
    "interrupt, exit_code", [(signal.SIGTERM, 143), (signal.SIGINT, 130)]
)
def test_termination_stops_both_process_groups(launcher, interrupt, exit_code) -> None:
    cmd, env, path = launcher
    process = subprocess.Popen(
        cmd,
        env={**env, "HOLD_CLIENT": "1"},
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        deadline = time.monotonic() + 5
        while not path.exists() or len(read_calls(path)) < 4:
            assert time.monotonic() < deadline, "client never started"
            time.sleep(0.02)
        process.send_signal(interrupt)
        stdout, stderr = process.communicate(timeout=8)
        assert process.returncode == exit_code, stdout + stderr
        assert_children_stopped(read_calls(path))
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def test_single_instance_lock_rejects_before_opening_devices(launcher) -> None:
    cmd, env, path = launcher
    with (path.parent / ".follow.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        result = subprocess.run(cmd, env=env, capture_output=True, timeout=8)
    assert result.returncode == 1
    assert not path.exists()


def test_launcher_forwards_gripper_options_only_to_unified_client(launcher) -> None:
    cmd, env, path = launcher
    options = ["--gripper-host", "127.0.0.1", "--gripper-force", "30"]
    result = subprocess.run(
        cmd + options, env=env, capture_output=True, text=True, timeout=8
    )
    assert result.returncode == 0, result.stderr + result.stdout
    calls = read_calls(path)
    assert calls[0]["args"][1] == "gripper-check"
    assert "--gripper-host" not in calls[3]["args"]
    assert calls[4]["args"][-4:] == options


@pytest.mark.parametrize("extra", [[], ["--skip-prepare"]])
def test_missing_gripper_blocks_arm_start_even_when_preparation_is_skipped(
    launcher, extra
):
    cmd, env, path = launcher
    result = subprocess.run(
        cmd + ["--enable-motion", "--yes", *extra],
        env={**env, "XCORE_GRIPPER_HOST": "127.0.0.1", "FAIL_GRIPPER": "1"},
        capture_output=True,
        text=True,
        timeout=8,
    )
    assert result.returncode == 1
    assert [c["args"][1] for c in read_calls(path)] == ["gripper-check"]
    assert "夹爪服务未就绪" in result.stderr


def test_arm_only_overrides_default_gripper_without_checking_service(launcher):
    cmd, env, path = launcher
    result = subprocess.run(
        cmd + ["--arm-only"],
        env={**env, "XCORE_GRIPPER_HOST": "127.0.0.1", "FAIL_GRIPPER": "1"},
        capture_output=True,
        timeout=8,
    )
    assert result.returncode == 0
    calls = read_calls(path)
    assert "gripper-check" not in [c["args"][1] for c in calls]
    assert "--gripper-host" not in calls[-1]["args"]


def test_arm_only_conflicts_with_explicit_gripper_before_hardware_commands(launcher):
    cmd, env, path = launcher
    result = subprocess.run(
        cmd + ["--arm-only", "--gripper-host", "127.0.0.1"],
        env=env,
        capture_output=True,
        timeout=8,
    )
    assert result.returncode == 1
    assert not path.exists()


@pytest.mark.parametrize(
    "configured, options, expected",
    [
        (None, [], "4000"),
        ("800", [], "800"),
        ("800", ["--prepare-speed", "1200"], "1200"),
    ],
)
def test_alignment_speed_independent_of_realtime_limit(
    launcher, configured, options, expected
):
    cmd, env, path = launcher
    if configured:
        env["XCORE_PREPARE_SPEED"] = configured
    result = subprocess.run(
        cmd + ["--enable-motion", "--yes", *options],
        env=env,
        capture_output=True,
        timeout=8,
    )
    assert result.returncode == 0
    calls = read_calls(path)
    prepare = next(c["args"] for c in calls if c["args"][1] == "follow-prepare")
    server = next(c["args"] for c in calls if c["args"][1] == "follow-server")
    assert prepare[prepare.index("--speed") + 1] == expected
    assert server[server.index("--max-speed-deg") + 1] == "75"


def test_state_display_flag_only_reaches_client(launcher):
    cmd, env, path = launcher
    result = subprocess.run(
        cmd + ["--show-state"], env=env, capture_output=True, timeout=8
    )
    assert result.returncode == 0
    calls = read_calls(path)
    assert "--show-state" not in calls[-2]["args"]
    assert "--show-state" in calls[-1]["args"]


def test_recording_client_inherits_keyboard_input(launcher) -> None:
    cmd, env, path = launcher
    result = subprocess.run(
        cmd
        + [
            "--enable-motion",
            "--yes",
            "--gripper-host",
            "127.0.0.1",
            "--raw-data-root",
            str(path.parent / "raw"),
            "--start-recording",
        ],
        env={**env, "CHECK_STDIN": "1"},
        input="r",
        text=True,
        capture_output=True,
        timeout=8,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "--start-recording" in next(
        c["args"] for c in read_calls(path) if c["args"][1] == "follow"
    )


def test_failed_preparation_never_starts_rt_server_or_client(launcher):
    cmd, env, path = launcher
    result = subprocess.run(
        cmd + ["--enable-motion", "--yes"],
        env={**env, "FAIL_PREPARE": "1"},
        capture_output=True,
        timeout=8,
    )
    assert result.returncode == 9
    assert [c["args"][1] for c in read_calls(path)] == [
        "doctor",
        "follow-check",
        "follow-prepare",
    ]
    assert_children_stopped(read_calls(path))


def test_skip_prepare_preserves_manual_alignment_mode(launcher):
    cmd, env, path = launcher
    result = subprocess.run(
        cmd + ["--enable-motion", "--yes", "--skip-prepare"],
        env=env,
        capture_output=True,
        timeout=8,
    )
    assert result.returncode == 0
    assert "follow-prepare" not in [c["args"][1] for c in read_calls(path)]


def test_first_zero_calibration_uses_sh_entry_without_existing_calibration(launcher):
    cmd, env, path = launcher
    (path.parent / "calib.json").unlink()
    result = subprocess.run(
        cmd + ["--enable-motion", "--yes", "--calibrate-zero"],
        env=env,
        capture_output=True,
        timeout=8,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    calls = read_calls(path)
    assert [c["args"][1] for c in calls] == [
        "doctor",
        "follow-prepare",
        "follow-server",
        "follow",
    ]
    assert "--calibrate-zero" in calls[1]["args"]


def test_interrupt_during_preparation_stops_owned_process_before_rt_start(launcher):
    cmd, env, path = launcher
    process = subprocess.Popen(
        cmd + ["--enable-motion", "--yes"],
        env={**env, "HOLD_PREPARE": "1"},
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    try:
        deadline = time.monotonic() + 5
        while not path.exists() or len(read_calls(path)) < 3:
            assert time.monotonic() < deadline
            time.sleep(0.02)
        process.send_signal(signal.SIGTERM)
        process.communicate(timeout=8)
        assert process.returncode == 143
        calls = read_calls(path)
        assert [c["args"][1] for c in calls] == [
            "doctor",
            "follow-check",
            "follow-prepare",
        ]
        assert_children_stopped(calls)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
