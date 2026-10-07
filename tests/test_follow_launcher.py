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
        "elif command == 'follow-server':\n"
        "    if os.environ.get('FAIL_SERVER'):\n"
        "        sys.exit(7)\n"
        "    print('CR7 follow server: fake', flush=True)\n"
        "    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))\n"
        "    while True: time.sleep(.02)\n"
        "elif command == 'follow':\n"
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
        if call["args"][1] in ("follow-server", "follow"):
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
    assert [c["args"][1] for c in calls] == [
        "doctor",
        "follow-check",
        "follow-server",
        "follow",
    ]
    assert ("--enable-motion" in calls[2]["args"]) is motion
    assert ("--dry-run" in calls[3]["args"]) is not motion
    assert_children_stopped(calls)


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
    assert "--gripper-host" not in calls[2]["args"]
    assert calls[3]["args"][-4:] == options
