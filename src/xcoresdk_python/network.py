"""Linux diagnostics and explicit, temporary NetworkManager address changes."""

from __future__ import annotations

import ipaddress
import json
import socket
import subprocess
import time
from typing import Any

from .exceptions import XCoreError


def system_command(*args: str) -> str:
    try:
        completed = subprocess.run(args, capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise XCoreError(f"{args[0]}: {exc}") from exc
    if completed.returncode:
        raise XCoreError(completed.stderr.strip() or f"{args[0]} failed")
    return completed.stdout


def diagnose(ip: str, *, probe_timeout: float = 2) -> dict[str, Any]:
    report: dict[str, Any] = {"robot_ip": ip, "sdk_port": 6666}
    for key, command in (
        ("addresses", ("ip", "-j", "addr", "show")),
        ("route", ("ip", "-j", "route", "get", ip)),
        ("neighbors", ("ip", "-j", "neigh", "show")),
    ):
        try:
            report[key] = json.loads(system_command(*command))
        except (XCoreError, json.JSONDecodeError) as exc:
            report[key] = {"error": str(exc)}
    try:
        with socket.create_connection((ip, 6666), timeout=probe_timeout):
            report["sdk_port_reachable"] = True
    except OSError as exc:
        report["sdk_port_reachable"] = False
        report["connection_error"] = str(exc)
    report["note"] = "TCP reachability is not a completed SDK handshake"
    routes = report.get("route")
    addresses = report.get("addresses")
    if isinstance(routes, list) and isinstance(addresses, list) and routes:
        device = routes[0].get("dev")
        report["route_interface"] = device
        report["same_subnet_on_route_interface"] = any(
            ipaddress.IPv4Address(ip)
            in ipaddress.IPv4Interface(f"{item['local']}/{item['prefixlen']}").network
            for address in addresses
            if address.get("ifname") == device
            for item in address.get("addr_info", [])
            if item.get("family") == "inet"
        )
        report["addresses"] = [
            {
                "interface": address["ifname"],
                "state": address.get("operstate"),
                "ipv4": [
                    f"{item['local']}/{item['prefixlen']}"
                    for item in address.get("addr_info", [])
                    if item.get("family") == "inet"
                ],
            }
            for address in addresses
        ]
    return report


def configure_address(interface: str, address: str, *, remove: bool = False) -> dict:
    if not interface or interface.startswith("-"):
        raise ValueError("Specify an existing wired interface")
    parsed = ipaddress.IPv4Interface(address)
    if parsed.ip in (parsed.network.network_address, parsed.network.broadcast_address):
        raise ValueError("PC address cannot be the network or broadcast address")
    system_command(
        "nmcli",
        "device",
        "modify",
        interface,
        "-ipv4.addresses" if remove else "+ipv4.addresses",
        str(parsed),
    )
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        devices = json.loads(
            system_command("ip", "-j", "addr", "show", "dev", interface)
        )
        found = any(
            item.get("local") == str(parsed.ip)
            and item.get("prefixlen") == parsed.network.prefixlen
            for device in devices
            for item in device.get("addr_info", [])
        )
        if found != remove:
            return {
                "interface": interface,
                "address": str(parsed),
                "removed": remove,
                "persistent": False,
            }
        time.sleep(0.2)
    raise XCoreError(
        "NetworkManager accepted the change but address verification timed out"
    )
