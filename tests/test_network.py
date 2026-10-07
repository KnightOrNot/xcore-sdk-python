import json

from xcore_sdk_python import network


def test_proxy_tcp_acceptance_does_not_imply_direct_controller_connection(monkeypatch):
    def command(*args):
        if "addr" in args:
            return json.dumps(
                [
                    {
                        "ifname": "Meta",
                        "operstate": "UP",
                        "addr_info": [
                            {"family": "inet", "local": "198.18.0.1", "prefixlen": 30}
                        ],
                    }
                ]
            )
        if "route" in args:
            return json.dumps([{"dev": "Meta", "gateway": "198.18.0.2"}])
        return "[]"

    class Socket:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

    monkeypatch.setattr(network, "system_command", command)
    monkeypatch.setattr(network.socket, "create_connection", lambda *a, **kw: Socket())
    report = network.diagnose("192.168.0.160")
    assert report["sdk_port_reachable"]
    assert report["same_subnet_on_route_interface"] is False
    assert report["route_interface"] == "Meta"
