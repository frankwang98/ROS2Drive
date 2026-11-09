#!/usr/bin/env python3
"""Exercise the actual C++ engine's interactive protocol without MQTT."""
import json
import subprocess
import sys

runner = sys.argv[1]
with subprocess.Popen(
    [runner, "--scenario", "port_transport", "--dt", "0.05"],
    stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
) as proc:
    def request(data):
        proc.stdin.write(data + "\n")
        proc.stdin.flush()
        return json.loads(proc.stdout.readline())

    first = request("STEP")
    assert first["kind"] == "frame"
    assert first["t"] > 0
    mission = request("MISSION smoke 1 1.0 0.5 30 1 2\n0 0 0\n2 0 0")
    assert mission["kind"] == "ack" and isinstance(mission["accepted"], bool)
    second = request("STEP")
    assert second["t"] > first["t"]
    assert isinstance(second["trajectory"], list)
    estop = request("ESTOP 1")
    assert estop == {"kind": "ack", "accepted": True}
    stopped = request("STEP")
    assert stopped["runtime_state"] == "ESTOP"
    proc.stdin.write("QUIT\n")
    proc.stdin.flush()
    assert proc.wait(timeout=5) == 0
