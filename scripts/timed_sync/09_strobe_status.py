#!/usr/bin/env python3
"""Report TSU strobe status: systemd service state + the daemon's status file.

The strobe daemon (tsu_strobe_service.py, run by tsu-strobe.service) writes JSON
to /run/tsu-strobe.status every second: running, pid, rearms, tsu_sec,
updated_unix_s. This reads that file (and `systemctl is-active tsu-strobe`) so
readout-tools / a person can confirm the strobe is alive and re-arming.

A healthy strobe: service active, status file fresh (updated < ~2 s ago),
running=True, and rearms increasing (~1/s) on repeat reads.
"""
import json, subprocess, sys, time

STATUS_PATH = "/run/tsu-strobe.status"
SERVICE = "tsu-strobe.service"

# systemd service state (best-effort; works whether or not the service is used)
try:
    active = subprocess.run(["systemctl", "is-active", SERVICE],
                            capture_output=True, text=True).stdout.strip()
except Exception as exc:
    active = f"<systemctl unavailable: {exc}>"
print(f"service {SERVICE}: {active}")

# status file
try:
    with open(STATUS_PATH) as f:
        st = json.load(f)
except FileNotFoundError:
    print(f"status file {STATUS_PATH}: MISSING -- daemon has not run "
          f"(or wrote nowhere). Strobe likely not started.")
    sys.exit(1)
except (OSError, json.JSONDecodeError) as exc:
    print(f"status file {STATUS_PATH}: unreadable ({exc})")
    sys.exit(1)

age = time.time() - st.get("updated_unix_s", 0)
print(f"status file {STATUS_PATH}:")
print(f"  running        {st.get('running')}")
print(f"  pid            {st.get('pid')}")
print(f"  rearms         {st.get('rearms')}")
print(f"  tsu_sec        {st.get('tsu_sec')}")
print(f"  updated        {age:.1f} s ago")

healthy = (active == "active" and st.get("running") and age < 3.0)
print("\nHEALTHY: strobe active and re-arming" if healthy else
      "\nNOT HEALTHY: check the service (journalctl -u tsu-strobe) -- "
      "stale status, not running, or service inactive.")
sys.exit(0 if healthy else 1)
