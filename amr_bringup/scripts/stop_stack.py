"""Stop this workspace's simulation - and nothing else."""

import os
import signal
import sys
import time

WORKSPACE = os.environ.get(
    "AMR_WORKSPACE",
    os.path.expanduser("~/delhivery_assgn/ros2_ws"))
INSTALL = os.path.join(os.path.realpath(WORKSPACE), "install")
PACKAGES = ("amr_gazebo", "amr_mapping", "amr_navigation", "amr_safety",
            "amr_bringup", "amr_core", "amr_description")


def argv_of(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as handle:
            return [a.decode(errors="replace")
                    for a in handle.read().split(b"\x00") if a]
    except OSError:
        return []


def ppid_of(pid):
    try:
        with open(f"/proc/{pid}/stat", "rb") as handle:
            data = handle.read().decode(errors="replace")
        return int(data[data.rindex(")") + 1:].split()[1])
    except (OSError, ValueError, IndexError):
        return 0


def ancestors_of(pid):
    """Every process between us and init. None of these may ever be killed."""
    chain, seen = set(), 0
    while pid > 1 and seen < 64:
        pid = ppid_of(pid)
        if pid <= 1 or pid in chain:
            break
        chain.add(pid)
        seen += 1
    return chain


def env_of(pid):
    try:
        with open(f"/proc/{pid}/environ", "rb") as handle:
            raw = handle.read()
    except OSError:
        return {}
    env = {}
    for item in raw.split(b"\x00"):
        if b"=" in item:
            key, _, value = item.decode(errors="replace").partition("=")
            env[key] = value
    return env


def from_this_workspace(pid):
    """True if the process was launched with this workspace overlaid."""
    env = env_of(pid)
    return any(INSTALL in env.get(key, "")
               for key in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH",
                           "LD_LIBRARY_PATH", "PYTHONPATH"))


def owned_by_us(pid):
    try:
        return os.stat(f"/proc/{pid}").st_uid == os.getuid()
    except OSError:
        return False


def classify(pid):
    """Return a reason string if this PID belongs to our simulation."""
    argv = argv_of(pid)
    if not argv:
        return None

    tokens = " ".join(argv).split()
    if not tokens:
        return None
    if os.path.basename(tokens[0]) == "ign":
        return "ignition"

    for arg in tokens[1:]:
        if arg.startswith(INSTALL + os.sep):
            for package in PACKAGES:
                if f"{os.sep}{package}{os.sep}" in arg:
                    return f"workspace node ({package})"

    if os.path.basename(tokens[0]) == "ros2" and "launch" in tokens:
        if any("amr_" in t for t in tokens):
            return "ros2 launch"

    if "--ros-args" in tokens and from_this_workspace(pid):
        return f"ros node ({os.path.basename(tokens[0])})"
    return None


def main():
    dry_run = "--dry-run" in sys.argv
    me = os.getpid()
    protected = ancestors_of(me) | {me}

    targets = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in protected or not owned_by_us(pid):
            continue
        reason = classify(pid)
        if reason:
            targets.append((pid, reason, " ".join(argv_of(pid))[:90]))

    if not targets:
        print("Nothing from this workspace is running.")
        return 0

    verb = "Would stop" if dry_run else "Stopping"
    print(f"{verb} {len(targets)} process(es) for {WORKSPACE}:")
    for pid, reason, cmd in sorted(targets):
        print(f"  {pid:>7}  {reason:<24} {cmd}")
    if dry_run:
        return 0

    pids = [t[0] for t in targets]
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        alive = []
        for pid in pids:
            try:
                os.kill(pid, 0)
                alive.append(pid)
            except OSError:
                pass
        if not alive:
            break
        for pid in alive:
            try:
                os.kill(pid, sig)
            except OSError:
                pass
        time.sleep(3 if sig is not signal.SIGKILL else 1)

    leftover = [p for p in (int(e) for e in os.listdir("/proc") if e.isdigit())
                if os.path.exists(f"/proc/{p}") and owned_by_us(p)
                and classify(p) == "ignition"]
    if leftover:
        print(f"WARNING: Ignition still running: {leftover}")
        print("Kill by PID before relaunching, or objects will flicker.")
        return 1

    print("Stopped. No Ignition process remains - safe to launch again.")
    if any(os.path.basename(argv_of(p)[0] if argv_of(p) else "")
           in ("gzserver", "gzclient")
           for p in (int(e) for e in os.listdir("/proc") if e.isdigit())):
        print("(Left alone: a classic Gazebo session from another project.)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
