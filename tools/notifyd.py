#!/usr/bin/env python3
"""Gateway: watches the arm's event log and pushes the important ones to a phone.

WHY THIS IS NOT IN THE FIRMWARE

The arm writes events to a file and this reads them. That split is the whole
point, not a convenience. The firmware has to cross compile to a Cortex-M4
with no heap, no exceptions and no network stack, and bolting an HTTPS client
to it would end that. Real fleets are built the same way: the machine speaks
a simple local protocol and something else translates it to the outside world.

It also means the arm cannot be made to wait on a web request, and a phone
that is switched off cannot slow a control loop down.

WHAT IT SENDS, AND WHAT IT DOES NOT

Not everything. A notification that arrives for every ordinary event is one
the operator learns to ignore, which is worse than none. By default only
events that need a person are sent; --all overrides that for testing.

Rate limiting and de-duplication are not polish either. An intermittent
connector can trip a fault fifty times a minute, and fifty identical pushes
tell you nothing that one push and a count does not.

USAGE

  python3 tools/notifyd.py --dry-run             see what would be sent
  python3 tools/notifyd.py --topic YOUR-TOPIC    actually send

The topic is a shared secret: anyone who knows it can read your
notifications, and anyone can write to it. Choose something unguessable, do
not commit it, and do not put anything private in a message.
"""
import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_LOG = os.path.join(HERE, "data", "events.jsonl")

# What each event means to a person, and how loudly to say it.
#   level 3  something needs attention now
#   level 2  worth knowing, the arm handled it
#   level 1  routine, only sent with --all
RULES = {
    "safety SAFE STOP":        (3, "Arm stopped", "rotating_light"),
    "BOX DROPPED IN TRANSIT":  (3, "Box dropped", "package"),
    "aborted by safety":       (3, "Job aborted", "warning"),
    "pick failed, gripper empty": (2, "Pick failed", "warning"),
    "wrong item, mass mismatch":  (2, "Wrong item", "warning"),
    "release failed, box stuck":  (2, "Box stuck in gripper", "warning"),
    "timed out":               (2, "Step timed out", "hourglass"),
    "safety DEGRADED":         (2, "Running degraded", "warning"),
    "safety NORMAL":           (1, "Back to normal", "white_check_mark"),
    "placed":                  (1, "Box placed", "white_check_mark"),
    "grip confirmed":          (1, "Picked up", "mechanical_arm"),
    "contact found":           (1, "Touched down", "point_down"),
    "job started":             (1, "Job started", "arrow_forward"),
    "recovered":               (1, "Recovered", "white_check_mark"),
}
PRIORITY = {3: "urgent", 2: "high", 1: "default"}


class Limiter:
    """Stops a flapping fault from becoming fifty notifications.

    Timed on the MACHINE's clock, not the wall clock. The two are the same
    while watching a live run, and completely different while replaying a
    log, where a whole session arrives in a fraction of a second: wall clock
    timing collapsed events that were ten seconds apart on the arm into one.
    Machine time is also the honest unit for the question being asked, which
    is whether the arm did the same thing twice in quick succession.
    """

    def __init__(self, per_minute, repeat_window_s):
        self.per_minute = per_minute
        self.repeat_window = repeat_window_s
        self.sent = []          # timestamps of recent sends
        self.last_of = {}       # event name -> (when, how many suppressed)

    def check(self, name, now):
        """Returns (send, suppressed_count) for this event."""
        prev = self.last_of.get(name)
        if prev and now - prev[0] < self.repeat_window:
            self.last_of[name] = (prev[0], prev[1] + 1)
            return False, prev[1] + 1

        self.sent = [t for t in self.sent if now - t < 60.0]
        if len(self.sent) >= self.per_minute:
            return False, 0

        self.sent.append(now)
        self.last_of[name] = (now, 0)
        return True, 0


def classify(event):
    for key, rule in RULES.items():
        if key in event:
            return rule
    return (1, event, "robot")


def compose(rec, level, title, suppressed):
    bits = [f"t = {rec.get('t', 0):.1f} s"]
    if rec.get("reason") and rec["reason"] != "none":
        bits.append(rec["reason"])
    if rec.get("task"):
        bits.append(f"during {rec['task']}")
    if rec.get("payload") is not None and rec["payload"] > 0.01:
        bits.append(f"carrying {rec['payload']:.2f} kg")
    if suppressed:
        bits.append(f"({suppressed} more suppressed)")
    return title, "  ·  ".join(bits)


def send(topic, title, body, level, tag, timeout):
    req = urllib.request.Request(
        f"https://ntfy.sh/{topic}",
        data=body.encode("utf-8"),
        headers={
            "Title": title,
            "Priority": PRIORITY[level],
            "Tags": tag,
        },
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.status


def read_once(path):
    """Everything currently in the file, then stop."""
    if not os.path.exists(path):
        return
    with open(path) as f:
        for line in f:
            yield line


def follow(path, from_start):
    """Yields lines as they are appended, surviving the file not existing yet.

    Never returns. Callers that want to stop must use read_once instead: an
    earlier version tried to break out of this loop on an empty line, which
    cannot happen, so --once hung forever waiting for a line that was never
    coming.
    """
    f = None
    pos = 0
    while True:
        if f is None:
            if not os.path.exists(path):
                time.sleep(0.3)
                continue
            f = open(path)
            if not from_start:
                f.seek(0, os.SEEK_END)
            pos = f.tell()

        line = f.readline()
        if line:
            pos = f.tell()
            yield line
            continue

        # The console truncates this file when it restarts, so a file that
        # has got SHORTER is a new run and has to be reopened from the top.
        try:
            if os.path.getsize(path) < pos:
                f.close()
                f = None
                continue
        except OSError:
            f.close()
            f = None
            continue
        time.sleep(0.2)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", help="your ntfy.sh topic, a shared secret")
    ap.add_argument("--dry-run", action="store_true",
                    help="print instead of sending (the default with no topic)")
    ap.add_argument("--file", default=DEFAULT_LOG)
    ap.add_argument("--all", action="store_true",
                    help="send routine events too, for testing")
    ap.add_argument("--from-start", action="store_true",
                    help="replay the whole log rather than following new lines")
    ap.add_argument("--once", action="store_true",
                    help="process what is there and exit")
    ap.add_argument("--per-minute", type=int, default=10,
                    help="cap, counted over a minute of MACHINE time")
    ap.add_argument("--repeat-window", type=float, default=20.0)
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()

    dry = args.dry_run or not args.topic
    min_level = 1 if args.all else 2

    print(f"watching {args.file}")
    print(f"  sending level {min_level} and above"
          f"{' (everything)' if args.all else ' (the ones needing a person)'}")
    print(f"  {'DRY RUN, nothing leaves this machine' if dry else 'topic ' + args.topic}")
    print(f"  at most {args.per_minute} per minute, "
          f"repeats within {args.repeat_window:.0f} s collapsed\n")

    lim = Limiter(args.per_minute, args.repeat_window)
    seen = 0
    pushed = 0

    source = read_once(args.file) if args.once else follow(
        args.file, args.from_start)
    for line in source:
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            continue

        seen += 1
        name = rec.get("event", "")
        level, title, tag = classify(name)
        if level < min_level:
            continue

        ok, suppressed = lim.check(name, float(rec.get('t', 0.0)))
        if not ok:
            continue

        title, body = compose(rec, level, title, suppressed)
        marker = {3: "!!", 2: "!", 1: "·"}[level]
        print(f"  {marker} {title}: {body}")
        pushed += 1

        if not dry:
            try:
                send(args.topic, title, body, level, tag, args.timeout)
            except (urllib.error.URLError, OSError) as e:
                # A phone that cannot be reached must never stop the gateway.
                print(f"    could not send: {e}")


    print(f"\n{seen} events read, {pushed} pushed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
