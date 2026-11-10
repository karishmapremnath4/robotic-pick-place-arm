#!/usr/bin/env python3
"""Builds a self-contained viewer page from a recorded run.

Everything is inlined, so the page opens from the filesystem with no server
and no network except the one CDN script tag. That matters because the
alternative, fetching the CSV with XHR, is blocked for file:// pages by every
browser's same-origin policy, and telling someone to start a web server
before they can look at a picture is a poor trade.

Reads   data/replay.csv, data/replay_events.tsv, data/scene.tsv
Writes  data/viewer.html
"""
import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(HERE, "data")


def read_scene(path):
    scene = {"objects": [], "cspace": [], "path": []}
    with open(path) as f:
        for line in f:
            parts = line.rstrip("\n").split("\t")
            key = parts[0]
            if key in ("table_y", "link1", "link2"):
                scene[key] = float(parts[1])
            elif key == "grid":
                scene["grid"] = int(parts[1])
            elif key == "object":
                scene["objects"].append(
                    {"x": float(parts[1]), "top": float(parts[2]),
                     "halfw": float(parts[3])})
            elif key == "cspace":
                scene["cspace"].append(parts[1])
            elif key == "path":
                scene["path"].append([float(parts[1]), float(parts[2])])
    return scene


def read_frames(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    head, body = rows[0], rows[1:]
    idx = {name: i for i, name in enumerate(head)}

    def g(r, name):
        return float(r[idx[name]])

    frames = []
    for r in body:
        if len(r) < len(head):
            continue
        boxes = []
        for b in range(3):
            kx, ky, kh = f"b{b}x", f"b{b}y", f"b{b}held"
            if kx in idx:
                boxes.append([g(r, kx), g(r, ky), int(r[idx[kh]])])
        health = [int(r[idx[f"h{i}"]]) for i in range(5) if f"h{i}" in idx]
        frames.append({
            "t": g(r, "t"),
            "q": [g(r, "tq1"), g(r, "tq2")],
            "e": [g(r, "eq1"), g(r, "eq2")],
            "pay": [g(r, "tpay"), g(r, "epay"), g(r, "psig")],
            "f": [g(r, "tforce"), g(r, "eforce")],
            "laser": g(r, "laser"),
            "task": int(r[idx["task"]]),
            "safety": int(r[idx["safety"]]),
            "survey": int(r[idx["survey"]]),
            "boxes": boxes,
            "health": health,
        })
    return frames


def read_events(path):
    out = []
    if not os.path.exists(path):
        return out
    with open(path) as f:
        for line in f:
            p = line.rstrip("\n").split("\t")
            if len(p) >= 2:
                out.append({"t": float(p[0]), "name": p[1]})
    return out


TASK_NAMES = ["idle", "move above pick", "descend to pick", "close gripper",
              "verify grip", "lift", "move above place", "descend to place",
              "release", "verify release", "retract", "recover", "done",
              "FAILED"]
SAFETY_NAMES = ["NORMAL", "DEGRADED", "SAFE STOP"]
SENSOR_NAMES = ["spi encoders", "drive position", "drive velocity", "tool imu",
                "laser ranger"]
HEALTH_NAMES = ["ok", "suspect", "FAULTED", "probation"]


def main():
    scene = read_scene(os.path.join(DATA, "scene.tsv"))
    frames = read_frames(os.path.join(DATA, "replay.csv"))
    events = read_events(os.path.join(DATA, "replay_events.tsv"))
    if not frames:
        print("no frames in data/replay.csv, run build/record first")
        return 1

    payload = {
        "scene": scene,
        "frames": frames,
        "events": events,
        "taskNames": TASK_NAMES,
        "safetyNames": SAFETY_NAMES,
        "sensorNames": SENSOR_NAMES,
        "healthNames": HEALTH_NAMES,
    }

    tpl = open(os.path.join(HERE, "tools", "viewer_template.html")).read()
    html = tpl.replace("__DATA__", json.dumps(payload, separators=(",", ":")))
    out = os.path.join(DATA, "viewer.html")
    with open(out, "w") as f:
        f.write(html)
    print(f"wrote {out}")
    print(f"  {len(frames)} frames, {frames[-1]['t']:.1f} s")
    print(f"  {len(events)} events, {len(scene['objects'])} objects")
    print(f"  {os.path.getsize(out) / 1024:.0f} KB, self contained")
    return 0


if __name__ == "__main__":
    sys.exit(main())
