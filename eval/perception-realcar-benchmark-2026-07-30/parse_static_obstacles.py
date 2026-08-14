#!/usr/bin/env python3
"""Parse a `ros2 topic echo /f1/perception/static_obstacles > file.txt` capture
(F1stateArr, YAML stream separated by '---') into per-message landmark lists,
and summarize: distinct ids, first/last message index, position drift.
Fast line-scan parser (yaml.safe_load_all is too slow on multi-100k-line dumps).

    python3 parse_static_obstacles.py <file.txt>
"""
import sys


def parse(path):
    """Returns list of [(id,x,y,v), ...] per message, in order."""
    msgs = []
    cur = []  # (id,x,y,v) accumulator for the in-progress list item
    id_ = x = y = v = None
    in_arr = False
    with open(path) as f:
        for line in f:
            s = line.rstrip("\n")
            if s == "---":
                if in_arr and id_ is not None:
                    cur.append((id_, x, y, v))
                msgs.append(cur)
                cur = []
                id_ = x = y = v = None
                in_arr = False
                continue
            if s.startswith("f1_state_arr:"):
                in_arr = True
                if s.strip() == "f1_state_arr: []":
                    in_arr = False
                continue
            if not in_arr:
                continue
            t = s.strip()
            if t.startswith("- header:"):
                if id_ is not None:
                    cur.append((id_, x, y, v))
                id_ = x = y = v = None
            elif t.startswith("x:"):
                x = float(t.split(":", 1)[1])
            elif t.startswith("y:"):
                y = float(t.split(":", 1)[1])
            elif t.startswith("id:"):
                id_ = float(t.split(":", 1)[1])
            elif t.startswith("v:"):
                v = float(t.split(":", 1)[1])
    if cur or id_ is not None:
        if in_arr and id_ is not None:
            cur.append((id_, x, y, v))
        msgs.append(cur)
    return msgs


def parse_stamps(path):
    """Per-message outer header.stamp as float seconds, same order/length as parse().
    None entries where the capture predates the header-stamp fix (2026-08-05) --
    caller should fall back to the uniform-duration approximation in that case.
    """
    stamps = []
    sec = nsec = None
    seen_arr = False  # outer header always precedes 'f1_state_arr:' in the dump
    with open(path) as f:
        for line in f:
            s = line.rstrip("\n")
            if s == "---":
                stamps.append(sec + nsec * 1e-9 if (sec is not None and nsec is not None) else None)
                sec = nsec = None
                seen_arr = False
                continue
            if s.startswith("f1_state_arr:"):
                seen_arr = True
                continue
            if seen_arr:
                continue
            t = s.strip()
            if t.startswith("sec:"):
                sec = int(t.split(":", 1)[1])
            elif t.startswith("nanosec:"):
                nsec = int(t.split(":", 1)[1])
    if sec is not None or nsec is not None:
        stamps.append(sec + nsec * 1e-9 if (sec is not None and nsec is not None) else None)
    return stamps


def summarize(msgs, label=""):
    n = len(msgs)
    nonempty = sum(1 for m in msgs if m)
    by_id = {}
    for i, m in enumerate(msgs):
        for id_, x, y, v in m:
            by_id.setdefault(id_, []).append((i, x, y, v))
    print(f"[{label}] messages={n}  non-empty={nonempty} ({nonempty/max(n,1)*100:.0f}%)  distinct ids={len(by_id)}")
    for id_, pts in sorted(by_id.items()):
        idxs = [p[0] for p in pts]
        xs = [p[1] for p in pts]; ys = [p[2] for p in pts]
        print(f"  id={id_:.0f}  n={len(pts)}  msg[{idxs[0]}..{idxs[-1]}]  "
              f"x[{min(xs):.3f},{max(xs):.3f}]  y[{min(ys):.3f},{max(ys):.3f}]  "
              f"drift={((max(xs)-min(xs))**2+(max(ys)-min(ys))**2)**0.5:.3f}m")
    return by_id


if __name__ == "__main__":
    msgs = parse(sys.argv[1])
    summarize(msgs, label=sys.argv[1].split("/")[-1])
