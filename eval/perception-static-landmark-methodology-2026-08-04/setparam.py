#!/usr/bin/env python3
"""Set a nested yaml param by <section>.<key> path, preserving comments/formatting.
Line-based (not a yaml round-trip) because keys like min_range_cluster appear in
multiple sections and we must not reformat the human-maintained config.

    python3 setparam.py <yaml> <section.key> <value>
    python3 setparam.py <yaml> --get <section.key>
"""
import sys, re

def indent_of(line):
    return len(line) - len(line.lstrip())

def find(lines, section, key):
    """Return index of the line defining section->key, or None."""
    sec_indent = None
    in_sec = False
    for i, ln in enumerate(lines):
        if not ln.strip() or ln.lstrip().startswith("#"):
            continue
        ind = indent_of(ln)
        name = ln.strip().split(":")[0].strip()
        if in_sec and ind <= sec_indent:
            in_sec = False           # left the section
        if name == section and ln.rstrip().endswith(":"):
            in_sec, sec_indent = True, ind
            continue
        if in_sec and name == key and ind > sec_indent:
            return i
    return None

def main():
    path = sys.argv[1]
    if sys.argv[2] == "--get":
        section, key = sys.argv[3].split(".")
        lines = open(path).read().split("\n")
        i = find(lines, section, key)
        print(lines[i].split(":", 1)[1].split("#")[0].strip() if i is not None else "NOTFOUND")
        return
    section, key = sys.argv[2].split(".")
    value = sys.argv[3]
    lines = open(path).read().split("\n")
    i = find(lines, section, key)
    if i is None:
        print(f"ERROR: {section}.{key} not found in {path}", file=sys.stderr)
        sys.exit(1)
    ln = lines[i]
    ind = " " * indent_of(ln)
    comment = ""
    if "#" in ln:
        comment = "  " + ln[ln.index("#"):]
    lines[i] = f"{ind}{key}: {value}{comment}"
    open(path, "w").write("\n".join(lines))
    print(f"set {section}.{key} = {value}")

main()
