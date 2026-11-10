"""Offline curriculum answer key. Synthetic checks, never operational approval."""
import ast
import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
for base in (ROOT, ROOT / "src", ROOT / "layer-b"):
    sys.path.insert(0, str(base))
CASES = []

def check(name, actual, expected):
    passed = actual == expected
    CASES.append({"case": name, "actual": actual, "expected": expected, "passed": passed})
    if not passed:
        raise AssertionError(f"{name}: expected {expected!r}, got {actual!r}")

def rejects(name, call, error=ValueError):
    try:
        call()
    except error:
        check(name, error.__name__, error.__name__)
    else:
        raise AssertionError(f"{name}: expected {error.__name__}")




def run():
    from topoplan.core import Grid
    r = Grid(["....."]*4).plan([0,0],[4,3])
    check("open-grid Manhattan optimum", r["routes"][0]["length"], 7)
    g = Grid([".......","...#...","...#...","...#...","......."],.2)
    q = g.plan([0,2],[6,2],2)
    check("two bounded signature alternatives", len(q["routes"]), 2)
    check("routes have different signatures", q["routes"][0]["signature"] != q["routes"][1]["signature"], True)
    check("both detours are length ten", [r["length"] for r in q["routes"]], [10,10])
    check("every returned segment clears inflated obstacles", all(g.clear_segment(a,b) for r in q["routes"] for a,b in zip(r["path"],r["path"][1:])), True)
    check("small search budget is not unreachability", Grid(["....."]*5).plan([0,0],[4,4],max_expansions=1)["status"], "expansion_limit")
    check("zero-length self route", Grid(["..."]).plan([1,0],[1,0])["routes"][0]["length"], 0)
    rejects("occupied start rejected", lambda: Grid([".#."]).plan([1,0],[2,0]))

if __name__ == "__main__":
    if sys.argv[1:] not in ([], ["--verify"]):
        raise SystemExit("Usage: python curriculum/lab.py [--verify]")
    run()
    report = {"schema_version": 1, "status": "PASS", "case_count": len(CASES),
              "cases": CASES, "scope": "Synthetic curriculum checks only"}
    if sys.argv[1:] == ["--verify"]:
        expected = json.loads((Path(__file__).parent / "expected.json").read_text())
        if report != expected:
            raise AssertionError("Observed answers differ from curriculum/expected.json")
    print(json.dumps(report, sort_keys=True, indent=2, allow_nan=False))
