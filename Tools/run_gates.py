#!/usr/bin/env python3
"""Run every engine-free static gate and print one PASS/FAIL line per gate.

Plugin-only gates (always run):
  - Tools/check_contract.py        static contract audit of the plugin sources
  - Tools/test_contract_audit.py   mutation tests proving the audit can fail

Repository gates (run only when the plugin sits inside the flax-test repo,
detected by <repo>/flaxtest.flaxproj next to <repo>/Plugins/MotionMatching):
  - Tools/mm_module_check.py             game <-> plugin boundary
  - Tools/mm_editor_decoupling_check.py  plugin editor module has no host conventions
  - Tools/mm_tests.py                    offline contracts / determinism

Not run here, because they need the Flax engine or a running editor: the native
build, the self-tests, Tools/mm_contract_test.py, Tools/mm_test_25.py and
Tools/mm_parity_check.py. Standalone-host compile checks live in
Tools/StandaloneMotionMatchingHost/verify_standalone_host.py.

Only the Python standard library is needed. Exit code is 0 when all gates pass.
Usage: python run_gates.py [-v]   (-v prints the output of failing gates)
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

PLUGIN_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PLUGIN_ROOT.parents[1]


def gate_list():
    gates = [
        ("plugin check_contract", PLUGIN_ROOT / "Tools" / "check_contract.py", PLUGIN_ROOT),
        ("plugin test_contract_audit", PLUGIN_ROOT / "Tools" / "test_contract_audit.py", PLUGIN_ROOT),
    ]
    in_repo = (REPO_ROOT / "flaxtest.flaxproj").is_file() and (REPO_ROOT / "Tools" / "mm_tests.py").is_file()
    if in_repo:
        for name in ("mm_module_check", "mm_editor_decoupling_check", "mm_tests"):
            gates.append(("repo " + name, REPO_ROOT / "Tools" / (name + ".py"), REPO_ROOT))
    return gates, in_repo


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("-v", "--verbose", action="store_true", help="print output of failing gates")
    args = parser.parse_args()

    gates, in_repo = gate_list()
    if not in_repo:
        print("note: not inside the flax-test repo, running plugin-only gates")
    failed = 0
    for name, script, cwd in gates:
        if not script.is_file():
            print("FAIL  %-34s missing script %s" % (name, script))
            failed += 1
            continue
        start = time.time()
        proc = subprocess.run([sys.executable, str(script)], cwd=str(cwd), stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, universal_newlines=True, errors="replace")
        seconds = time.time() - start
        ok = proc.returncode == 0
        print("%s  %-34s (%.1fs)" % ("PASS" if ok else "FAIL", name, seconds))
        if not ok:
            failed += 1
            if args.verbose:
                print(proc.stdout)
    print("%d/%d gates passed" % (len(gates) - failed, len(gates)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
