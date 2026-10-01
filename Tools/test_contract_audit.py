"""Focused mutation checks for check_contract.py (stdlib only)."""
import importlib.util
from pathlib import Path
import sys


HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("check_contract", HERE / "check_contract.py")
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)


def expect_failure(label, rel, mutate):
    original = (gate.ROOT / rel).read_text(encoding="utf-8")
    mutated = mutate(original)
    if mutated == original:
        raise AssertionError("mutation did not change the source (stale marker): " + label)
    failures = gate.audit(overrides={rel: mutated})
    if not failures:
        raise AssertionError("mutation was not rejected: " + label)


def main():
    baseline = gate.audit()
    if baseline:
        raise AssertionError("baseline audit must be clean before mutating: " + "; ".join(baseline))
    expect_failure(
        "caller loop filter recovery assignment",
        "Source/MotionMatching/Runtime/MotionMatchingRuntimePolicy.cpp",
        lambda text: text.replace(
            "settings.LoopFilter = (reseeded || keepReseedArmed) ? MotionLoopFilter::LoopOnly :",
            "settings.LoopFilter = (reseeded || keepReseedArmed) ? MotionLoopFilter::Any :",
            1),
    )
    expect_failure(
        "end recovery loop-only retry",
        "Source/MotionMatching/Runtime/MotionMatchingRuntimePolicy.cpp",
        lambda text: text.replace(
            "retry.LoopFilter = MotionLoopFilter::LoopOnly;",
            "retry.LoopFilter = MotionLoopFilter::Any;",
            1),
    )
    expect_failure(
        "legacy asset TypeName",
        "Source/MotionMatching/Database/MotionMatchingDatabase.cpp",
        lambda text: text.replace(
            'MotionMatchingDatabase::TypeName = TEXT("Game.MotionMatchingDatabase")',
            'MotionMatchingDatabase::TypeName = TEXT("MotionMatching.MotionMatchingDatabase")',
            1),
    )
    expect_failure(
        "forbidden game coupling",
        "Source/MotionMatching/MotionMatchingController.cs",
        lambda text: text + "\nPlayerCoordinator hostLeak;\n",
    )
    expect_failure(
        "public documentation link",
        "README.md",
        lambda text: text.replace("docs/LIFECYCLE.md", "docs/REMOVED.md", 1),
    )
    print("contract audit mutation tests: PASS (5/5)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
