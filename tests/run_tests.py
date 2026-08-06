"""Run the BILBO regression suite.

These tests exercise the real flight modules with only the hardware libraries
stubbed, and they exist to protect the Critical/High safety fixes made before
the first autonomous flight. Re-run them after any tuning change.

    python tests/run_tests.py

They do NOT replace the props-off bench procedure in tools/ -- in particular,
the yaw sign must still be confirmed against the physical airframe with
tools/verify_yaw_sign.py, because no test can know which way the camera is
bolted on.
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SUITES = ("audit_static.py", "test_invariants.py", "test_logic.py",
          "test_state.py", "test_runtime.py")


def main():
    failed = []
    for suite in SUITES:
        print("\n" + "=" * 70)
        print("RUNNING %s" % suite)
        print("=" * 70)
        result = subprocess.run([sys.executable, os.path.join(HERE, suite)])
        if result.returncode != 0:
            failed.append(suite)

    print("\n" + "=" * 70)
    if failed:
        print("SUITES FAILED: %s" % ", ".join(failed))
        return 1
    print("ALL SUITES PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
