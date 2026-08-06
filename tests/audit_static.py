"""Static audit of the BILBO codebase.

Catches the classes of bug that pyflakes cannot see:

  1. `cfg.SOMETHING` / `tracker.something` where the attribute does not exist.
     This is what a deleted or renamed constant leaves behind, and it only
     surfaces at runtime -- often deep inside a branch that a bench test does
     not reach.
  2. Cross-module calls with keyword arguments the callee does not accept, or
     missing required arguments. This is the exact shape of the two LED crashes
     (`effect=` vs `state=`, and pi5neo's `brightness=`).
"""

import ast
import inspect
import io
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "tests"))
import harness  # noqa: F401,E402  installs hardware stubs
sys.path.insert(0, os.path.join(REPO, "detectors"))

import camera      # noqa: E402
import config      # noqa: E402
import controller  # noqa: E402
import drone       # noqa: E402
import led         # noqa: E402
import lidar       # noqa: E402
import tracker     # noqa: E402

# alias used in source -> the real module object
MODULES = {
    "cfg": config,
    "config": config,
    "camera": camera,
    "controller": controller,
    "drone": drone,
    "led": led,
    "lidar": lidar,
    "tracker": tracker,
}

# Stubbed in this environment, so attribute checks would give false positives.
SKIP_ALIASES = {"mavutil", "cv2", "picam2", "imx500", "intrinsics", "np",
                "serial", "neo", "request", "m", "logging", "os", "sys",
                "time", "math", "csv", "signal", "argparse", "inspect",
                "threading", "glob", "types", "importlib", "harness"}

TARGETS = []
for folder in ("detectors", "tools"):
    d = os.path.join(REPO, folder)
    for name in sorted(os.listdir(d)):
        if name.endswith(".py"):
            TARGETS.append(os.path.join(d, name))

problems = []
attr_checked = 0
call_checked = 0
unverifiable = []


def rel(path):
    return os.path.relpath(path, REPO).replace(os.sep, "/")


for path in TARGETS:
    src = io.open(path, encoding="utf-8").read()
    tree = ast.parse(src, filename=path)

    for node in ast.walk(tree):
        # ---- 1. module attribute existence --------------------------------
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name):
            alias = node.value.id
            if alias in SKIP_ALIASES:
                continue
            mod = MODULES.get(alias)
            if mod is None:
                continue
            attr_checked += 1
            if not hasattr(mod, node.attr):
                problems.append(
                    "%s:%d  %s.%s does not exist on module %s"
                    % (rel(path), node.lineno, alias, node.attr,
                       mod.__name__)
                )

        # ---- 2. cross-module call signatures ------------------------------
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                and isinstance(node.func.value, ast.Name):
            alias = node.func.value.id
            mod = MODULES.get(alias)
            if mod is None:
                continue
            func = getattr(mod, node.func.attr, None)
            if func is None or not callable(func):
                continue
            try:
                sig = inspect.signature(func)
            except (TypeError, ValueError):
                unverifiable.append("%s:%d %s.%s"
                                    % (rel(path), node.lineno, alias,
                                       node.func.attr))
                continue

            call_checked += 1
            params = sig.parameters
            has_var_kw = any(p.kind == p.VAR_KEYWORD for p in params.values())
            has_var_pos = any(p.kind == p.VAR_POSITIONAL
                              for p in params.values())

            # keywords the callee cannot accept
            for kw in node.keywords:
                if kw.arg is None:      # **kwargs at the call site
                    continue
                if kw.arg not in params and not has_var_kw:
                    problems.append(
                        "%s:%d  %s.%s() does not accept keyword %r  (accepts: %s)"
                        % (rel(path), node.lineno, alias, node.func.attr,
                           kw.arg, ", ".join(params) or "nothing")
                    )

            # too many positionals
            positional = [p for p in params.values()
                          if p.kind in (p.POSITIONAL_ONLY,
                                        p.POSITIONAL_OR_KEYWORD)]
            n_given = len([a for a in node.args
                           if not isinstance(a, ast.Starred)])
            if not has_var_pos and n_given > len(positional):
                problems.append(
                    "%s:%d  %s.%s() takes %d positional arg(s), %d given"
                    % (rel(path), node.lineno, alias, node.func.attr,
                       len(positional), n_given)
                )

            # required parameters left unsupplied
            supplied = set(p.name for p in positional[:n_given])
            supplied |= set(kw.arg for kw in node.keywords if kw.arg)
            starred = any(isinstance(a, ast.Starred) for a in node.args)
            double_starred = any(kw.arg is None for kw in node.keywords)
            if not starred and not double_starred:
                for p in params.values():
                    if p.kind in (p.VAR_POSITIONAL, p.VAR_KEYWORD):
                        continue
                    if p.default is not inspect.Parameter.empty:
                        continue
                    if p.name == "self":
                        continue
                    if p.name not in supplied:
                        problems.append(
                            "%s:%d  %s.%s() missing required argument %r"
                            % (rel(path), node.lineno, alias,
                               node.func.attr, p.name)
                        )

print("audited %d files: %d module attributes, %d cross-module calls"
      % (len(TARGETS), attr_checked, call_checked))
if unverifiable:
    print("\nsignature not introspectable (%d):" % len(unverifiable))
    for u in unverifiable:
        print("  " + u)

if problems:
    print("\n%d PROBLEM(S):" % len(problems))
    for p in problems:
        print("  " + p)
    raise SystemExit(1)

print("\nno problems found")
