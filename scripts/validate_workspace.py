#!/usr/bin/env python3
"""
AIRHOUND Workspace Validator

Pre-flight check for the ROS2 workspace. Catches the stuff that silently
breaks at runtime: topic mismatches, msg type mismatches, broken entry points,
missing package deps, import errors.

Run from repo root:
    python3 scripts/validate_workspace.py

Or with verbose output:
    python3 scripts/validate_workspace.py -v

Exit code 0 = all checks pass, 1 = failures found.

Maintainer: Rylan (Perception Lead)
"""

import ast
import os
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# ── Config ──────────────────────────────────────────────────────────────────

WORKSPACE = Path(__file__).resolve().parent.parent / "ws_ros2" / "src"
VERBOSE = "-v" in sys.argv or "--verbose" in sys.argv


# ── Data structures ─────────────────────────────────────────────────────────


@dataclass
class TopicEndpoint:
    """A single publisher or subscriber."""

    node_file: str  # relative path to source file
    topic: str  # topic name
    msg_type: str  # e.g. "Float64", "Detection2DArray"
    direction: str  # "pub" or "sub"
    line: int  # line number in source


@dataclass
class EntryPoint:
    """A console_scripts entry point from setup.py."""

    executable: str  # e.g. "tracking_node"
    module: str  # e.g. "tracking_geometry.tracking_geometry"
    function: str  # e.g. "main"


@dataclass
class PackageInfo:
    """Parsed ROS2 package metadata."""

    name: str
    path: Path
    build_type: str  # "ament_python" or "ament_cmake"
    exec_depends: list = field(default_factory=list)
    entry_points: list = field(default_factory=list)
    python_files: list = field(default_factory=list)


@dataclass
class CheckResult:
    """Result of a single validation check."""

    name: str
    passed: bool
    message: str
    severity: str = "error"  # "error" or "warn"


# ── Parsing ─────────────────────────────────────────────────────────────────


def find_packages() -> list[PackageInfo]:
    """Find all ROS2 packages in the workspace."""
    packages = []
    for pkg_xml in WORKSPACE.rglob("package.xml"):
        pkg_dir = pkg_xml.parent
        # Skip nested packages (e.g. px4_msgs submodule internals)
        rel = pkg_dir.relative_to(WORKSPACE)
        if len(rel.parts) > 1 and rel.parts[0] == "px4_msgs":
            continue

        tree = ET.parse(pkg_xml)
        root = tree.getroot()

        name = root.findtext("name", "")
        build_type = ""
        export = root.find("export")
        if export is not None:
            bt = export.findtext("build_type", "")
            build_type = bt

        exec_deps = [dep.text for dep in root.findall("exec_depend") if dep.text]

        # Parse entry points from setup.py if ament_python
        entry_points = []
        setup_py = pkg_dir / "setup.py"
        if setup_py.exists():
            entry_points = parse_entry_points(setup_py)

        # Find all python source files
        py_files = []
        for pattern in ["**/*.py"]:
            for f in pkg_dir.rglob(pattern):
                if "__pycache__" not in str(f) and "test" not in f.parts:
                    py_files.append(f)

        packages.append(
            PackageInfo(
                name=name,
                path=pkg_dir,
                build_type=build_type,
                exec_depends=exec_deps,
                entry_points=entry_points,
                python_files=py_files,
            )
        )
    return packages


def parse_entry_points(setup_py: Path) -> list[EntryPoint]:
    """Extract console_scripts entry points from setup.py."""
    content = setup_py.read_text()
    entries = []

    # Match patterns like: "name = module.path:function"
    pattern = r'"(\w+)\s*=\s*([\w.]+):(\w+)"'
    for match in re.finditer(pattern, content):
        entries.append(
            EntryPoint(
                executable=match.group(1),
                module=match.group(2),
                function=match.group(3),
            )
        )
    return entries


def extract_topics(py_file: Path) -> list[TopicEndpoint]:
    """Extract publisher/subscriber topic info from a Python ROS2 node."""
    try:
        content = py_file.read_text(encoding="utf-8")
    except (UnicodeDecodeError, ValueError):
        return []  # Skip binary / non-UTF-8 files
    endpoints = []

    # Match create_publisher(MsgType, 'topic', ...)
    pub_pattern = r"create_publisher\(\s*(\w+)\s*,\s*['\"]([^'\"]+)['\"]"
    for match in re.finditer(pub_pattern, content):
        line = content[: match.start()].count("\n") + 1
        endpoints.append(
            TopicEndpoint(
                node_file=str(py_file.relative_to(WORKSPACE)),
                topic=match.group(2),
                msg_type=match.group(1),
                direction="pub",
                line=line,
            )
        )

    # Match create_subscription(MsgType, 'topic', ...)
    sub_pattern = r"create_subscription\(\s*(\w+)\s*,\s*['\"]([^'\"]+)['\"]"
    for match in re.finditer(sub_pattern, content):
        line = content[: match.start()].count("\n") + 1
        endpoints.append(
            TopicEndpoint(
                node_file=str(py_file.relative_to(WORKSPACE)),
                topic=match.group(2),
                msg_type=match.group(1),
                direction="sub",
                line=line,
            )
        )

    return endpoints


def check_python_imports(py_file: Path, pkg_path: Path) -> list[str]:
    """Check that intra-package imports resolve (AST parse only, no execution)."""
    errors = []
    try:
        tree = ast.parse(py_file.read_text(encoding="utf-8"))
    except (SyntaxError, UnicodeDecodeError) as e:
        return [f"Parse error: {e}"] if isinstance(e, SyntaxError) else []

    pkg_name = pkg_path.name
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            # Only check intra-package imports (not rclpy, numpy, etc.)
            if node.module.startswith(pkg_name + "."):
                # e.g. "from tracking_geometry.kalman_filter import X"
                # → check that tracking_geometry/kalman_filter.py exists
                parts = node.module.split(".")
                rel_path = Path(*parts[1:]).with_suffix(".py")
                target = pkg_path / pkg_name / rel_path
                if not target.exists():
                    errors.append(
                        f"import '{node.module}' → {target.relative_to(WORKSPACE)} not found"
                    )
    return errors


# ── Checks ──────────────────────────────────────────────────────────────────


def check_topic_wiring(all_endpoints: list[TopicEndpoint]) -> list[CheckResult]:
    """Verify publisher/subscriber pairs match on topic name AND msg type."""
    results = []

    # Group by topic
    by_topic: dict[str, list[TopicEndpoint]] = {}
    for ep in all_endpoints:
        by_topic.setdefault(ep.topic, []).append(ep)

    for topic, eps in sorted(by_topic.items()):
        pubs = [e for e in eps if e.direction == "pub"]
        subs = [e for e in eps if e.direction == "sub"]

        # Skip PX4 internal topics and diagnostic topics
        if topic.startswith("/fmu/") or topic.startswith("/perception/"):
            continue

        # Check: topic has both pub and sub
        if pubs and not subs:
            results.append(
                CheckResult(
                    name=f"topic:{topic}",
                    passed=True,  # warn, not fail
                    message=f"published by {pubs[0].node_file}:{pubs[0].line} but no subscriber found (may be external)",
                    severity="warn",
                )
            )
            continue

        if subs and not pubs:
            results.append(
                CheckResult(
                    name=f"topic:{topic}",
                    passed=True,
                    message=f"subscribed by {subs[0].node_file}:{subs[0].line} but no publisher found (may be external)",
                    severity="warn",
                )
            )
            continue

        # Check: msg types match across all pubs and subs
        pub_types = set(e.msg_type for e in pubs)
        sub_types = set(e.msg_type for e in subs)

        if pub_types != sub_types:
            pub_detail = ", ".join(
                f"{e.msg_type} ({e.node_file}:{e.line})" for e in pubs
            )
            sub_detail = ", ".join(
                f"{e.msg_type} ({e.node_file}:{e.line})" for e in subs
            )
            results.append(
                CheckResult(
                    name=f"topic:{topic}",
                    passed=False,
                    message=f"MSG TYPE MISMATCH — publishers: [{pub_detail}], subscribers: [{sub_detail}]",
                )
            )
        else:
            results.append(
                CheckResult(
                    name=f"topic:{topic}",
                    passed=True,
                    message=f"OK — {pub_types.pop()} ({len(pubs)} pub, {len(subs)} sub)",
                )
            )

    return results


def check_entry_points(packages: list[PackageInfo]) -> list[CheckResult]:
    """Verify setup.py entry points resolve to real modules with main()."""
    results = []

    for pkg in packages:
        for ep in pkg.entry_points:
            # Check module file exists
            parts = ep.module.split(".")
            mod_path = pkg.path / Path(*parts).with_suffix(".py")

            if not mod_path.exists():
                # Try as package (directory with __init__.py)
                alt_path = pkg.path / Path(*parts) / "__init__.py"
                if not alt_path.exists():
                    results.append(
                        CheckResult(
                            name=f"entry:{pkg.name}/{ep.executable}",
                            passed=False,
                            message=f"'{ep.module}' → {mod_path.relative_to(WORKSPACE)} not found",
                        )
                    )
                    continue

            # Check main() function exists
            try:
                tree = ast.parse(mod_path.read_text())
                func_names = [
                    n.name
                    for n in ast.walk(tree)
                    if isinstance(n, ast.FunctionDef)
                ]
                if ep.function not in func_names:
                    results.append(
                        CheckResult(
                            name=f"entry:{pkg.name}/{ep.executable}",
                            passed=False,
                            message=f"'{ep.function}()' not found in {mod_path.relative_to(WORKSPACE)}",
                        )
                    )
                else:
                    results.append(
                        CheckResult(
                            name=f"entry:{pkg.name}/{ep.executable}",
                            passed=True,
                            message=f"OK — {ep.module}:{ep.function}",
                        )
                    )
            except SyntaxError as e:
                results.append(
                    CheckResult(
                        name=f"entry:{pkg.name}/{ep.executable}",
                        passed=False,
                        message=f"SyntaxError in {mod_path.relative_to(WORKSPACE)}: {e}",
                    )
                )

    return results


def check_imports(packages: list[PackageInfo]) -> list[CheckResult]:
    """Verify intra-package Python imports resolve."""
    results = []

    for pkg in packages:
        for py_file in pkg.python_files:
            if py_file.name.startswith("_"):
                continue
            errors = check_python_imports(py_file, pkg.path)
            if errors:
                for err in errors:
                    results.append(
                        CheckResult(
                            name=f"import:{py_file.relative_to(WORKSPACE)}",
                            passed=False,
                            message=err,
                        )
                    )
            elif VERBOSE:
                results.append(
                    CheckResult(
                        name=f"import:{py_file.relative_to(WORKSPACE)}",
                        passed=True,
                        message="OK",
                    )
                )

    return results


def check_package_deps(packages: list[PackageInfo]) -> list[CheckResult]:
    """Check that msg types used in code have corresponding package.xml deps."""
    results = []

    # Map msg types to required packages
    msg_to_pkg = {
        "Float32": "std_msgs",
        "Float64": "std_msgs",
        "String": "std_msgs",
        "Int32": "std_msgs",
        "Bool": "std_msgs",
        "Image": "sensor_msgs",
        "CompressedImage": "sensor_msgs",
        "CameraInfo": "sensor_msgs",
        "PointStamped": "geometry_msgs",
        "Detection2DArray": "vision_msgs",
        "Detection2D": "vision_msgs",
    }

    for pkg in packages:
        used_msg_pkgs = set()
        for py_file in pkg.python_files:
            try:
                content = py_file.read_text(encoding="utf-8")
            except (UnicodeDecodeError, ValueError):
                continue
            for msg_type, msg_pkg in msg_to_pkg.items():
                if msg_type in content:
                    used_msg_pkgs.add(msg_pkg)

        for msg_pkg in used_msg_pkgs:
            if msg_pkg not in pkg.exec_depends:
                results.append(
                    CheckResult(
                        name=f"dep:{pkg.name}",
                        passed=False,
                        message=f"uses {msg_pkg} messages but missing <exec_depend>{msg_pkg}</exec_depend> in package.xml",
                    )
                )

    if not results:
        results.append(
            CheckResult(
                name="dep:all",
                passed=True,
                message="All msg package dependencies declared",
            )
        )

    return results


def check_launch_node_refs(packages: list[PackageInfo]) -> list[CheckResult]:
    """Check that launch files reference valid package/executable combos."""
    results = []

    # Build lookup: package_name → set of executable names
    pkg_executables: dict[str, set[str]] = {}
    for pkg in packages:
        execs = set()
        for ep in pkg.entry_points:
            execs.add(ep.executable)
        # For ament_cmake, check install(PROGRAMS ...) in CMakeLists
        cmake = pkg.path / "CMakeLists.txt"
        if cmake.exists():
            content = cmake.read_text()
            # Match installed Python scripts
            for match in re.finditer(r"install\(PROGRAMS\s+src/(\w+\.py)", content):
                # Executable name is filename without .py
                execs.add(match.group(1).replace(".py", ""))
                # Also add with .py since ROS2 cmake installs keep the extension
                execs.add(match.group(1))
        pkg_executables[pkg.name] = execs

    # Find all launch files
    for pkg in packages:
        for launch_file in pkg.path.rglob("*.launch.py"):
            try:
                content = launch_file.read_text(encoding="utf-8")
            except (UnicodeDecodeError, ValueError):
                continue
            rel = launch_file.relative_to(WORKSPACE)

            # Match Node(package='X', executable='Y', ...)
            node_pattern = r"Node\([^)]*package=['\"](\w+)['\"][^)]*executable=['\"](\w+)['\"]"
            for match in re.finditer(node_pattern, content, re.DOTALL):
                ref_pkg = match.group(1)
                ref_exec = match.group(2)

                if ref_pkg not in pkg_executables:
                    # External package (realsense, etc.) — skip
                    if VERBOSE:
                        results.append(
                            CheckResult(
                                name=f"launch:{rel}",
                                passed=True,
                                message=f"references external package '{ref_pkg}' (not validated)",
                                severity="warn",
                            )
                        )
                    continue

                if ref_exec not in pkg_executables[ref_pkg]:
                    results.append(
                        CheckResult(
                            name=f"launch:{rel}",
                            passed=False,
                            message=f"references '{ref_pkg}/{ref_exec}' but no such executable found",
                        )
                    )
                else:
                    if VERBOSE:
                        results.append(
                            CheckResult(
                                name=f"launch:{rel}",
                                passed=True,
                                message=f"OK — {ref_pkg}/{ref_exec}",
                            )
                        )

    return results


def check_empty_files(packages: list[PackageInfo]) -> list[CheckResult]:
    """Flag empty Python files (excluding __init__.py)."""
    results = []
    for pkg in packages:
        pkg_src = pkg.path / pkg.name
        if not pkg_src.is_dir():
            continue
        for py_file in pkg_src.glob("*.py"):
            if py_file.name == "__init__.py":
                continue
            if py_file.stat().st_size == 0:
                results.append(
                    CheckResult(
                        name=f"empty:{py_file.relative_to(WORKSPACE)}",
                        passed=True,
                        message=f"empty placeholder (0 bytes)",
                        severity="warn",
                    )
                )
    return results


# ── Runner ──────────────────────────────────────────────────────────────────


def main():
    if not WORKSPACE.exists():
        print(f"ERROR: Workspace not found at {WORKSPACE}")
        sys.exit(1)

    print("=" * 60)
    print("  AIRHOUND Workspace Validator")
    print(f"  Workspace: {WORKSPACE}")
    print("=" * 60)
    print()

    # Discover
    packages = find_packages()
    print(f"Found {len(packages)} packages: {', '.join(p.name for p in packages)}")
    print()

    # Extract all topic endpoints
    all_endpoints = []
    for pkg in packages:
        for py_file in pkg.python_files:
            all_endpoints.extend(extract_topics(py_file))

    if VERBOSE:
        print(f"Found {len(all_endpoints)} topic endpoints")
        print()

    # Run checks
    all_results: list[CheckResult] = []

    print("── Entry Points ──")
    results = check_entry_points(packages)
    all_results.extend(results)
    print_results(results)

    print("── Intra-Package Imports ──")
    results = check_imports(packages)
    all_results.extend(results)
    print_results(results)

    print("── Topic Wiring ──")
    results = check_topic_wiring(all_endpoints)
    all_results.extend(results)
    print_results(results)

    print("── Package Dependencies ──")
    results = check_package_deps(packages)
    all_results.extend(results)
    print_results(results)

    print("── Launch File References ──")
    results = check_launch_node_refs(packages)
    all_results.extend(results)
    print_results(results)

    print("── Empty Files ──")
    results = check_empty_files(packages)
    all_results.extend(results)
    print_results(results)

    # Summary
    errors = [r for r in all_results if not r.passed and r.severity == "error"]
    warns = [r for r in all_results if r.passed and r.severity == "warn"]
    passed = [r for r in all_results if r.passed and r.severity != "warn"]

    print("=" * 60)
    print(f"  RESULTS: {len(passed)} passed, {len(warns)} warnings, {len(errors)} errors")
    if errors:
        print()
        print("  ERRORS:")
        for r in errors:
            print(f"    ✗ {r.name}: {r.message}")
    print("=" * 60)

    sys.exit(1 if errors else 0)


def print_results(results: list[CheckResult]):
    for r in results:
        if not r.passed:
            print(f"  ✗ {r.name}: {r.message}")
        elif r.severity == "warn":
            print(f"  ⚠ {r.name}: {r.message}")
        elif VERBOSE:
            print(f"  ✓ {r.name}: {r.message}")

    if not results:
        print("  (none)")
    print()


if __name__ == "__main__":
    main()
