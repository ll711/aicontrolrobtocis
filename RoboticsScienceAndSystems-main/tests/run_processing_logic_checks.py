"""Execute actual RouteEditor/telemetry PDE logic with minimal Java API stubs.

This is a headless logic test, not a Processing compiler or a GUI/render test.
Requires an existing JDK; pass --java-home when Java is not on PATH. Generated
Java/classes stay in build-host/processing-logic and never alter sketch sources.
"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess


def processing_float_literals(source):
    # Processing makes unsuffixed decimal literals float. Preserve strings,
    # character literals, comments, integer types, and explicitly typed literals.
    token = re.compile(r'("(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*[\s\S]*?\*/)|'
                       r'(?<![\w.])((?:\d+\.\d*|\.\d+)(?:[eE][+-]?\d+)?)([fFdD]?)(?![\w.])')
    return token.sub(lambda match: match[1] if match[1] is not None else match[2] + (match[3] or "f"), source)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--java-home", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    sketch = root / "DigitalTwin"
    app_source = (sketch / "DigitalTwin.pde").read_text(encoding="utf-8")
    sim_source = (sketch / "Simulator.pde").read_text(encoding="utf-8")
    for handler in ["mousePressed", "mouseDragged", "mouseReleased", "mouseWheel"]:
        if not re.search(r"simulator\.routeEditor\." + handler + r"\s*\(", app_source):
            raise AssertionError(f"Sketch does not forward {handler} to route editor")
    if "routeEditor.handleLine(lastTcpLine, now)" not in sim_source:
        raise AssertionError("Simulator does not route protocol replies before telemetry")
    if not re.search(r"scale\(\s*zoom\s*,\s*-zoom\s*\)", sim_source):
        raise AssertionError("World view must retain upward-positive Y coordinates")
    java = str(args.java_home / "bin" / "java.exe") if args.java_home else shutil.which("java")
    javac = str(args.java_home / "bin" / "javac.exe") if args.java_home else shutil.which("javac")
    if not java or not javac:
        raise SystemExit("An existing JDK is required: use --java-home PATH. Processing is not installed by this test.")
    output = root / "build-host" / "processing-logic"
    output.mkdir(parents=True, exist_ok=True)
    template = (root / "tests" / "ProcessingLogicHarness.java.template").read_text(encoding="utf-8")
    pde = "\n".join(processing_float_literals((sketch / name).read_text(encoding="utf-8"))
                    for name in ["RouteEditor.pde", "TCPTelemetry.pde"])
    source = output / "ProcessingLogicHarness.java"
    source.write_text(template.replace("// INSERT_ACTUAL_PDE_SOURCE", pde), encoding="utf-8")
    subprocess.run([javac, "-encoding", "UTF-8", "-d", str(output), str(source),
                    str(sketch / "TreeModels.java"), str(sketch / "TwoTreeController.java")], check=True)
    subprocess.run([java, "-cp", str(output), "ProcessingLogicHarness"], check=True)
    print("PASS: sketch mouse/telemetry/world-view wiring checks; headless PDE logic only (no GUI rendering).")


if __name__ == "__main__":
    main()
