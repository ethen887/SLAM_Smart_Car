"""Check maintained C files; this is not an MCU firmware build."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def main():
    os.chdir(ROOT)
    tracked = subprocess.check_output(["git", "ls-files", "-z"]).decode("utf-8").split("\0")
    excluded = {"vendor", "third_party", "generated", "Drivers", "Middlewares", "ra"}
    files = sorted(p for p in tracked if Path(p).suffix in {".c", ".h"}
                   and not excluded.intersection(Path(p).parts[:-1]))
    if not files:
        print("No maintained C/H files: checks skipped. Firmware build NOT configured.")
        return 0
    formatter = shutil.which("clang-format-18")
    analyzer = shutil.which("cppcheck")
    if not formatter or not analyzer:
        print("Install clang-format-18 and cppcheck before running this check.")
        return 1
    subprocess.run([formatter, "--version"], check=True)
    subprocess.run([analyzer, "--version"], check=True)
    failed = False
    for path in files:
        result = subprocess.run([formatter, "--dry-run", "--Werror", "--", path], check=False)
        failed |= result.returncode != 0
    sources = [p for p in files if p.endswith(".c")]
    if sources:
        with tempfile.TemporaryDirectory() as directory:
            file_list = Path(directory) / "sources.txt"
            file_list.write_text("\n".join(sources) + "\n", encoding="utf-8")
            result = subprocess.run([
                analyzer, "--enable=warning,performance,portability", "--error-exitcode=1",
                "--inline-suppr", "--suppress=missingIncludeSystem", "--std=c11",
                "--language=c", "--template=gcc", f"--file-list={file_list}",
            ], check=False)
            failed |= result.returncode != 0
    print("Generic C checks only: MCU compilation and hardware tests are separate requirements.")
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
