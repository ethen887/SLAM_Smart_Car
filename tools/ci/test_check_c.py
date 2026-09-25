"""Verify scope and failure propagation without requiring native analysis binaries."""

from pathlib import Path
from subprocess import CompletedProcess
import unittest
from unittest.mock import patch

import check_c


class CCheckTests(unittest.TestCase):
    @patch("check_c.os.chdir")
    @patch("check_c.subprocess.check_output", return_value=b"README.md\0")
    @patch("check_c.shutil.which")
    def test_empty_repository_skips_tools(self, which, output, chdir):
        self.assertEqual(check_c.main(), 0)
        which.assert_not_called()

    @patch("check_c.os.chdir")
    @patch("check_c.subprocess.check_output", return_value=b"src/main.c\0")
    @patch("check_c.shutil.which", return_value=None)
    def test_missing_tools_fail_when_code_exists(self, which, output, chdir):
        self.assertEqual(check_c.main(), 1)

    def test_scope_and_tool_failures(self):
        tracked = (b"src/main.c\0include/main.h\0firmware/Core/Src/app.c\0"
                   b"vendor/sdk.c\0firmware/Drivers/hal.c\0firmware/ra/fsp.c\0")
        for failing in (None, "format", "analysis"):
            with self.subTest(failing=failing):
                formatted = []
                analyzed = []

                def run(command, **kwargs):
                    result = 0
                    if "--dry-run" in command:
                        formatted.append(command[-1])
                        result = int(failing == "format")
                    for arg in command:
                        if arg.startswith("--file-list="):
                            analyzed.extend(Path(arg.split("=", 1)[1]).read_text().splitlines())
                            result = int(failing == "analysis")
                    return CompletedProcess(command, result)

                with patch("check_c.os.chdir"), \
                     patch("check_c.subprocess.check_output", return_value=tracked), \
                     patch("check_c.shutil.which", side_effect=lambda tool: tool), \
                     patch("check_c.subprocess.run", side_effect=run):
                    self.assertEqual(check_c.main(), int(failing is not None))
                self.assertEqual(formatted, ["firmware/Core/Src/app.c", "include/main.h", "src/main.c"])
                self.assertEqual(analyzed, ["firmware/Core/Src/app.c", "src/main.c"])


if __name__ == "__main__":
    unittest.main()
