"""Configure-only regression tests for MSVC Release compiler flags.

Run with ``python -m unittest discover -s tests -p test_cmake_release.py -v``
from a Visual Studio developer environment with CMake and Ninja available.
"""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


@unittest.skipUnless(sys.platform == "win32" and shutil.which("cl"), "requires MSVC")
class TestMSVCReleaseFlags(unittest.TestCase):
    def check_release_flags(self, override=None):
        source = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(prefix="coacd-release-") as directory:
            command = [
                "cmake", "-S", str(source), "-B", directory, "-G", "Ninja",
                "-DCMAKE_BUILD_TYPE=Release", "-DWITH_3RD_PARTY_LIBS=OFF",
                "-DWITH_OPENMP=OFF", "-DWITH_STD_THREADS=OFF",
            ]
            if override:
                command.append("-DCMAKE_CXX_FLAGS_RELEASE=" + override)
            subprocess.run(command, check=True, capture_output=True, text=True)
            entries = json.loads((Path(directory) / "compile_commands.json").read_text())
            self.assertTrue(entries)
            for entry in entries:
                flags = entry["command"]
                with self.subTest(file=entry["file"]):
                    self.assertNotRegex(flags, r"(?:^|\s)[/-]O3(?:\s|$)")
                    expected = "O1" if override else "O2"
                    self.assertRegex(flags, rf"(?:^|\s)[/-]{expected}(?:\s|$)")
                    self.assertRegex(flags, r"(?:^|\s)[/-]DNDEBUG(?:\s|$)")
                    if override:
                        self.assertRegex(flags, r"(?:^|\s)[/-]DCOACD_RELEASE_SENTINEL(?:\s|$)")

    def test_default_release_uses_msvc_optimization(self):
        self.check_release_flags()

    def test_explicit_release_flags_are_preserved(self):
        self.check_release_flags("/O1 /DNDEBUG /DCOACD_RELEASE_SENTINEL")


if __name__ == "__main__":
    unittest.main()
