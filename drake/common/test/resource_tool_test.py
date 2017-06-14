"""Performs simple within-sandbox tests for resource_tool, exercising its
command-line API through all corner cases.
"""

import re
import subprocess
import os
import unittest


class TestResourceTool(unittest.TestCase):
    def _check_call(self, args, expected_returncode=0):
        """Run resource_tool with the given args; return output.
        """
        try:
            output = subprocess.check_output(
                ["drake/common/resource_tool"] + args,
                stderr=subprocess.STDOUT)
            returncode = 0
        except subprocess.CalledProcessError as e:
            output = e.output
            returncode = e.returncode
        self.assertEqual(
            returncode, expected_returncode,
            "Expected returncode %r from %r but got %r with output %r" % (
                expected_returncode, args, returncode, output))
        return output

    def test_help(self):
        output = self._check_call([
            "--help",
            ], expected_returncode=1)
        self.assertTrue("Find Drake-related resources" in output)
        self.assertGreater(len(output), 1000)

    def test_no_arguments(self):
        output = self._check_call([], expected_returncode=1)
        self.assertGreater(len(output), 1000)

    def test_print_resource_path_found(self):
        output = self._check_call([
             "--print_resource_path",
             "drake/common/test/resource_tool_test_data.txt",
            ], expected_returncode=0)
        absolute_path = output.strip()
        with open(absolute_path, 'r') as data:
            self.assertEqual(
                data.read(),
                "Test data for drake/common/test/resource_tool_test.py.\n")

    def test_print_resource_path_error(self):
        output = self._check_call([
            "--print_resource_path",
            "drake/no_such_file",
            ], expected_returncode=1)
        self.assertTrue("could not find resource" in output)

    def test_help_example_is_correct(self):
        # Look at the usage message, and snarf its Pendulum example paths.
        output = self._check_call([
            "--help",
            ], expected_returncode=1)
        output = output.replace("\n", " ")
        m = re.search(
            (r"-print_resource_path.*`(drake.*)`.*" +
             r"absolute path.*?`/home/user/tmp/(drake/.*)`"),
            output)
        self.assertTrue(m is not None, "Could not match in " + repr(output))
        example_relpath, example_abspath = m.groups()

        # Ask for the help message's example and make sure it still works.
        output = self._check_call([
            "--print_resource_path",
            example_relpath,
            ], expected_returncode=0)
        absolute_path = output.strip()
        self.assertTrue(absolute_path.endswith(example_abspath))
        self.assertTrue(os.path.exists(absolute_path))


if __name__ == '__main__':
    unittest.main()
