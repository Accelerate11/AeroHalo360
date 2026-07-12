#!/usr/bin/env python3
import importlib.util
import pathlib
import tempfile
import unittest

MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / 'launch' / '_replay_utils.py'
spec = importlib.util.spec_from_file_location('replay_utils', MODULE_PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReplayUtilsTest(unittest.TestCase):
    def test_empty_and_missing_paths_fail(self):
        with self.assertRaises(RuntimeError):
            module.validate_bag_path('')
        with self.assertRaises(RuntimeError):
            module.validate_bag_path('/definitely/missing/aero_halo_bag')

    def test_directory_requires_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(RuntimeError):
                module.validate_bag_path(directory)
            metadata = pathlib.Path(directory) / 'metadata.yaml'
            metadata.write_text('rosbag2_bagfile_information: {}\n', encoding='utf-8')
            self.assertEqual(module.validate_bag_path(directory), directory)

    def test_only_supported_bag_files_are_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            invalid = pathlib.Path(directory) / 'bag.txt'
            invalid.write_text('', encoding='utf-8')
            with self.assertRaises(RuntimeError):
                module.validate_bag_path(str(invalid))
            valid = invalid.with_suffix('.mcap')
            valid.write_bytes(b'')
            self.assertEqual(module.validate_bag_path(str(valid)), str(valid))


if __name__ == '__main__':
    unittest.main()
