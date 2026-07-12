#!/usr/bin/env python3
import importlib.util
import pathlib
import tempfile
import textwrap
import unittest

MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / 'launch' / '_config_layering.py'
spec = importlib.util.spec_from_file_location('config_layering', MODULE_PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ConfigLayeringTest(unittest.TestCase):
    def write_config(self, directory, name, body):
        path = pathlib.Path(directory) / name
        path.write_text(textwrap.dedent(body), encoding='utf-8')
        return str(path)

    def test_base_profile_cli_priority(self):
        with tempfile.TemporaryDirectory() as directory:
            base = self.write_config(directory, 'base.yaml', '''
                /**:
                  ros__parameters:
                    target_frame: base_link
                    debug:
                      publish_markers: true
                    mavlink:
                      baud: 115200
            ''')
            profile = self.write_config(directory, 'profile.yaml', '''
                /**:
                  ros__parameters:
                    debug:
                      publish_markers: false
                    mavlink:
                      baud: 460800
            ''')
            result = module.build_effective_parameters(
                base, profile, {'mavlink.baud': 921600})
            self.assertEqual(result['target_frame'], 'base_link')
            self.assertFalse(result['debug.publish_markers'])
            self.assertEqual(result['mavlink.baud'], 921600)

    def test_installation_overlay_is_loaded_before_cli(self):
        with tempfile.TemporaryDirectory() as directory:
            base = self.write_config(directory, 'base.yaml', '''
                /**:
                  ros__parameters:
                    target_frame: base_link
            ''')
            installation = self.write_config(directory, 'install.yaml', '''
                /**:
                  ros__parameters:
                    installation:
                      static_tf:
                        enable: true
                        xyz: [1.0, 2.0, 3.0]
            ''')
            result = module.build_effective_parameters(
                base, None, {'target_frame': 'vehicle'}, installation)
            self.assertEqual(result['installation.static_tf.xyz'], [1.0, 2.0, 3.0])
            self.assertEqual(result['target_frame'], 'vehicle')

    def test_empty_cli_values_do_not_override_yaml(self):
        result = module.optional_cli_overrides(
            {'target_frame': '', 'debug.publish_markers': ''},
            {'debug.publish_markers'},
            set(),
        )
        self.assertEqual(result, {})

    def test_cli_types_are_strict(self):
        result = module.optional_cli_overrides(
            {'debug.publish_markers': 'false', 'mavlink.baud': '921600'},
            {'debug.publish_markers'},
            {'mavlink.baud'},
        )
        self.assertIs(result['debug.publish_markers'], False)
        self.assertEqual(result['mavlink.baud'], 921600)
        with self.assertRaises(RuntimeError):
            module.parse_bool('flag', 'maybe')

    def test_missing_or_invalid_file_fails(self):
        with self.assertRaises(RuntimeError):
            module.normalize_config_path('/definitely/missing/aero.yaml', 'profile')
        with tempfile.TemporaryDirectory() as directory:
            invalid = self.write_config(directory, 'invalid.yaml', 'value: 1')
            with self.assertRaises(RuntimeError):
                module.load_parameter_file(invalid)

    def test_vector_requires_finite_exact_count(self):
        self.assertEqual(module.parse_vector('xyz', '1, 2, 3', 3), (1.0, 2.0, 3.0))
        with self.assertRaises(RuntimeError):
            module.parse_vector('xyz', '1 2', 3)
        with self.assertRaises(RuntimeError):
            module.parse_vector('xyz', '1 2 nan', 3)


if __name__ == '__main__':
    unittest.main()
