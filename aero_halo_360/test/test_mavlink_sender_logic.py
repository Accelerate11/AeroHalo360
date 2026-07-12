#!/usr/bin/env python3
import importlib.util
import os
import pathlib
import types
import unittest

os.environ['MAVLINK20'] = '0'

MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'mavlink_obstacle_sender.py'
spec = importlib.util.spec_from_file_location('mavlink_obstacle_sender', MODULE_PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FakeHeartbeat:
    def __init__(self, system_id, component_id, vehicle_type=None, autopilot=None):
        self._system_id = system_id
        self._component_id = component_id
        self.type = (
            module.mavutil.mavlink.MAV_TYPE_QUADROTOR
            if vehicle_type is None and module.mavutil is not None
            else vehicle_type
        )
        self.autopilot = (
            module.mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
            if autopilot is None and module.mavutil is not None
            else autopilot
        )

    def get_srcSystem(self):
        return self._system_id

    def get_srcComponent(self):
        return self._component_id


class FakeSectorMessage:
    def __init__(self):
        self.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=100, nanosec=0))
        self.source_sequence = 1
        self.source_age_ms = 10.0
        self.distances = [1001] * 72
        self.min_distance_cm = 25
        self.max_distance_cm = 1000
        self.increment_deg = 5.0
        self.angle_offset_deg = 0.0
        self.status = 0
        self.status_text = 'OK'


class FakeMav:
    def __init__(self, raise_on_obstacle=False):
        self.obstacle_calls = []
        self.raise_on_obstacle = raise_on_obstacle

    def obstacle_distance_send(self, *args):
        if self.raise_on_obstacle:
            raise OSError('serial disconnected')
        self.obstacle_calls.append(args)

    def statustext_send(self, *args):
        pass


class FakeMaster:
    def __init__(self, raise_on_obstacle=False):
        self.mav = FakeMav(raise_on_obstacle)
        self.closed = 0

    def close(self):
        self.closed += 1


class FakeLogger:
    def warn(self, message):
        self.last_warn = message


class MavlinkSenderLogicTest(unittest.TestCase):
    def test_external_mavlink1_setting_is_overridden(self):
        self.assertEqual(os.environ.get('MAVLINK20'), '1')

    def test_default_component_is_companion_not_autopilot(self):
        self.assertEqual(module.DEFAULT_SOURCE_SYSTEM, 1)
        self.assertNotEqual(module.default_source_component(), 1)

    def test_no_sector_input_fails_closed(self):
        distances, status, degraded = module.choose_distances_for_send(
            None, None, 10.0, True, 0.9, 'virtual_wall', 80)
        self.assertTrue(degraded)
        self.assertEqual(status, 'DEGRADED_NO_SECTOR')
        self.assertEqual(distances, [80] * 72)

    def test_stale_source_progress_fails_closed(self):
        distances, status, degraded = module.choose_distances_for_send(
            [1001] * 72, 10.0, 11.0, True, 0.9, 'virtual_wall', 80)
        self.assertTrue(degraded)
        self.assertEqual(status, 'DEGRADED_SECTOR_TIMEOUT')
        self.assertEqual(distances, [80] * 72)

    def test_upstream_degraded_fails_closed_even_when_topic_arrives(self):
        distances, status, degraded = module.choose_distances_for_send(
            [1001] * 72,
            10.0,
            10.1,
            True,
            0.9,
            'virtual_wall',
            80,
            upstream_degraded=True,
        )
        self.assertTrue(degraded)
        self.assertEqual(status, 'DEGRADED_UPSTREAM_STATUS')
        self.assertEqual(distances, [80] * 72)

    def test_contract_mismatch_is_rejected(self):
        message = FakeSectorMessage()
        self.assertEqual(
            module.validate_sector_contract(message, 25, 1000, 5, 0.0),
            '',
        )
        message.increment_deg = 10.0
        self.assertEqual(
            module.validate_sector_contract(message, 25, 1000, 5, 0.0),
            'DEGRADED_SECTOR_INCREMENT_MISMATCH',
        )
        message.increment_deg = 5.0
        message.header.stamp.sec = 0
        self.assertEqual(
            module.validate_sector_contract(message, 25, 1000, 5, 0.0),
            'DEGRADED_SOURCE_STAMP_ZERO',
        )

    @unittest.skipIf(module.mavutil is None, 'pymavlink 未安装')
    def test_heartbeat_only_accepts_configured_autopilot(self):
        valid = FakeHeartbeat(1, 1)
        wrong_system = FakeHeartbeat(2, 1)
        own = FakeHeartbeat(1, module.default_source_component())
        gcs = FakeHeartbeat(
            1,
            190,
            module.mavutil.mavlink.MAV_TYPE_GCS,
            module.mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        )
        self.assertTrue(module.heartbeat_matches(
            valid, 1, 1, 1, module.default_source_component()))
        self.assertFalse(module.heartbeat_matches(
            wrong_system, 1, 1, 1, module.default_source_component()))
        self.assertFalse(module.heartbeat_matches(
            own, 1, 1, 1, module.default_source_component()))
        self.assertFalse(module.heartbeat_matches(
            gcs, 1, 1, 1, module.default_source_component()))

    def test_connection_states_are_distinct(self):
        fake = types.SimpleNamespace(
            master=None,
            last_send_error='',
            wait_heartbeat_on_connect=True,
            heartbeat_is_fresh=lambda now: False,
        )
        state = module.MavlinkObstacleSender.mavlink_state(fake, 1.0)
        self.assertEqual(state, module.STATE_DISCONNECTED)

        fake.master = object()
        self.assertEqual(
            module.MavlinkObstacleSender.mavlink_state(fake, 1.0),
            module.STATE_HEARTBEAT_WAITING,
        )
        fake.heartbeat_is_fresh = lambda now: True
        self.assertEqual(
            module.MavlinkObstacleSender.mavlink_state(fake, 1.0),
            module.STATE_AUTOPILOT_CONNECTED,
        )
        fake.wait_heartbeat_on_connect = False
        self.assertEqual(
            module.MavlinkObstacleSender.mavlink_state(fake, 1.0),
            module.STATE_PORT_OPEN,
        )
        fake.master = None
        fake.last_send_error = 'failed'
        self.assertEqual(
            module.MavlinkObstacleSender.mavlink_state(fake, 1.0),
            module.STATE_SEND_ERROR,
        )

    def test_close_connection_releases_old_descriptor(self):
        master = FakeMaster()
        fake = types.SimpleNamespace(master=master)
        module.MavlinkObstacleSender.close_connection(fake)
        self.assertIsNone(fake.master)
        self.assertEqual(master.closed, 1)

    def make_sender_for_send(self, master):
        sender = types.SimpleNamespace(
            master=master,
            sensor_type=0,
            increment_deg=5,
            min_distance_cm=25,
            max_distance_cm=1000,
            angle_offset_deg=7.5,
            frame=12,
            statustext_on_degraded=False,
            last_send_error='',
            last_send_monotonic=None,
            send_intervals_ms=[],
            last_source_age_ms=10.0,
            last_source_to_mav_ms=None,
            last_sector_receive_monotonic=None,
            last_status_publish=0.0,
            send_count=0,
            connect_if_due=lambda: None,
            read_autopilot_heartbeat=lambda: None,
            current_distances_and_status=lambda: ([100] * 72, 'OK', False),
        )
        return sender

    def test_obstacle_distance_contains_mavlink2_extension_fields(self):
        master = FakeMaster()
        sender = self.make_sender_for_send(master)
        module.MavlinkObstacleSender.send_obstacle_distance(sender)
        self.assertEqual(len(master.mav.obstacle_calls), 1)
        arguments = master.mav.obstacle_calls[0]
        self.assertEqual(len(arguments), 9)
        self.assertEqual(arguments[6], 5.0)
        self.assertEqual(arguments[7], 7.5)
        self.assertEqual(arguments[8], 12)
        self.assertEqual(sender.send_count, 1)

    def test_send_error_closes_old_connection_and_schedules_backoff(self):
        master = FakeMaster(raise_on_obstacle=True)
        sender = self.make_sender_for_send(master)
        sender.get_logger = lambda: FakeLogger()
        sender.send_error_count = 0
        sender.reconnect_initial_ms = 500
        sender.reconnect_max_ms = 5000
        sender.current_reconnect_delay_ms = 500
        sender.next_connect_monotonic = 0.0
        sender.close_connection = (
            lambda: module.MavlinkObstacleSender.close_connection(sender)
        )
        sender.schedule_reconnect = (
            lambda error: module.MavlinkObstacleSender.schedule_reconnect(sender, error)
        )
        module.MavlinkObstacleSender.send_obstacle_distance(sender)
        self.assertIsNone(sender.master)
        self.assertEqual(master.closed, 1)
        self.assertEqual(sender.send_error_count, 1)
        self.assertGreater(sender.next_connect_monotonic, 0.0)

    def test_validation_error_becomes_virtual_wall(self):
        distances, status, degraded = module.choose_distances_for_send(
            [1001] * 72,
            10.0,
            10.1,
            True,
            0.9,
            'virtual_wall',
            80,
            validation_error='DEGRADED_SOURCE_SEQUENCE_BACKWARD',
        )
        self.assertTrue(degraded)
        self.assertEqual(status, 'DEGRADED_SOURCE_SEQUENCE_BACKWARD')
        self.assertEqual(distances, [80] * 72)



    def test_sender_parameter_validation_rejects_unsafe_contract(self):
        values = {
            'connection': '/dev/ttyTEST',
            'baud': 921600,
            'source_system': 1,
            'source_component': 191,
            'target_system': 1,
            'target_component': 1,
            'send_rate_hz': 10.0,
            'heartbeat_rate_hz': 1.0,
            'sensor_type': 0,
            'frame': 9,
            'min_distance_cm': 25,
            'max_distance_cm': 1000,
            'increment_deg': 5,
            'angle_offset_deg': 0.0,
            'input_warn_timeout_ms': 600,
            'input_fail_timeout_ms': 1000,
            'recovery_healthy_frames': 3,
            'max_source_age_ms': 1000,
            'fail_closed': True,
            'startup_mode': 'virtual_wall',
            'virtual_wall_cm': 80,
            'heartbeat_timeout_ms': 3000,
            'reconnect_initial_ms': 500,
            'reconnect_max_ms': 10000,
            'get_logger': lambda: FakeLogger(),
        }
        fake = types.SimpleNamespace(**values)
        with self.assertRaises(ValueError):
            module.MavlinkObstacleSender.validate_parameters(fake)
        fake.frame = 12
        fake.increment_deg = 10
        with self.assertRaises(ValueError):
            module.MavlinkObstacleSender.validate_parameters(fake)
        fake.increment_deg = 5
        fake.input_warn_timeout_ms = -1
        with self.assertRaises(ValueError):
            module.MavlinkObstacleSender.validate_parameters(fake)

    def test_two_stage_guard_tolerates_332ms_jitter(self):
        guard = module.TwoStageStaleGuard(0.4, 0.8, 1)
        guard.mark_healthy_frame(1, 10.0)
        self.assertEqual(guard.evaluate(10.332), ('OK', False))

    def test_two_stage_guard_warns_then_fails_closed(self):
        guard = module.TwoStageStaleGuard(0.4, 0.8, 1)
        guard.mark_healthy_frame(1, 10.0)
        self.assertEqual(guard.evaluate(10.5), ('WARN_SECTOR_STALE', False))
        self.assertEqual(guard.evaluate(10.9), ('DEGRADED_SECTOR_TIMEOUT', True))

    def test_two_stage_guard_requires_new_frames_to_recover(self):
        guard = module.TwoStageStaleGuard(0.4, 0.8, 3)
        guard.mark_healthy_frame(1, 10.0)
        self.assertTrue(guard.evaluate(10.0)[1])
        self.assertTrue(guard.evaluate(10.1)[1])
        guard.mark_healthy_frame(2, 10.2)
        self.assertTrue(guard.evaluate(10.2)[1])
        guard.mark_healthy_frame(3, 10.3)
        self.assertEqual(guard.evaluate(10.3), ('OK', False))
        self.assertGreaterEqual(guard.transition_count, 1)

    def test_contract_rejects_min_max_and_offset_mismatch(self):
        message = FakeSectorMessage()
        message.min_distance_cm = 24
        self.assertEqual(
            module.validate_sector_contract(message, 25, 1000, 5, 0.0),
            'DEGRADED_SECTOR_MIN_MISMATCH',
        )
        message.min_distance_cm = 25
        message.max_distance_cm = 999
        self.assertEqual(
            module.validate_sector_contract(message, 25, 1000, 5, 0.0),
            'DEGRADED_SECTOR_MAX_MISMATCH',
        )
        message.max_distance_cm = 1000
        message.angle_offset_deg = 5.0
        self.assertEqual(
            module.validate_sector_contract(message, 25, 1000, 5, 0.0),
            'DEGRADED_SECTOR_OFFSET_MISMATCH',
        )

    def test_repeated_close_does_not_leak_fake_descriptors(self):
        masters = [FakeMaster() for _ in range(20)]
        for master in masters:
            fake = types.SimpleNamespace(master=master)
            module.MavlinkObstacleSender.close_connection(fake)
            module.MavlinkObstacleSender.close_connection(fake)
        self.assertTrue(all(master.closed == 1 for master in masters))
if __name__ == '__main__':
    unittest.main()
