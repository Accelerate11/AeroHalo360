#!/usr/bin/env python3
import math
import os
import time
from typing import List, Optional, Tuple

# 必须覆盖外部错误预设；MAVLink 1 无法可靠携带 OBSTACLE_DISTANCE 扩展字段。
os.environ['MAVLINK20'] = '1'

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

from aero_halo_360.msg import SectorDistances

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover - 单元测试允许在没有 pymavlink 时加载纯函数
    mavutil = None

SECTOR_COUNT = 72
DEFAULT_SOURCE_SYSTEM = 1
DEFAULT_SOURCE_COMPONENT_FALLBACK = 191
UINT16_MAX = 65535
UPSTREAM_OK = 0
UPSTREAM_WARN_FILTERED_EMPTY = 7
UPSTREAM_WARN_CLOUD_STALE = 14

STATE_DISCONNECTED = 'DISCONNECTED'
STATE_PORT_OPEN = 'PORT_OPEN'
STATE_HEARTBEAT_WAITING = 'HEARTBEAT_WAITING'
STATE_AUTOPILOT_CONNECTED = 'AUTOPILOT_CONNECTED'
STATE_SEND_ERROR = 'SEND_ERROR'


def to_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ('1', 'true', 'yes', 'on'):
            return True
        if normalized in ('0', 'false', 'no', 'off'):
            return False
        raise ValueError(f'无法解析布尔值: {value}')
    return bool(value)


def default_source_component() -> int:
    if mavutil is None:
        return DEFAULT_SOURCE_COMPONENT_FALLBACK
    return int(getattr(
        mavutil.mavlink,
        'MAV_COMP_ID_ONBOARD_COMPUTER',
        DEFAULT_SOURCE_COMPONENT_FALLBACK,
    ))


def virtual_wall_distances(virtual_wall_cm: int) -> List[int]:
    value = max(0, min(UINT16_MAX - 1, int(virtual_wall_cm)))
    return [value] * SECTOR_COUNT


def unknown_distances() -> List[int]:
    return [UINT16_MAX] * SECTOR_COUNT


def age_seconds(last_time: Optional[float], now: float) -> Optional[float]:
    if last_time is None:
        return None
    return max(0.0, now - last_time)



class TwoStageStaleGuard:
    """使用单调时钟实现警告、失效闭锁和按新帧恢复。"""

    def __init__(self, warn_timeout_s: float, fail_timeout_s: float, recovery_frames: int):
        if warn_timeout_s <= 0.0 or fail_timeout_s <= warn_timeout_s:
            raise ValueError('陈旧阈值必须满足 0 < warn < fail')
        if recovery_frames < 1:
            raise ValueError('恢复健康帧数必须至少为 1')
        self.warn_timeout_s = warn_timeout_s
        self.fail_timeout_s = fail_timeout_s
        self.recovery_frames = recovery_frames
        self.last_progress: Optional[float] = None
        self.last_sequence: Optional[int] = None
        self.latched = True
        self.healthy_frames = 0
        self.transition_count = 0
        self.last_state = 'DEGRADED_NO_SECTOR'
        self.transition_reason = 'STARTUP_NO_SECTOR'
        self.intervals_ms = []

    def _transition(self, state: str):
        if state != self.last_state:
            self.transition_count += 1
            self.transition_reason = f'{self.last_state} -> {state}'
            self.last_state = state

    def mark_fault(self, reason: str):
        self.latched = True
        self.healthy_frames = 0
        self._transition(reason)

    def mark_healthy_frame(self, sequence: int, now: float):
        if self.last_progress is not None and now - self.last_progress > self.fail_timeout_s:
            self.latched = True
            self.healthy_frames = 0
        if self.last_sequence is None or sequence > self.last_sequence:
            if self.last_progress is not None:
                self.intervals_ms.append(max(0.0, now - self.last_progress) * 1000.0)
                self.intervals_ms = self.intervals_ms[-200:]
            self.last_sequence = sequence
            self.last_progress = now
            if self.latched:
                self.healthy_frames += 1
                if self.healthy_frames >= self.recovery_frames:
                    self.latched = False
                    self.healthy_frames = 0

    def interval_metrics(self):
        if not self.intervals_ms:
            return 0.0, 0.0, 0.0
        ordered = sorted(self.intervals_ms)
        p95_index = int(0.95 * (len(ordered) - 1))
        return self.intervals_ms[-1], max(ordered), ordered[p95_index]

    def evaluate(self, now: float) -> Tuple[str, bool]:
        if self.last_progress is None:
            state = 'DEGRADED_NO_SECTOR'
            hard_failure = True
        else:
            age = max(0.0, now - self.last_progress)
            if age > self.fail_timeout_s:
                self.latched = True
                self.healthy_frames = 0
                state = 'DEGRADED_SECTOR_TIMEOUT'
                hard_failure = True
            elif self.latched:
                state = 'DEGRADED_SECTOR_RECOVERY'
                hard_failure = True
            elif age > self.warn_timeout_s:
                state = 'WARN_SECTOR_STALE'
                hard_failure = False
            else:
                state = 'OK'
                hard_failure = False
        self._transition(state)
        return state, hard_failure

def choose_distances_for_send(
    last_distances: Optional[List[int]],
    last_source_progress_monotonic: Optional[float],
    now_monotonic: float,
    fail_closed: bool,
    input_timeout_s: float,
    startup_mode: str,
    virtual_wall_cm: int,
    upstream_degraded: bool = False,
    validation_error: str = '',
) -> Tuple[Optional[List[int]], Optional[str], bool]:
    if validation_error:
        if fail_closed:
            return virtual_wall_distances(virtual_wall_cm), validation_error, True
        return None, validation_error, True

    if upstream_degraded:
        if fail_closed:
            return virtual_wall_distances(virtual_wall_cm), 'DEGRADED_UPSTREAM_STATUS', True
        return last_distances, 'DEGRADED_UPSTREAM_STATUS', True

    if last_source_progress_monotonic is None or last_distances is None:
        if not fail_closed:
            return None, 'WAITING_FOR_SECTOR', True
        if startup_mode == 'unknown':
            return unknown_distances(), 'DEGRADED_NO_SECTOR', True
        if startup_mode == 'hold':
            return None, 'WAITING_FOR_SECTOR', True
        return virtual_wall_distances(virtual_wall_cm), 'DEGRADED_NO_SECTOR', True

    source_progress_age = now_monotonic - last_source_progress_monotonic
    if source_progress_age > input_timeout_s:
        if fail_closed:
            return virtual_wall_distances(virtual_wall_cm), 'DEGRADED_SECTOR_TIMEOUT', True
        return last_distances, 'DEGRADED_SECTOR_TIMEOUT', True

    return last_distances, None, False


def heartbeat_source(message) -> Tuple[int, int]:
    return int(message.get_srcSystem()), int(message.get_srcComponent())


def heartbeat_is_autopilot_candidate(message, source_system: int, source_component: int) -> bool:
    system_id, component_id = heartbeat_source(message)
    if system_id == source_system and component_id == source_component:
        return False
    if mavutil is None:
        return True
    if int(getattr(message, 'type', -1)) == int(mavutil.mavlink.MAV_TYPE_GCS):
        return False
    if int(getattr(message, 'autopilot', -1)) == int(mavutil.mavlink.MAV_AUTOPILOT_INVALID):
        return False
    return True


def heartbeat_matches(
    message,
    target_system: int,
    target_component: int,
    source_system: int,
    source_component: int,
) -> bool:
    if not heartbeat_is_autopilot_candidate(message, source_system, source_component):
        return False
    system_id, component_id = heartbeat_source(message)
    if target_system > 0 and system_id != target_system:
        return False
    if target_component > 0 and component_id != target_component:
        return False
    return True


def message_stamp_seconds(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1_000_000_000.0


def validate_sector_contract(
    message,
    expected_min_cm: int,
    expected_max_cm: int,
    expected_increment_deg: int,
    expected_angle_offset_deg: float,
) -> str:
    if len(message.distances) != SECTOR_COUNT:
        return 'DEGRADED_SECTOR_LENGTH'
    if int(message.min_distance_cm) != expected_min_cm:
        return 'DEGRADED_SECTOR_MIN_MISMATCH'
    if int(message.max_distance_cm) != expected_max_cm:
        return 'DEGRADED_SECTOR_MAX_MISMATCH'
    if not math.isclose(
        float(message.increment_deg),
        float(expected_increment_deg),
        abs_tol=1.0e-4,
    ):
        return 'DEGRADED_SECTOR_INCREMENT_MISMATCH'
    if not math.isclose(
        float(message.angle_offset_deg),
        float(expected_angle_offset_deg),
        abs_tol=1.0e-4,
    ):
        return 'DEGRADED_SECTOR_OFFSET_MISMATCH'
    if message_stamp_seconds(message) <= 0.0:
        return 'DEGRADED_SOURCE_STAMP_ZERO'
    return ''


class MavlinkObstacleSender(Node):
    def __init__(self):
        super().__init__('mavlink_obstacle_sender')

        self.profile_name = str(self.param('runtime.profile_name', 'default'))
        self.connection = str(self.param('mavlink.connection', 'udpout:127.0.0.1:14550'))
        self.baud = int(self.param('mavlink.baud', 921600))
        self.source_system = int(self.param('mavlink.source_system', DEFAULT_SOURCE_SYSTEM))
        self.source_component = int(self.param('mavlink.source_component', default_source_component()))
        self.target_system = int(self.param('mavlink.target_system', 1))
        self.target_component = int(self.param('mavlink.target_component', 1))
        self.send_rate_hz = float(self.param('mavlink.send_rate_hz', 10.0))
        self.heartbeat_rate_hz = float(self.param('mavlink.heartbeat_rate_hz', 1.0))
        self.sensor_type = int(self.param('mavlink.sensor_type', 0))
        self.frame = int(self.param('mavlink.frame', 12))
        self.min_distance_cm = int(self.param('mavlink.min_distance_cm', 25))
        self.max_distance_cm = int(self.param('mavlink.max_distance_cm', 1000))
        self.increment_deg = int(self.param('mavlink.increment_deg', 5))
        self.angle_offset_deg = float(self.param('mavlink.angle_offset_deg', 0.0))
        self.input_warn_timeout_ms = int(
            self.param('mavlink.input_warn_timeout_ms', 600))
        self.input_fail_timeout_ms = int(
            self.param('mavlink.input_fail_timeout_ms', 1000))
        self.recovery_healthy_frames = int(
            self.param('mavlink.recovery_healthy_frames', 3))
        self.max_source_age_ms = int(
            self.param('mavlink.max_source_age_ms', self.input_fail_timeout_ms))
        self.fail_closed = to_bool(self.param('mavlink.fail_closed', True))
        self.startup_mode = str(self.param('mavlink.startup_mode', 'virtual_wall'))
        self.virtual_wall_cm = int(self.param('mavlink.virtual_wall_cm', 80))
        self.statustext_on_degraded = to_bool(
            self.param('mavlink.statustext_on_degraded', True))
        self.wait_heartbeat_on_connect = to_bool(
            self.param('mavlink.wait_heartbeat_on_connect', True))
        self.heartbeat_timeout_ms = int(self.param('mavlink.heartbeat_timeout_ms', 3000))
        self.reconnect_initial_ms = int(self.param('mavlink.reconnect_initial_ms', 500))
        self.reconnect_max_ms = int(self.param('mavlink.reconnect_max_ms', 10000))
        self.sector_topic = str(
            self.param('sector_topic', '/aero_halo_360/sector_distances'))
        self.diagnostics_topic = str(self.param(
            'mavlink.diagnostics_topic',
            '/aero_halo_360/mavlink_diagnostics',
        ))

        self.validate_parameters()
        self.stale_guard = TwoStageStaleGuard(
            self.input_warn_timeout_ms / 1000.0,
            self.input_fail_timeout_ms / 1000.0,
            self.recovery_healthy_frames,
        )
        if mavutil is None:
            raise RuntimeError(
                '缺少 pymavlink，sender 无法运行。请执行：'
                'python3 -m pip install --user pymavlink')

        self.master = None
        self.last_distances: Optional[List[int]] = None
        self.last_sector_receive_monotonic: Optional[float] = None
        self.last_source_progress_monotonic: Optional[float] = None
        self.last_source_sequence: Optional[int] = None
        self.last_source_age_ms: Optional[float] = None
        self.last_source_to_mav_ms: Optional[float] = None
        self.last_upstream_status = UPSTREAM_OK
        self.last_status_text = 'WAITING_FOR_SECTOR'
        self.last_validation_error = ''
        self.last_status_publish = 0.0
        self.last_send_error = ''
        self.last_send_monotonic: Optional[float] = None
        self.send_intervals_ms = []
        self.last_autopilot_heartbeat_monotonic: Optional[float] = None
        self.locked_target_system: Optional[int] = None
        self.locked_target_component: Optional[int] = None
        self.send_count = 0
        self.send_error_count = 0
        self.reconnect_count = 0
        self.ignored_heartbeat_count = 0
        self.next_connect_monotonic = 0.0
        self.current_reconnect_delay_ms = self.reconnect_initial_ms
        self.last_reported_state = ''

        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, self.diagnostics_topic, 10)
        self.create_subscription(
            SectorDistances, self.sector_topic, self.on_sector_distances, 10)
        self.create_timer(
            1.0 / self.send_rate_hz, self.send_obstacle_distance)
        self.create_timer(
            1.0 / self.heartbeat_rate_hz, self.send_heartbeat)
        self.create_timer(1.0, self.publish_diagnostics)

        self.connect_if_due(force=True)

    def param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def validate_parameters(self):
        errors = []
        if not self.connection.strip():
            errors.append('mavlink.connection 不能为空')
        if self.baud <= 0:
            errors.append(f'mavlink.baud 必须大于 0，当前为 {self.baud}')
        for name, value in (
            ('mavlink.source_system', self.source_system),
            ('mavlink.source_component', self.source_component),
            ('mavlink.target_system', self.target_system),
            ('mavlink.target_component', self.target_component),
        ):
            if value < 0 or value > 255:
                errors.append(f'{name} 必须在 0..255，当前为 {value}')
        if self.source_system == 0 or self.source_component == 0:
            errors.append('mavlink.source_system/source_component 不能为 0')
        if self.send_rate_hz <= 0.0 or not math.isfinite(self.send_rate_hz):
            errors.append(f'mavlink.send_rate_hz 必须为有限正数，当前为 {self.send_rate_hz}')
        if self.heartbeat_rate_hz <= 0.0 or not math.isfinite(self.heartbeat_rate_hz):
            errors.append(
                f'mavlink.heartbeat_rate_hz 必须为有限正数，当前为 {self.heartbeat_rate_hz}')
        if self.sensor_type < 0 or self.sensor_type > 255:
            errors.append(f'mavlink.sensor_type 必须在 0..255，当前为 {self.sensor_type}')
        if self.frame != 12:
            errors.append(f'mavlink.frame 固定要求 MAV_FRAME_BODY_FRD(12)，当前为 {self.frame}')
        if self.increment_deg != 5:
            errors.append(f'mavlink.increment_deg 固定要求 5，当前为 {self.increment_deg}')
        if self.min_distance_cm <= 0 or self.max_distance_cm > UINT16_MAX - 1:
            errors.append('mavlink min/max 距离必须位于 1..65534 cm')
        if self.min_distance_cm >= self.max_distance_cm:
            errors.append('mavlink.min_distance_cm 必须小于 mavlink.max_distance_cm')
        if self.input_warn_timeout_ms <= 0 or \
                self.input_fail_timeout_ms <= self.input_warn_timeout_ms:
            errors.append('mavlink 必须满足 0 < input_warn_timeout_ms < input_fail_timeout_ms')
        if self.recovery_healthy_frames < 1:
            errors.append('mavlink.recovery_healthy_frames 必须至少为 1')
        if self.max_source_age_ms <= 0:
            errors.append('mavlink.max_source_age_ms 必须大于 0')
        if self.heartbeat_timeout_ms <= 0:
            errors.append('mavlink.heartbeat_timeout_ms 必须大于 0')
        if self.startup_mode not in ('virtual_wall', 'unknown', 'hold'):
            errors.append(
                f'mavlink.startup_mode 只能是 virtual_wall/unknown/hold，当前为 {self.startup_mode}')
        if self.virtual_wall_cm < self.min_distance_cm or \
                self.virtual_wall_cm > self.max_distance_cm:
            errors.append('mavlink.virtual_wall_cm 必须位于 min/max 距离范围内')
        if self.reconnect_initial_ms <= 0 or self.reconnect_max_ms < self.reconnect_initial_ms:
            errors.append('MAVLink 重连退避必须满足 0 < initial <= max')
        if not math.isfinite(self.angle_offset_deg):
            errors.append('mavlink.angle_offset_deg 必须是有限数值')
        if errors:
            raise ValueError('MAVLink 参数校验失败：\n- ' + '\n- '.join(errors))
        if not self.fail_closed:
            self.get_logger().warn('警告：mavlink.fail_closed=false，必须由操作者显式承担风险')

    def close_connection(self):
        old_master = self.master
        self.master = None
        if old_master is None:
            return
        try:
            close_method = getattr(old_master, 'close', None)
            if callable(close_method):
                close_method()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'关闭旧 MAVLink 连接时出错: {exc}')

    def schedule_reconnect(self, error: str):
        self.last_send_error = error
        self.send_error_count += 1
        self.close_connection()
        now = time.monotonic()
        self.next_connect_monotonic = now + self.current_reconnect_delay_ms / 1000.0
        self.current_reconnect_delay_ms = min(
            self.reconnect_max_ms,
            self.current_reconnect_delay_ms * 2,
        )

    def connect_if_due(self, force: bool = False):
        if self.master is not None:
            return
        now = time.monotonic()
        if not force and now < self.next_connect_monotonic:
            return
        try:
            self.master = mavutil.mavlink_connection(
                self.connection,
                baud=self.baud,
                source_system=self.source_system,
                source_component=self.source_component,
                force_connected=False,
            )
            self.last_send_error = ''
            self.current_reconnect_delay_ms = self.reconnect_initial_ms
            self.reconnect_count += 1
            self.get_logger().info(f'MAVLink 端口已打开: {self.connection}')
        except Exception as exc:  # noqa: BLE001
            self.master = None
            self.last_send_error = str(exc)
            self.next_connect_monotonic = now + self.current_reconnect_delay_ms / 1000.0
            self.current_reconnect_delay_ms = min(
                self.reconnect_max_ms,
                self.current_reconnect_delay_ms * 2,
            )
            self.get_logger().error(
                f'MAVLink 端口打开失败，稍后重试: {exc}')

    def on_sector_distances(self, msg: SectorDistances):
        now_monotonic = time.monotonic()
        self.last_sector_receive_monotonic = now_monotonic
        self.last_upstream_status = int(msg.status)
        self.last_status_text = msg.status_text or 'OK'

        contract_error = validate_sector_contract(
            msg,
            self.min_distance_cm,
            self.max_distance_cm,
            self.increment_deg,
            self.angle_offset_deg,
        )
        ros_now_s = self.get_clock().now().nanoseconds / 1_000_000_000.0
        stamp_age_ms = max(0.0, (ros_now_s - message_stamp_seconds(msg)) * 1000.0)
        self.last_source_age_ms = max(stamp_age_ms, float(msg.source_age_ms))

        sequence = int(msg.source_sequence)
        if self.last_source_sequence is not None and sequence < self.last_source_sequence:
            contract_error = 'DEGRADED_SOURCE_SEQUENCE_BACKWARD'
        if self.last_source_age_ms > self.max_source_age_ms:
            contract_error = 'DEGRADED_SOURCE_STAMP_OLD'

        self.last_validation_error = contract_error
        if contract_error:
            self.stale_guard.mark_fault(contract_error)
            return

        upstream_hard_failure = self.last_upstream_status not in (
            UPSTREAM_OK,
            UPSTREAM_WARN_FILTERED_EMPTY,
            UPSTREAM_WARN_CLOUD_STALE,
        )
        if upstream_hard_failure:
            self.stale_guard.mark_fault('DEGRADED_UPSTREAM_STATUS')
            return

        self.last_distances = [
            max(0, min(UINT16_MAX, int(value)))
            for value in msg.distances
        ]
        self.stale_guard.mark_healthy_frame(sequence, now_monotonic)
        self.last_source_sequence = self.stale_guard.last_sequence
        self.last_source_progress_monotonic = self.stale_guard.last_progress

    def read_autopilot_heartbeat(self):
        if self.master is None:
            return
        try:
            while True:
                message = self.master.recv_match(type='HEARTBEAT', blocking=False)
                if message is None:
                    return
                if not heartbeat_matches(
                    message,
                    self.target_system,
                    self.target_component,
                    self.source_system,
                    self.source_component,
                ):
                    self.ignored_heartbeat_count += 1
                    continue
                system_id, component_id = heartbeat_source(message)
                self.locked_target_system = system_id
                self.locked_target_component = component_id
                self.last_autopilot_heartbeat_monotonic = time.monotonic()
        except Exception as exc:  # noqa: BLE001
            self.schedule_reconnect(f'读取飞控 heartbeat 失败: {exc}')

    def heartbeat_age_s(self, now: Optional[float] = None) -> Optional[float]:
        return age_seconds(
            self.last_autopilot_heartbeat_monotonic,
            time.monotonic() if now is None else now,
        )

    def heartbeat_is_fresh(self, now: Optional[float] = None) -> bool:
        if not self.wait_heartbeat_on_connect:
            return True
        age = self.heartbeat_age_s(now)
        return age is not None and age <= self.heartbeat_timeout_ms / 1000.0

    def current_distances_and_status(self) -> Tuple[Optional[List[int]], str, bool]:
        now_monotonic = time.monotonic()
        stale_status, hard_failure = self.stale_guard.evaluate(now_monotonic)
        if hard_failure:
            if not self.fail_closed:
                distances = self.last_distances
            elif self.startup_mode == 'unknown' and self.last_distances is None:
                distances = unknown_distances()
            elif self.startup_mode == 'hold' and self.last_distances is None:
                distances = None
            else:
                distances = virtual_wall_distances(self.virtual_wall_cm)
            return distances, stale_status, True

        if self.wait_heartbeat_on_connect and not self.heartbeat_is_fresh(now_monotonic):
            return (
                virtual_wall_distances(self.virtual_wall_cm),
                'DEGRADED_AUTOPILOT_HEARTBEAT',
                True,
            )
        if stale_status != 'OK':
            return self.last_distances, stale_status, True
        return self.last_distances, self.last_status_text or 'OK', False

    def send_heartbeat(self):
        self.connect_if_due()
        if self.master is None:
            return
        self.read_autopilot_heartbeat()
        if self.master is None:
            return
        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'HEARTBEAT 发送失败，进入退避重连: {exc}')
            self.schedule_reconnect(f'heartbeat send: {exc}')

    def send_obstacle_distance(self):
        self.connect_if_due()
        if self.master is None:
            return
        self.read_autopilot_heartbeat()
        if self.master is None:
            return

        distances, status_text, degraded = self.current_distances_and_status()
        if distances is None:
            return
        try:
            self.master.mav.obstacle_distance_send(
                int(time.time() * 1_000_000),
                self.sensor_type,
                distances,
                self.increment_deg,
                self.min_distance_cm,
                self.max_distance_cm,
                float(self.increment_deg),
                self.angle_offset_deg,
                self.frame,
            )
            self.last_send_error = ''
            previous_send = self.last_send_monotonic
            self.last_send_monotonic = time.monotonic()
            if previous_send is not None:
                self.send_intervals_ms.append(
                    (self.last_send_monotonic - previous_send) * 1000.0)
                self.send_intervals_ms = self.send_intervals_ms[-200:]
            if self.last_source_age_ms is not None:
                receive_age = age_seconds(
                    self.last_sector_receive_monotonic, self.last_send_monotonic) or 0.0
                self.last_source_to_mav_ms = (
                    self.last_source_age_ms + receive_age * 1000.0)
            self.send_count += 1
            if self.statustext_on_degraded and degraded:
                now_wall = time.time()
                if now_wall - self.last_status_publish > 2.0:
                    text = f'AeroHalo360 {status_text}'[:50]
                    self.master.mav.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_WARNING,
                        text.encode('ascii', errors='replace'),
                    )
                    self.last_status_publish = now_wall
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'OBSTACLE_DISTANCE 发送失败，进入退避重连: {exc}')
            self.schedule_reconnect(f'obstacle send: {exc}')

    def mavlink_state(self, now: Optional[float] = None) -> str:
        if self.master is None:
            return STATE_SEND_ERROR if self.last_send_error else STATE_DISCONNECTED
        if self.last_send_error:
            return STATE_SEND_ERROR
        if not self.wait_heartbeat_on_connect:
            return STATE_PORT_OPEN
        if not self.heartbeat_is_fresh(now):
            return STATE_HEARTBEAT_WAITING
        return STATE_AUTOPILOT_CONNECTED

    def publish_diagnostics(self):
        now_monotonic = time.monotonic()
        distances, sector_status, sector_degraded = self.current_distances_and_status()
        state = self.mavlink_state(now_monotonic)
        if state != self.last_reported_state:
            self.get_logger().info(
                f'MAVLink 状态变化: {self.last_reported_state or "START"} -> {state}')
            self.last_reported_state = state

        healthy_connection = (
            state == STATE_AUTOPILOT_CONNECTED or
            (not self.wait_heartbeat_on_connect and state == STATE_PORT_OPEN)
        )

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        item = DiagnosticStatus()
        item.name = 'aero_halo_360/mavlink_obstacle_sender'
        item.hardware_id = 'AeroHalo360'
        item.level = (
            DiagnosticStatus.OK
            if healthy_connection and not sector_degraded and self.last_send_monotonic is not None
            else DiagnosticStatus.WARN
        )
        item.message = f'{state}; {sector_status}'

        heartbeat_age = self.heartbeat_age_s(now_monotonic)
        send_age = age_seconds(self.last_send_monotonic, now_monotonic)
        receive_age = age_seconds(self.last_sector_receive_monotonic, now_monotonic)
        source_progress_age = age_seconds(
            self.last_source_progress_monotonic, now_monotonic)
        interval_recent, interval_max, interval_p95 = (
            self.stale_guard.interval_metrics())
        send_interval_ms = (
            self.send_intervals_ms[-1] if self.send_intervals_ms else 0.0)
        observed_send_hz = (
            1000.0 / send_interval_ms if send_interval_ms > 0.0 else 0.0)

        values = {
            'connection': self.connection,
            'profile_name': self.profile_name,
            'sector_topic': self.sector_topic,
            'mavlink_frame': str(self.frame),
            'configured_send_rate_hz': f'{self.send_rate_hz:.1f}',
            'observed_send_rate_hz': f'{observed_send_hz:.1f}',
            'safety_summary': (
                f'fail_closed={self.fail_closed}, wall_cm={self.virtual_wall_cm}, '
                f'warn_ms={self.input_warn_timeout_ms}, '
                f'fail_ms={self.input_fail_timeout_ms}'),
            'sector_publisher_count': str(self.count_publishers(self.sector_topic)),
            'connection_state': state,
            'source_system': str(self.source_system),
            'source_component': str(self.source_component),
            'target_system_config': str(self.target_system),
            'target_component_config': str(self.target_component),
            'target_system_locked': str(self.locked_target_system or 0),
            'target_component_locked': str(self.locked_target_component or 0),
            'heartbeat_required': str(self.wait_heartbeat_on_connect).lower(),
            'heartbeat_age_ms': (
                'unknown' if heartbeat_age is None else f'{heartbeat_age * 1000.0:.1f}'),
            'last_send_age_ms': (
                'unknown' if send_age is None else f'{send_age * 1000.0:.1f}'),
            'sector_receive_age_ms': (
                'unknown' if receive_age is None else f'{receive_age * 1000.0:.1f}'),
            'sector_source_progress_age_ms': (
                'unknown' if source_progress_age is None
                else f'{source_progress_age * 1000.0:.1f}'),
            'sector_source_age_ms': (
                'unknown' if self.last_source_age_ms is None
                else f'{self.last_source_age_ms:.1f}'),
            'sector_sequence': str(self.last_source_sequence or 0),
            'sector_upstream_status': str(self.last_upstream_status),
            'sector_status': sector_status,
            'sector_interval_recent_ms': f'{interval_recent:.1f}',
            'sector_interval_max_ms': f'{interval_max:.1f}',
            'sector_interval_p95_ms': f'{interval_p95:.1f}',
            'source_to_mavlink_ms': (
                'unknown' if self.last_source_to_mav_ms is None
                else f'{self.last_source_to_mav_ms:.1f}'),
            'stale_transition_count': str(self.stale_guard.transition_count),
            'stale_transition_reason': self.stale_guard.transition_reason,
            'stale_recovery_healthy_frames': str(self.stale_guard.healthy_frames),
            'sector_validation_error': self.last_validation_error,
            'has_sector_input': str(
                self.last_sector_receive_monotonic is not None).lower(),
            'sending_distances': str(distances is not None).lower(),
            'send_count': str(self.send_count),
            'send_error_count': str(self.send_error_count),
            'reconnect_count': str(self.reconnect_count),
            'ignored_heartbeat_count': str(self.ignored_heartbeat_count),
            'next_reconnect_ms': str(max(
                0,
                int((self.next_connect_monotonic - now_monotonic) * 1000.0),
            )),
            'fail_closed': str(self.fail_closed).lower(),
            'last_send_error': self.last_send_error,
        }
        item.values = [
            KeyValue(key=key, value=value)
            for key, value in values.items()
        ]
        array.status.append(item)
        self.diagnostics_pub.publish(array)

    def destroy_node(self):
        self.close_connection()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = MavlinkObstacleSender()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
