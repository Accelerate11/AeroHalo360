#!/usr/bin/env python3
import argparse
import json
import os
import pathlib
import statistics
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node


def percentile(values, quantile):
    if not values:
        return None
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * quantile))
    return ordered[index]


def find_process_pid(process_name):
    for entry in pathlib.Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if (entry / 'comm').read_text(encoding='utf-8').strip() == process_name:
                return int(entry.name)
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return None


def process_sample(pid):
    if pid is None:
        return None
    try:
        stat = pathlib.Path(f'/proc/{pid}/stat').read_text(encoding='utf-8').split()
        statm = pathlib.Path(f'/proc/{pid}/statm').read_text(encoding='utf-8').split()
        cpu_ticks = int(stat[13]) + int(stat[14])
        rss_bytes = int(statm[1]) * os.sysconf('SC_PAGE_SIZE')
        return cpu_ticks, rss_bytes
    except (FileNotFoundError, PermissionError, ProcessLookupError, IndexError, ValueError):
        return None


class BenchmarkCollector(Node):
    def __init__(self):
        super().__init__('aero_halo_360_benchmark')
        self.samples = {
            'processing_latency_ms': [],
            'source_to_publish_ms': [],
            'source_to_mavlink_ms': [],
            'processing_rate_hz': [],
            'cloud_interval_ms': [],
            'sector_interval_ms': [],
        }
        self.create_subscription(
            DiagnosticArray,
            '/aero_halo_360/diagnostics',
            self.on_diagnostics,
            20,
        )
        self.create_subscription(
            DiagnosticArray,
            '/aero_halo_360/mavlink_diagnostics',
            self.on_diagnostics,
            20,
        )

    def on_diagnostics(self, message):
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            mapping = {
                'processing_latency_ms': 'processing_latency_ms',
                'source_to_publish_ms': 'source_to_publish_ms',
                'source_to_mavlink_ms': 'source_to_mavlink_ms',
                'processing_rate_hz': 'processing_rate_hz',
                'cloud_interval_recent_ms': 'cloud_interval_ms',
                'sector_interval_recent_ms': 'sector_interval_ms',
            }
            for key, output_key in mapping.items():
                try:
                    value = float(values[key])
                except (KeyError, ValueError):
                    continue
                if value >= 0.0:
                    self.samples[output_key].append(value)


def summarize(values):
    return {
        'count': len(values),
        'p50': percentile(values, 0.50),
        'p95': percentile(values, 0.95),
        'p99': percentile(values, 0.99),
        'max': max(values) if values else None,
        'mean': statistics.fmean(values) if values else None,
    }


def main():
    parser = argparse.ArgumentParser(
        description='采集 AeroHalo360 ROS diagnostics 的时延、频率、CPU 和内存基准')
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--process-name', default='cloud_processor')
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error('--duration 必须大于 0')

    pid = find_process_pid(args.process_name)
    start_process = process_sample(pid)
    start_monotonic = time.monotonic()

    rclpy.init()
    node = BenchmarkCollector()
    try:
        deadline = start_monotonic + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    elapsed = max(0.001, time.monotonic() - start_monotonic)
    end_process = process_sample(pid)
    cpu_percent = None
    rss_mib = None
    if start_process is not None and end_process is not None:
        tick_delta = max(0, end_process[0] - start_process[0])
        cpu_percent = tick_delta / os.sysconf('SC_CLK_TCK') / elapsed * 100.0
        rss_mib = end_process[1] / (1024.0 * 1024.0)

    report = {
        'schema': 'aero_halo_360_benchmark_v1',
        'scenario': args.scenario,
        'duration_s': elapsed,
        'process_name': args.process_name,
        'process_pid': pid,
        'cpu_percent_one_core': cpu_percent,
        'rss_mib_end': rss_mib,
        'metrics': {
            key: summarize(values)
            for key, values in node.samples.items()
        },
    }
    output = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        pathlib.Path(args.output).write_text(output + '\n', encoding='utf-8')
    print(output)

    required = report['metrics']['processing_latency_ms']['count']
    return 0 if required > 0 else 2


if __name__ == '__main__':
    raise SystemExit(main())
