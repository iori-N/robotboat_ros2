#!/usr/bin/env python3
"""Detect u-blox roles and update ublox_gnss launch ports."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass
import glob
import os
from pathlib import Path
import re
import select
import subprocess
import sys
import termios
import time


DEFAULT_LAUNCH_FILE = (
    Path(__file__).resolve().parents[1]
    / 'src/ublox_gnss_driver/launch/ublox_gnss.launch.py'
)
DEFAULT_SERVICE = 'ros2-ublox'
DEFAULT_BAUD = 115200
UBX_SYNC = b'\xb5\x62'
HPPOSLLH = (0x01, 0x14)
RELPOSNED = (0x01, 0x3C)
TARGETS = {
    RELPOSNED: 'NAV-RELPOSNED',
    HPPOSLLH: 'NAV-HPPOSLLH',
}
BAUDS = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
    921600: termios.B921600,
}


@dataclass(frozen=True)
class Detection:
    """Detected message counts for one serial device."""

    target: str
    by_path: str
    bytes_read: int
    hpposllh: int
    relposned: int


def ubx_checksum(frame_body: bytes) -> tuple[int, int]:
    """Compute a UBX checksum."""
    ck_a = 0
    ck_b = 0
    for value in frame_body:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def parse_ubx(buffer: bytes) -> tuple[list[tuple[int, int]], bytes]:
    """Parse complete UBX frames from a byte buffer."""
    messages = []
    index = 0

    while True:
        sync = buffer.find(UBX_SYNC, index)
        if sync < 0:
            tail = buffer[-1:] if buffer.endswith(UBX_SYNC[:1]) else b''
            return messages, tail

        if len(buffer) - sync < 8:
            return messages, buffer[sync:]

        msg_class = buffer[sync + 2]
        msg_id = buffer[sync + 3]
        length = buffer[sync + 4] | (buffer[sync + 5] << 8)
        frame_len = 8 + length

        if len(buffer) - sync < frame_len:
            return messages, buffer[sync:]

        frame_body = buffer[sync + 2:sync + 6 + length]
        ck_a = buffer[sync + 6 + length]
        ck_b = buffer[sync + 7 + length]

        if ubx_checksum(frame_body) == (ck_a, ck_b):
            messages.append((msg_class, msg_id))
            index = sync + frame_len
        else:
            index = sync + 2


def configure_port(fd: int, baud: int) -> None:
    """Configure a serial file descriptor for raw UBX reads."""
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = attrs[2] | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = BAUDS[baud]
    attrs[5] = BAUDS[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def sniff(path: str, seconds: float, baud: int) -> tuple[int, Counter]:
    """Read one serial device and count UBX message types."""
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_port(fd, baud)
        end_time = time.monotonic() + seconds
        buffer = b''
        counts = Counter()
        bytes_read = 0

        while time.monotonic() < end_time:
            timeout = max(0.0, min(0.2, end_time - time.monotonic()))
            readable, _, _ = select.select([fd], [], [], timeout)
            if not readable:
                continue

            chunk = os.read(fd, 4096)
            if not chunk:
                continue

            bytes_read += len(chunk)
            parsed, buffer = parse_ubx(buffer + chunk)
            counts.update(parsed)

        return bytes_read, counts
    finally:
        os.close(fd)


def collect_by_path() -> dict[str, list[str]]:
    """Group /dev/serial/by-path symlinks by real device."""
    grouped: dict[str, list[str]] = defaultdict(list)
    for link in sorted(glob.glob('/dev/serial/by-path/*')):
        target = os.path.realpath(link)
        grouped[target].append(link)
    return grouped


def run_systemctl(args: list[str]) -> subprocess.CompletedProcess[str]:
    """Run systemctl and capture output."""
    return subprocess.run(
        ['systemctl', *args],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def is_service_active(service: str) -> bool:
    """Return True when a systemd service is active."""
    result = run_systemctl(['is-active', '--quiet', service])
    return result.returncode == 0


def stop_service(service: str) -> bool:
    """Stop a systemd service if possible."""
    result = run_systemctl(['stop', service])
    if result.returncode == 0:
        print(f'Stopped systemd service: {service}')
        return True
    print(
        f'Failed to stop {service}. Run this script with sudo or stop it manually.',
        file=sys.stderr,
    )
    if result.stderr.strip():
        print(result.stderr.strip(), file=sys.stderr)
    return False


def start_service(service: str) -> bool:
    """Start a systemd service if possible."""
    result = run_systemctl(['start', service])
    if result.returncode == 0:
        print(f'Started systemd service: {service}')
        return True
    print(f'Failed to start {service}.', file=sys.stderr)
    if result.stderr.strip():
        print(result.stderr.strip(), file=sys.stderr)
    return False


def detect_devices(seconds: float, baud: int) -> list[Detection]:
    """Sniff all /dev/serial/by-path devices and return detected counts."""
    grouped = collect_by_path()
    if not grouped:
        raise RuntimeError('No devices found under /dev/serial/by-path')

    detections = []
    for target, links in sorted(grouped.items()):
        by_path = sorted(links)[0]
        try:
            bytes_read, counts = sniff(target, seconds, baud)
        except PermissionError as exc:
            raise RuntimeError(
                f'Permission denied while reading {target}. '
                'Add the user to dialout or run with sudo.'
            ) from exc
        except OSError as exc:
            print(f'Skipping {target}: {exc}', file=sys.stderr)
            continue

        detections.append(Detection(
            target=target,
            by_path=by_path,
            bytes_read=bytes_read,
            hpposllh=counts[HPPOSLLH],
            relposned=counts[RELPOSNED],
        ))

    return detections


def select_unique(
    matches: list[Detection],
    count_attr: str,
    role: str,
) -> Detection:
    """Select exactly one device for a role."""
    if len(matches) != 1:
        detail = ', '.join(
            f'{item.by_path}={getattr(item, count_attr)}' for item in matches
        ) or 'none'
        raise RuntimeError(
            f'Expected exactly one {role} device, found {len(matches)} ({detail})'
        )
    return matches[0]


def select_ntrip(detections: list[Detection], min_count: int) -> Detection:
    """Select the RTK rover receiver from devices publishing HPPOSLLH."""
    matches = [item for item in detections if item.hpposllh >= min_count]
    return select_unique(
        matches, 'hpposllh', 'HPPOSLLH/NTRIP receiver')


def select_moving_base(detections: list[Detection], min_count: int) -> Detection:
    """Select the moving-base receiver from RELPOSNED-only devices."""
    matches = [
        item for item in detections
        if item.relposned >= min_count and item.hpposllh < min_count
    ]
    return select_unique(
        matches, 'relposned', 'RELPOSNED-only moving-base receiver')


def replace_launch_default(text: str, arg_name: str, value: str) -> tuple[str, str]:
    """Replace one DeclareLaunchArgument default_value and return old value."""
    pattern = re.compile(
        rf"(DeclareLaunchArgument\(\s*'{re.escape(arg_name)}',"
        rf".*?default_value=)'([^']*)'",
        re.DOTALL,
    )
    match = pattern.search(text)
    if match is None:
        raise RuntimeError(f'Launch argument not found: {arg_name}')
    old_value = match.group(2)
    return text[:match.start(2)] + value + text[match.end(2):], old_value


def update_launch_file(
    launch_file: Path,
    ntrip_port: str,
    moving_base_port: str,
    baud: int,
    write: bool,
    backup: bool,
) -> bool:
    """Show or apply launch file changes."""
    original = launch_file.read_text()
    updated = original
    changes = []

    replacements = {
        'ntrip_port': ntrip_port,
        'ntrip_baudrate': str(baud),
        'moving_base_port': moving_base_port,
        'moving_base_baudrate': str(baud),
    }

    for arg_name, new_value in replacements.items():
        updated, old_value = replace_launch_default(updated, arg_name, new_value)
        changes.append((arg_name, old_value, new_value))

    print('Launch updates:')
    changed = False
    for arg_name, old_value, new_value in changes:
        marker = 'update' if old_value != new_value else 'keep'
        print(f'  {marker}: {arg_name}: {old_value} -> {new_value}')
        changed = changed or old_value != new_value

    if not changed:
        print('No launch file changes needed.')
        return False

    if not write:
        print('Dry run only. Re-run with --write to update the launch file.')
        return False

    if backup:
        backup_file = launch_file.with_suffix(launch_file.suffix + '.bak')
        backup_file.write_text(original)
        print(f'Backup written: {backup_file}')

    launch_file.write_text(updated)
    print(f'Updated launch file: {launch_file}')
    return True


def print_detections(detections: list[Detection]) -> None:
    """Print detection summary."""
    print('Detected devices:')
    for item in detections:
        print(f'  {item.by_path}')
        print(f'    target: {item.target}')
        print(f'    bytes: {item.bytes_read}')
        print(f'    {TARGETS[HPPOSLLH]}: {item.hpposllh}')
        print(f'    {TARGETS[RELPOSNED]}: {item.relposned}')


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Detect HPPOSLLH/RELPOSNED senders and update ublox launch ports.'
    )
    parser.add_argument('--seconds', type=float, default=6.0)
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD, choices=sorted(BAUDS))
    parser.add_argument('--min-count', type=int, default=1)
    parser.add_argument('--launch-file', type=Path, default=DEFAULT_LAUNCH_FILE)
    parser.add_argument('--service', default=DEFAULT_SERVICE)
    parser.add_argument('--no-stop-service', action='store_true')
    parser.add_argument('--no-restart-service', action='store_true')
    parser.add_argument('--no-backup', action='store_true')
    parser.add_argument('--write', action='store_true')
    return parser.parse_args()


def main() -> int:
    """Run detection and optionally update the launch file."""
    args = parse_args()

    if args.seconds <= 0:
        print('--seconds must be positive', file=sys.stderr)
        return 2
    if args.min_count <= 0:
        print('--min-count must be positive', file=sys.stderr)
        return 2

    launch_file = args.launch_file.expanduser().resolve()
    if not launch_file.exists():
        print(f'Launch file not found: {launch_file}', file=sys.stderr)
        return 2

    service_was_active = False
    service_stopped = False
    try:
        if not args.no_stop_service and args.service:
            service_was_active = is_service_active(args.service)
            if service_was_active:
                if not stop_service(args.service):
                    return 1
                service_stopped = True
                time.sleep(1.0)

        detections = detect_devices(args.seconds, args.baud)
        print_detections(detections)

        ntrip = select_ntrip(detections, args.min_count)
        moving_base = select_moving_base(detections, args.min_count)

        if ntrip.by_path == moving_base.by_path:
            print(
                'HPPOSLLH and RELPOSNED were detected on the same device; '
                'refusing to update ports.',
                file=sys.stderr,
            )
            return 1

        print('Selected roles:')
        print(f'  ntrip_port: {ntrip.by_path}')
        print(f'  moving_base_port: {moving_base.by_path}')

        update_launch_file(
            launch_file=launch_file,
            ntrip_port=ntrip.by_path,
            moving_base_port=moving_base.by_path,
            baud=args.baud,
            write=args.write,
            backup=not args.no_backup,
        )
        return 0
    except RuntimeError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1
    finally:
        if service_stopped and service_was_active and not args.no_restart_service:
            start_service(args.service)


if __name__ == '__main__':
    raise SystemExit(main())
