#!/usr/bin/env python3
"""Read PMON serial output.  Only explicit, eligible --send-cr writes one 0x0D."""
import argparse
import datetime
import os
import pathlib
import select
import termios
import time

PMON_MARKERS = (b"pmon", b"boot menu", b"seekfree", b"boot.cfg", b"timeout")
LINUX_MARKERS = (b"now booting", b"loading file", b"boot with parameters", b"linux version", b"starting kernel")
SERIAL_STATE_PMON = "SERIAL_STATE_PMON"
SERIAL_STATE_LINUX_BOOTING = "SERIAL_STATE_LINUX_BOOTING"
SERIAL_STATE_SILENT = "SERIAL_STATE_SILENT"
SERIAL_STATE_UNKNOWN = "SERIAL_STATE_UNKNOWN"

def classify_serial(data):
    """Linux markers win: they demonstrate the later boot stage in mixed captures."""
    lower = data.lower()
    if not lower:
        return SERIAL_STATE_SILENT
    if any(marker in lower for marker in LINUX_MARKERS):
        return SERIAL_STATE_LINUX_BOOTING
    if any(marker in lower for marker in PMON_MARKERS):
        return SERIAL_STATE_PMON
    return SERIAL_STATE_UNKNOWN

def marker_names(data, markers):
    lower = data.lower()
    return [marker.decode("ascii") for marker in markers if marker in lower]

def safe_display(data):
    printable = ''.join(chr(byte) if 32 <= byte <= 126 else '.' for byte in data)
    return "ascii=%r hex=%s" % (printable, data.hex(' '))

def should_write_cr(state, send_cr, force_single_cr_on_silent):
    return send_cr and (state == SERIAL_STATE_PMON or
                        (state == SERIAL_STATE_SILENT and force_single_cr_on_silent))

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/ttyACM0')
    parser.add_argument('--dry-run', action='store_true', help='read only (also the default)')
    parser.add_argument('--send-cr', action='store_true', help='allow at most one CR (0x0D), only for PMON')
    parser.add_argument('--force-single-cr-on-silent', action='store_true',
                        help='ONLY after on-site confirmation that a silent board is stopped in PMON')
    args = parser.parse_args()
    if args.dry_run and args.send_cr:
        parser.error('--dry-run and --send-cr are mutually exclusive')
    if args.force_single_cr_on_silent and not args.send_cr:
        parser.error('--force-single-cr-on-silent requires --send-cr')

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    root = pathlib.Path(__file__).resolve().parents[2]
    log_path = root / 'docs' / 'pmon_autoboot_repair_round2' / ('serial_recovery_' + stamp + '.log')
    log_path.parent.mkdir(parents=True, exist_ok=True)
    fd = None
    old = None
    try:
        flags = (os.O_RDWR if args.send_cr else os.O_RDONLY) | os.O_NOCTTY | os.O_NONBLOCK
        fd = os.open(args.device, flags)
        old = termios.tcgetattr(fd)
        attrs = termios.tcgetattr(fd)
        attrs[0] = termios.IGNPAR
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = termios.B115200
        attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        captured = bytearray()
        with log_path.open('w', encoding='utf-8') as log:
            log.write('device=%s mode=%s; 115200 8N1; software/hardware flow control disabled; DTR/RTS untouched\n' %
                      (args.device, 'send-cr' if args.send_cr else 'dry-run'))
            def read_for(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    ready, _, _ = select.select([fd], [], [], min(.25, end - time.monotonic()))
                    if ready:
                        try:
                            chunk = os.read(fd, 4096)
                        except BlockingIOError:
                            continue
                        if chunk:
                            captured.extend(chunk)
                            log.write('%s %s\n' % (datetime.datetime.now().isoformat(timespec='milliseconds'), safe_display(chunk)))
                            log.flush()
            read_for(5)
            before = bytes(captured)
            state = classify_serial(before)
            allow_write = should_write_cr(state, args.send_cr, args.force_single_cr_on_silent)
            write_count = 0
            log.write('state=%s bytes_before_write=%d pmon_markers=%s linux_markers=%s\n' %
                      (state, len(before), marker_names(before, PMON_MARKERS), marker_names(before, LINUX_MARKERS)))
            if allow_write:
                write_count = os.write(fd, b'\r')
                if write_count != 1:
                    raise OSError('CR write count was %d, expected 1' % write_count)
            log.write('whether_cr_written=%s exact_write_count=%d\n' % (write_count == 1, write_count))
            log.flush()
            read_for(30)
            after = bytes(captured)
            log.write('markers_after_write=%s final_state=%s\n' %
                      (marker_names(after, PMON_MARKERS + LINUX_MARKERS), classify_serial(after)))
        print(log_path)
    except OSError as exc:
        print('serial open/read failure: %s' % exc)
        print('If permission is required, manually run one of:')
        print('sudo python3 %s --send-cr' % pathlib.Path(__file__).resolve())
        print('sudo python3 %s --send-cr --force-single-cr-on-silent' % pathlib.Path(__file__).resolve())
        return 1
    finally:
        if fd is not None:
            if old is not None:
                try:
                    termios.tcsetattr(fd, termios.TCSANOW, old)
                except termios.error:
                    pass
            os.close(fd)
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
