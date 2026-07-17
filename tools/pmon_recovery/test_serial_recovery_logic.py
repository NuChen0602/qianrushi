#!/usr/bin/env python3
"""Offline tests only: no serial device is opened."""
import board_recover_serial as serial

classification = [
    (b'', serial.SERIAL_STATE_SILENT),
    (b'PMON>', serial.SERIAL_STATE_PMON),
    (b'seekfree boot menu', serial.SERIAL_STATE_PMON),
    (b'Now booting...', serial.SERIAL_STATE_LINUX_BOOTING),
    (b'Loading file...', serial.SERIAL_STATE_LINUX_BOOTING),
    (b'\x01\xfe\x02', serial.SERIAL_STATE_UNKNOWN),
    (b'SeEkFrEe BOOT MENU', serial.SERIAL_STATE_PMON),
    (b'PMON> Now booting...', serial.SERIAL_STATE_LINUX_BOOTING),
]
for payload, expected in classification:
    assert serial.classify_serial(payload) == expected, (payload, serial.classify_serial(payload))

# Mocked write eligibility: actual os.open/os.write are never called in this module.
assert not serial.should_write_cr(serial.SERIAL_STATE_PMON, False, False)
assert serial.should_write_cr(serial.SERIAL_STATE_PMON, True, False)
assert not serial.should_write_cr(serial.SERIAL_STATE_SILENT, True, False)
assert serial.should_write_cr(serial.SERIAL_STATE_SILENT, True, True)
assert not serial.should_write_cr(serial.SERIAL_STATE_UNKNOWN, True, False)
assert not serial.should_write_cr(serial.SERIAL_STATE_LINUX_BOOTING, True, False)
print('8 classification tests and 6 mocked write-eligibility tests passed')
