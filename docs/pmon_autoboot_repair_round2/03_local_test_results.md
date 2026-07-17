# Local test results

Passed:

- `python3 -m py_compile` for serial recovery, reference state machine, and offline serial tests.
- `bash -n` for recovery, discovery, and read-only audit scripts.
- `sh -n` for reference-only apply and rollback scripts.
- `pmon_menu_state_test.py`: 9 reference state-machine cases passed.
- `test_serial_recovery_logic.py`: 8 classifications and 6 mocked write-eligibility cases passed without opening a serial device.

The restricted-token scan found no executable `rg`, `menuentry`, `eval`, kill, reboot, firmware-write, or destructive disk command. The only `/dev/ttyS0` and `/dev/ttyS1` use is `fuser -v` in the board read-only audit.
