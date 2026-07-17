# PMON automatic-start recovery — preparation result

Prepared in `/home/chen/Library_Patrol_Project`; no board connection, serial port, SSH session, or board-side file was touched during this preparation.

- Automatic serial recovery script: created, default read-only. Explicit `--send-cr` writes exactly one CR (`0x0D`) and nothing else.
- SSH: not attempted; actual `BOARD_IP` is therefore unknown. The candidate address remains unverified.
- Current board `boot.cfg`, kernel, cmdline, UART holders, and Wi-Fi state: unknown pending the supplied read-only audit.
- Exact vendor PMON source: not found in the local search. `not_delay` is therefore not confirmed in the board firmware.
- A reference-only behavioral patch and nine-case local model were generated; nothing was compiled or flashed.

No Wi-Fi, kernel, device tree, HS-S77, CI302, business code, PMON, or board files were modified. `/dev/ttyS0` and `/dev/ttyS1` were not accessed. No restart, power operation, or disconnect occurred.

The one required human action is: run `tools/pmon_recovery/recover_board_and_wait_ssh.sh`, review its dry-run serial log, and only if it indicates PMON manually run the printed single-CR command.
