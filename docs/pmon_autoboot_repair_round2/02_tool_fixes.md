# Tool fixes

`board_recover_serial.py` now has explicit PMON, Linux-booting, silent, and unknown states. Only PMON permits `--send-cr`; silence additionally requires explicit `--force-single-cr-on-silent`. The write is exactly `b"\r"`, checked for return count one, and terminal attributes are restored.

`recover_board_and_wait_ssh.sh` now uses `grep`, safe line parsing, known-IP polling once per second, `--wait-only`, `--known-ip`, and `--timeout`. It never invokes sudo, sends serial data, accepts a host key, or uses `eval`.

Discovery scans only TCP/22 in the target subnet with at most 16 workers, deduplicates results, displays normalized key fingerprints, and outputs `BOARD_IP` plus `HOSTKEY_APPROVED`. `--accept-hostkey` remains display-only. Audit now uses strict SSH options, saves full dmesg locally, and continues on unavailable board commands. Apply and rollback are POSIX-`sh` reference-only scripts for this round.
