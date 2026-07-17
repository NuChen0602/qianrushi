# PMON recovery tools

All tools are conservative. `board_recover_serial.py` defaults to read-only; only an explicit `--send-cr` in PMON state writes exactly one `0x0D` to `/dev/ttyACM0`. A silent port additionally requires explicit `--force-single-cr-on-silent` and on-site confirmation. It never touches `/dev/ttyS0` or `/dev/ttyS1`.

1. Run `./recover_board_and_wait_ssh.sh`.
2. If it reports PMON, review the serial log and manually run the displayed `sudo ... --send-cr` command once.
3. Confirm the shown host-key fingerprint, then use `./discover_board_ip.sh --accept-hostkey` and record it in `known_hosts` through your normal approved process. Do not remove existing entries.
4. Run `./board_readonly_audit.sh BOARD_IP > audit_TIMESTAMP.log`.

The apply/rollback scripts are `REFERENCE_ONLY_NOT_EXECUTED` for round 2. They must not be copied to or run on the board. The current `showmenu` value must be established by the read-only audit first.
