# Manual cold-start validation

1. Record `/boot/boot.cfg` and the backup path; run `sync`.
2. Manually shut down the board, unplug host USB, and leave CI302, HS-S77, wireless serial, and temperature/humidity wiring unchanged.
3. Power it from battery only. Do not send keys. Wait 120 seconds and check SSH.
4. On success record SSH reachability time, `/proc/uptime`, `/proc/cmdline`, `/proc/consoles`, and `boot.cfg`.
5. Repeat at least three times.
6. On failure reconnect USB, use the recovery script’s manually authorised one-CR operation, wait for SSH, then run the rollback script with the recorded backup path.

Codex must not perform the shutdown, USB operation, or power-on.
