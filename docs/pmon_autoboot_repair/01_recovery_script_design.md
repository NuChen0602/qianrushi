# Recovery design

`board_recover_serial.py` configures `/dev/ttyACM0` at 115200 8N1 without flow control, uses `O_NOCTTY`, never changes DTR/RTS, and does not issue reset ioctls. It restores the prior terminal attributes before closing. It passively logs five seconds of timestamped printable-safe and hexadecimal traffic. Default is read-only and opens the port read-only. Only `--send-cr` can write, only if PMON/boot/menu/seekfree is seen or Linux markers are absent, and then writes exactly one byte `0x0d`; it reads up to 30 more seconds and closes the device.

`recover_board_and_wait_ssh.sh` first looks for an approved SSH target. Otherwise it performs only the dry run and prints the manual `sudo` command. It never supplies or stores a password. IP discovery probes TCP/22 only, limits scan concurrency to 16, displays SSH host-key fingerprints, and emits `BOARD_IP` only for a matching known key or the user’s explicit `--accept-hostkey` invocation.
