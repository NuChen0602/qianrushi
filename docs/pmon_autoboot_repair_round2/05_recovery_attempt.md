# Recovery attempt

`nc -z -w 1 192.168.43.192 22` returned exit status 1. No SSH host key was fetched and no SSH session was attempted. The subsequent single dry serial run was silent and did not write.

Recovery is paused at the authorization boundary. The permitted command, only after a human on-site confirmation and only once, is:

```sh
sudo python3 /home/chen/Library_Patrol_Project/tools/pmon_recovery/board_recover_serial.py --send-cr --force-single-cr-on-silent
```

After the user runs it, Codex must only run `recover_board_and_wait_ssh.sh --wait-only --timeout 120`; it must never send another CR.
