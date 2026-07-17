# Next safe action

The only next action is for an on-site user to decide whether the known silent-PMON condition is confirmed. If confirmed, manually run once:

```sh
sudo python3 /home/chen/Library_Patrol_Project/tools/pmon_recovery/board_recover_serial.py --send-cr --force-single-cr-on-silent
```

Then report completion so the wait-only, no-serial SSH check can proceed. Do not run apply/rollback scripts, reboot, or change `boot.cfg`.
