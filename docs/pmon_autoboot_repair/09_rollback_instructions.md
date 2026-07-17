# Rollback

Only after an apply trial, run on the board: `tools/pmon_recovery/rollback_boot_cfg_trial.sh /boot/boot.cfg.before_showmenu0_TIMESTAMP`. The script requires that exact backup-path form, atomically restores the file, syncs, verifies its SHA-256 against the backup, and prints it. It does not reboot. No backup exists yet because no board-side change occurred.
