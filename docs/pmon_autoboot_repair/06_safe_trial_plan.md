# Safe boot.cfg trial plan

Do not run the apply script until the audit records the current file. If it already says `timeout 3`, `default 0`, and `showmenu 0`, make no change and proceed only to manual cold-start validation. If, and only if, it says `showmenu 1` while satisfying every guarded condition, copy `apply_boot_cfg_trial.sh` to the board and run it manually. It backs up, hashes, atomically replaces, and syncs. Any other format is a stop condition. It neither restarts nor changes a kernel, UART setting, Wi-Fi, device tree, or maintenance entries.
