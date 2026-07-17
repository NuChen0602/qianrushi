# Tool defects found

- Recovery depended on `rg`, used unsafe `eval`, hid discovery diagnostics through stderr redirection, did not implement the promised wait, and could not forward/parse options safely.
- Serial recovery classified absence of Linux markers as a write trigger, so silence could have caused a CR without explicit on-site authorization.
- Discovery treated `--accept-hostkey` as approval and compared whole key material rather than normalized algorithm/fingerprint pairs.
- Audit did not set `ConnectTimeout`, mixed full `dmesg` into the report, and could stop on unavailable BusyBox commands.
- The boot trial parser expected `menuentry` rather than the actual `title`-block format and was not POSIX `sh` compatible.
