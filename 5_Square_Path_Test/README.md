# Square Path Voice Test

Standalone Loongson 2K0301 test for a voice-triggered square path.

Default voice flow:

1. Say `你好小亚` to arm.
2. Say `小车前进` to start the square path.
3. Say `停止` during motion to stop immediately.

Extra calibration commands after `你好小亚`:

- `小车后退`: run one straight side only.
- `小车左转`: run one left 90-degree corner approximation.
- `小车右转`: run one right 90-degree corner approximation.

The chassis uses front steering, so each corner is implemented as:

```text
servo right -> drive forward by turn_counts_90 -> servo center
```

Build:

```bash
./build.sh
```

Board run:

```bash
/etc/init.d/S99_voice_motion_test stop
/home/root/square_path_voice_test run --config /home/root/square_path_config.ini
```

Tune:

- `side_counts`: length of each square side.
- `turn_counts_90`: approximate 90-degree corner amount.
- `straight_pulse_on_ms` / `straight_pulse_off_ms`: straight motion power and pacing.
- `turn_pulse_on_ms` / `turn_pulse_off_ms`: turning motion power and pacing.
