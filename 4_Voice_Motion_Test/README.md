# Voice Motion Test

This is a standalone Loongson 2K0301 user-space test for voice-controlled short motion.
It does not depend on ROS, Python, OpenCV, network, or the PC after deployment.

## Build

```bash
./build.sh
```

The output is:

```bash
build/voice_motion_test
```

## Board Files

Copy these files to the board:

```text
/home/root/voice_motion_test
/home/root/voice_motion_config.ini
```

Optional boot script:

```text
/etc/init.d/S99_voice_motion_test
```

## First Test

Keep the wheels lifted.

```bash
./voice_motion_test probe --port /dev/ttyS1 --baud 115200
```

Speak the voice commands and confirm that frames are printed. If nothing is printed,
retry with `--baud 9600`.

## Learn Commands

```bash
./voice_motion_test learn --config /home/root/voice_motion_config.ini --port /dev/ttyS1 --baud 115200
```

It learns these commands in order:

```text
start, stop, forward, back, left, right
```

## Run

```bash
./voice_motion_test run --config /home/root/voice_motion_config.ini
```

Safety behavior:

- It starts locked.
- Say the learned start command to unlock.
- Say one motion command.
- After one short motion it locks again.
- Say the learned stop command at any time to stop and lock.

## PC/ROS Agent Mode

For the real-robot ROS bridge, run the board program as a UDP agent:

```bash
./voice_motion_test agent --config /home/root/voice_motion_config.ini --port 15000
```

The PC sends simple text commands:

```text
PING
STATUS
STOP
MOVE forward 30
MOVE back 30
TURN left 16
TURN right 16
SERVO center
```

The agent keeps the same DIR/PWM-swapped wiring adaptation used by voice mode.

To boot into PC/ROS agent mode instead of standalone voice mode:

```bash
echo agent > /home/root/voice_motion_mode
/etc/init.d/S99_voice_motion_test restart
```

To return to standalone voice mode:

```bash
echo run > /home/root/voice_motion_mode
/etc/init.d/S99_voice_motion_test restart
```

## Install Autostart

Only install this after `probe`, `learn`, and manual `run` all work.

```bash
cp /home/root/S99_voice_motion_test /etc/init.d/S99_voice_motion_test
chmod +x /etc/init.d/S99_voice_motion_test
/etc/init.d/S99_voice_motion_test start
```
