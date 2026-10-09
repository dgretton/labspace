# Arm follows your hand

A small end-to-end Nestbox demo: a Quest 3 tracks your hand, the headset's coordinate
system is aligned to the arm's from a few shared touch points, and then the RoArm follows
your pinch through `roarm/bridge.py`, @Sem07-mit's laptop-to-Pi bridge, used as-is: it opens an
SSH tunnel to the Pi and starts `roarm_server.py` there if it isn't already running.

```
Quest hands ─USB─▶ laptop: demo/follow_hand.py ─bridge.py SSH tunnel─▶ Pi: roarm_server.py ─serial─▶ arm
                   (align + follow)
```

## Run it

1. Headset: install the NestboxHand arm-demo build, plug in USB, then on the laptop
   `adb reverse tcp:12345 tcp:12345`. Start the app.
2. Check hands are arriving: `python demo/follow_hand.py --pipe --probe > /dev/null` (Ctrl-C when you see `righthand`).
3. Run through the bridge (needs key-based `ssh pi@<host>` to work without a password):
   ```
   cd demo
   python follow_hand.py --bridge pi@<pi-host>
   ```
   Alternative without the server: `python follow_hand.py --pipe | ssh <pi> 'python labspace/roarm/basicinput.py /dev/ttyUSB0'`.
   Via the server each step takes ~1.2 s (it waits for the move to finish), and the very first move ~10 s.
4. Calibrate: the arm visits 6 points. At each, touch the gripper tip with your right index
   fingertip, hold still, press Enter. The fit error is printed; it's saved to `data/hand_cal.json`
   (reuse with `--load-cal data/hand_cal.json`).
5. Follow: pinch thumb to index and move; the arm follows. Release to stop. Ctrl-C sends it home.

Needs `numpy`, `protobuf`, and `twig_pb2.py` (generated from nestbox's `protos/twig.proto`) next to the script.

## Safety

- The arm only moves while you pinch.
- Targets are clamped to `SAFE_*` limits at the top of `follow_hand.py`. These should match
  the safe ranges found by sweeping with `roarm/guessandcheck.py`.
- Each command moves at most 3 cm, since `T:1041` is a direct (fast) move.

## Nestbox aligner mode (`--nestbox`)

Uses the old nestbox package (repo `nestbox`, branch `armdemo`) instead of the standalone Kabsch fit:

```
./run_nestbox_demo.sh --arm-host <pi> [--load-pairs data/cal_pairs.json]
```

This starts Redis and a fresh nestbox daemon (TCP 8650, deterministic Gauss-Newton aligner), sets up adb reverse for
12345 (hands) and 8651 (overlay), and runs `follow_hand.py --nestbox`. Each calibration touch becomes a gripper-tip
feature seen in 'roarm' and in the pinned 'vr', and following uses nestbox's live estimate. `nestbox_link.py` streams
overlay geometry to the Quest app (`NestboxOverlay.cs`):
- a white box and axes at the estimated arm base;
- green dots at the touch points, and cyan dots at the arm points mapped through the estimate;
- yellow residual lines between them;
- a magenta dot at the follow target.

Raw pairs go to `data/cal_pairs.json`. Each headset has its own tracking origin, so recalibrate when you switch headsets.
Follow-mode moves are sent without waiting for the server's reply: up to 4 per second, 0.5–1.5 cm each.

## Handheld camera ray test

```
./run_camera_test.sh [--depth-sigma 0.5] [--pixel-sigma 2] [--fresh-noise] [--convergence-time 0.3]
```

Your right hand is a camera. Features are placed in the pinned VR world; features in view get long (ray-like)
Gaussian measurements; the nestbox Gauss-Newton aligner, running in-process, back-solves the camera.
- **Overlay:** green frustum = true camera, magenta = solved camera, translucent cyan = 1-sigma measurement ellipsoids
  placed through the solved pose, yellow = 2-sigma solved-position uncertainty.
- **Caveat:** each measurement's depth is currently simulated as true range + noise, which no monocular camera actually
  measures. See `docs/BEARING_MEASUREMENTS_PLAN.md` in the nestbox repo for the bearing-measurement replacement.

Both launchers default to the Python environment `~/.venvs/nestbox` (from nestbox's `python/requirements-armdemo.txt`
plus `pyserial`); override with `NESTBOX_PY`.
