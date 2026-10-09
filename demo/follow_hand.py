"""
Arm follows your hand: Quest hand tracking -> Nestbox-style alignment -> RoArm.

The headset (NestboxHand app) streams hand Twigs over TCP. This script:
  1. listens for those Twigs,
  2. CALIBRATES: moves the arm to a few safe points; at each one you touch the
     gripper tip with your index fingertip and press Enter. A rigid VR->arm
     transform is fit to those pairs (Kabsch), i.e. the two coordinate systems
     get aligned from shared observations of the same feature,
  3. FOLLOWS: while you pinch (thumb to index), the arm moves to your fingertip,
     clamped to the arm's known-safe workspace.

Arm control goes through roarm/bridge.py (SSH tunnel to roarm_server.py on the Pi), or straight over serial with the {"T":1041,...} command from
roarm/basicinput.py, or through roarm/roarm_server.py over TCP.

Examples:
  python follow_hand.py --bridge pi@sdna-hdb            # via roarm/bridge.py (recommended)
  python follow_hand.py --pipe | ssh <pi> 'python labspace/roarm/basicinput.py /dev/ttyUSB0'
  python follow_hand.py --serial /dev/ttyUSB0            # run on the Pi
  python follow_hand.py --arm-host 10.1.20.227           # run on a laptop, Pi runs roarm_server.py
  python follow_hand.py --arm-host 10.1.20.227 --load-cal data/hand_cal.json
"""
import argparse
import json
import os
import socket
import sys
import threading
import time

import numpy as np

import twig_pb2

# ---- Hand point indices. NestboxHand's SendHandTwigs.cs sends OVRSkeleton.Bones in order. ----
# Bone count -> (thumb tip, index tip): legacy OVR hand skeleton has 24 bones, OpenXR hand skeleton has 26.
TIPS_BY_BONE_COUNT = {24: (19, 20), 26: (5, 10)}
INDEX_TIP = None  # None = choose from bone count of the first frame received
THUMB_TIP = None
PINCH_DISTANCE = 0.025  # meters

# ---- Safe workspace, in arm coordinates (meters, x forward, z up). ----
# Edit these to match the safe ranges found with roarm/guessandcheck.py sweeps.
SAFE_RADIUS = (0.20, 0.45)
SAFE_X = (0.10, 0.45)
SAFE_Y = (-0.25, 0.25)
SAFE_Z = (0.03, 0.40)
HOME = np.array([0.32, 0.0, 0.23])

# Calibration targets, spread out in all three axes, all inside the safe workspace.
# First point is deliberately far from HOME so the arm visibly moves. The four marked
# "reached" fit to ~2 cm RMS in the first lab run (2026-10-03); the other two are new.
CAL_POINTS = np.array([
    [0.35, -0.10, 0.30],  # reached
    [0.25, -0.15, 0.15],  # reached
    [0.40, 0.00, 0.10],   # reached
    [0.35, 0.10, 0.30],   # reached
    [0.28, 0.12, 0.18],   # replaces [0.25, 0.15, 0.15], which the arm didn't visibly reach
    [0.22, 0.00, 0.28],   # replaces HOME, where the arm already was
])


def clamp_to_safe(p):
    p = np.array(p, dtype=float)
    lo, hi = SAFE_RADIUS
    r = np.linalg.norm(p)
    if r < lo:
        p *= lo / max(r, 1e-6)
    elif r > hi:
        p *= hi / r
    # box last, so the box limits (e.g. the table) are never violated
    p[0] = np.clip(p[0], *SAFE_X)
    p[1] = np.clip(p[1], *SAFE_Y)
    p[2] = np.clip(p[2], *SAFE_Z)
    return p


class HandReceiver:
    """Accepts the headset's length-prefixed Twig stream and keeps the latest points per hand."""

    def __init__(self, port):
        self.port = port
        self.latest = {}  # stream_id -> (timestamp, Nx3 array)
        self.lock = threading.Lock()

    def start(self):
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', self.port))
        srv.listen(1)
        print(f"Listening for headset on port {self.port}")
        while True:
            conn, addr = srv.accept()
            print(f"Headset connected from {addr[0]}")
            threading.Thread(target=self._handle, args=(conn,), daemon=True).start()

    def _recv_exact(self, conn, n):
        data = b''
        while len(data) < n:
            chunk = conn.recv(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def _handle(self, conn):
        with conn:
            while True:
                header = self._recv_exact(conn, 4)
                if header is None:
                    print("Headset disconnected")
                    return
                body = self._recv_exact(conn, int.from_bytes(header, 'big'))
                if body is None:
                    return
                twig = twig_pb2.Twig()
                twig.ParseFromString(body)
                if not twig.measurements:
                    continue
                points = np.array([list(s.mean)[:3] for s in twig.measurements[0].samples])
                if points.size == 0 or np.all(np.abs(points) < 1e-6):
                    continue  # hand not tracked
                with self.lock:
                    self.latest[twig.streamId] = (time.time(), points)

    def get(self, hand, max_age=0.3):
        with self.lock:
            entry = self.latest.get(hand)
        if entry is None or time.time() - entry[0] > max_age:
            return None
        return entry[1]


class PipeArm:
    """Drives roarm/basicinput.py, unmodified, by answering its prompts on its stdin.

    basicinput.py asks "Enter x,y,z in mm:" then "Send this? (y/n):", so each move is two lines.
    Usage: python follow_hand.py --pipe | ssh <pi> 'python labspace/roarm/basicinput.py /dev/ttyUSB0'
    """

    def __init__(self, out):
        self.out = out

    def move(self, p, settle=0.0):
        x, y, z = (np.asarray(p) * 1000).tolist()
        self.out.write(f"{x:.1f},{y:.1f},{z:.1f}\ny\n")
        self.out.flush()
        time.sleep(settle)


class SerialArm:
    """Direct serial, same command as roarm/basicinput.py."""

    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, baudrate=115200, dsrdtr=None, timeout=0.1)
        self.ser.setRTS(False)
        self.ser.setDTR(False)

    def move(self, p, settle=0.0):
        x, y, z = (np.asarray(p) * 1000).tolist()
        self.ser.write(json.dumps({"T": 1041, "x": x, "y": y, "z": z, "t": 3.14}).encode() + b'\n')
        time.sleep(settle)


class ServerArm:
    """TCP client for roarm/roarm_server.py (which blocks until the move should be done)."""

    def __init__(self, host, port):
        self.host, self.port = host, port
        self.sock = socket.create_connection((host, port))
        self.inflight = threading.BoundedSemaphore(10)

    def move_nowait(self, p):
        """Send a move on its own short-lived connection and don't wait for the server's ~1 s settle reply,
        so following can issue small moves several times a second. Returns False (move dropped) if too many
        are still in flight."""
        if not self.inflight.acquire(blocking=False):
            return False
        x, y, z = np.asarray(p).tolist()
        msg = json.dumps({"type": "move_xyzt", "x": x, "y": y, "z": z, "grip_angle": 3.14}).encode()

        def send():
            try:
                with socket.create_connection((self.host, self.port), timeout=5) as s:
                    s.sendall(msg)
                    s.settimeout(15)
                    s.recv(4096)  # let the server finish its reply so it doesn't hit a reset connection
            except OSError:
                pass
            finally:
                self.inflight.release()
        threading.Thread(target=send, daemon=True).start()
        return True

    def move(self, p, settle=0.0):
        x, y, z = np.asarray(p).tolist()
        self.sock.sendall(json.dumps({"type": "move_xyzt", "x": x, "y": y, "z": z, "grip_angle": 3.14}).encode())
        self.sock.recv(4096)
        time.sleep(settle)


def fit_rigid(vr_pts, arm_pts):
    """Kabsch: find R, t minimizing |R @ vr + t - arm|. VR (Unity) is left-handed, so mirror x first."""
    A = np.array(vr_pts, dtype=float)
    A[:, 0] *= -1
    B = np.array(arm_pts, dtype=float)
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    t = cb - R @ ca
    residuals = np.linalg.norm((R @ A.T).T + t - B, axis=1)
    return R, t, residuals


def vr_to_arm(cal, p):
    q = np.array(p, dtype=float)
    q[0] *= -1
    return cal['R'] @ q + cal['t']


def average_point(receiver, hand, idx, seconds=0.75):
    samples = []
    end = time.time() + seconds
    while time.time() < end:
        pts = receiver.get(hand)
        if pts is not None:
            samples.append(pts[idx])
        time.sleep(0.02)
    if len(samples) < 5:
        return None
    return np.median(samples, axis=0)


def calibrate(arm, receiver, hand, link=None):
    print("\n=== CALIBRATION ===")
    print(f"Arm will visit {len(CAL_POINTS)} points. At each, touch the gripper tip with your "
          f"{hand} index fingertip, hold still, and press Enter (or type 's' to skip).")
    vr_pts, arm_pts = [], []
    for i, target in enumerate(CAL_POINTS):
        arm.move(target, settle=3.0)
        while True:
            ans = input(f"[{i + 1}/{len(CAL_POINTS)}] arm at {np.round(target, 3)} m. Touch, then Enter: ").strip().lower()
            if ans == 's':
                break
            tip = average_point(receiver, hand, INDEX_TIP)
            if tip is None:
                print("  No fresh hand data. Is the hand visible to the headset? Try again.")
                continue
            vr_pts.append(tip)
            arm_pts.append(target)
            if link is not None:
                link.add_pair(tip, target)
            break
    if len(vr_pts) < 3:
        sys.exit("Need at least 3 calibration points.")
    R, t, res = fit_rigid(vr_pts, arm_pts)
    print(f"Alignment residuals (cm): {np.round(res * 100, 1)}  RMS {np.sqrt(np.mean(res ** 2)) * 100:.1f} cm")
    arm.move(HOME, settle=2.0)
    return {'R': R, 't': t, 'residuals': res}


def follow(arm, receiver, hand, cal, min_step=0.005, max_step=0.015, period=0.25, link=None):
    print("\n=== FOLLOWING === pinch (thumb to index) to make the arm follow; release to stop. Ctrl-C to quit.")
    if link is not None:
        print("Using the nestbox aligner's live estimate (falls back to Kabsch until it is available).")
    last = HOME.copy()
    last_report = 0
    while True:
        if link is not None and time.time() - last_report > 5 and link.ready():
            print(f"  nestbox fit RMS {link.residual_rms() * 100:.1f} cm over {len(link.pairs)} pairs")
            last_report = time.time()
        pts = receiver.get(hand)
        if pts is not None and np.linalg.norm(pts[INDEX_TIP] - pts[THUMB_TIP]) < PINCH_DISTANCE:
            pinch = (pts[INDEX_TIP] + pts[THUMB_TIP]) / 2
            arm_pt = link.arm_from_unity(pinch) if link is not None else None
            target = clamp_to_safe(arm_pt if arm_pt is not None else vr_to_arm(cal, pinch))
            step = target - last
            if np.linalg.norm(step) > max_step:  # direct moves are fast, so approach big jumps gradually
                target = clamp_to_safe(last + step / np.linalg.norm(step) * max_step)
            if np.linalg.norm(target - last) > min_step:
                print(f"-> {np.round(target, 3)}")
                if hasattr(arm, 'move_nowait'):
                    if not arm.move_nowait(target):
                        time.sleep(period)
                        continue
                else:
                    arm.move(target)
                last = target
                if link is not None:
                    link.target = target
        time.sleep(period)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--bridge', metavar='SSH_HOST', help="use roarm/bridge.py's SSH tunnel to the Pi, e.g. pi@sdna-hdb")
    g.add_argument('--pipe', action='store_true', help='write moves to stdout for roarm/basicinput.py')
    g.add_argument('--serial', help='serial port of the arm (run on the Pi)')
    g.add_argument('--arm-host', help='host running roarm_server.py')
    ap.add_argument('--arm-port', type=int, default=8659)
    ap.add_argument('--listen-port', type=int, default=12345, help='port the headset streams Twigs to')
    ap.add_argument('--hand', default='righthand', help='Twig stream id of the hand to use')
    ap.add_argument('--load-cal', help='skip calibration and load this file')
    ap.add_argument('--save-cal', default='data/hand_cal.json')
    ap.add_argument('--index-tip', type=int, help='override INDEX_TIP point index')
    ap.add_argument('--thumb-tip', type=int, help='override THUMB_TIP point index')
    ap.add_argument('--probe', action='store_true', help='just print incoming hand data')
    ap.add_argument('--nestbox', action='store_true',
                    help='feed calibration pairs to a running nestbox daemon (set NESTBOX_CONFIG) and follow its estimate')
    ap.add_argument('--overlay-port', type=int, default=8651, help='with --nestbox: port the headset overlay connects to')
    ap.add_argument('--load-pairs', help='with --nestbox: replay saved calibration pairs instead of touching again')
    ap.add_argument('--save-pairs', default='data/cal_pairs.json')
    args = ap.parse_args()
    global INDEX_TIP, THUMB_TIP
    INDEX_TIP = args.index_tip if args.index_tip is not None else INDEX_TIP
    THUMB_TIP = args.thumb_tip if args.thumb_tip is not None else THUMB_TIP

    if args.pipe:
        # stdout belongs to basicinput.py now; everything human-facing (incl. input() prompts) goes to stderr
        pipe_out, sys.stdout = sys.stdout, sys.stderr

    receiver = HandReceiver(args.listen_port)
    receiver.start()

    if args.probe:
        while True:
            for hand, (ts, pts) in list(receiver.latest.items()):
                print(f"{hand}: {len(pts)} pts, age {time.time() - ts:.2f}s, root {np.round(pts[0], 3)}")
            time.sleep(1)

    bridge = None
    if args.bridge:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'roarm'))
        from bridge import RoArmBridge  # starts roarm_server.py on the Pi if needed, tunnels port 8659
        bridge = RoArmBridge(ssh_host=args.bridge)
        bridge.open()
        arm = ServerArm('localhost', bridge.local_port)
    elif args.pipe:
        arm = PipeArm(pipe_out)
    elif args.serial:
        arm = SerialArm(args.serial)
    else:
        arm = ServerArm(args.arm_host, args.arm_port)

    link = None
    if args.nestbox:
        from nestbox_link import NestboxLink, OverlayServer
        link = NestboxLink()
        OverlayServer(link, port=args.overlay_port)

    cal = None
    if args.load_cal:
        with open(args.load_cal) as f:
            d = json.load(f)
        cal = {'R': np.array(d['R']), 't': np.array(d['t'])}
    if args.load_pairs:
        with open(args.load_pairs) as f:
            d = json.load(f)
        R, t, res = fit_rigid(d['vr'], d['arm'])
        cal = {'R': R, 't': t, 'residuals': res}
        print(f"Loaded {len(d['vr'])} pairs; Kabsch RMS {np.sqrt(np.mean(res ** 2)) * 100:.1f} cm")
        if link is not None:
            for v, a in zip(d['vr'], d['arm']):
                link.add_pair(v, a)
    print(f"Waiting for '{args.hand}' data from the headset...")
    while (first := receiver.get(args.hand)) is None:
        time.sleep(0.2)
    if INDEX_TIP is None or THUMB_TIP is None:
        n = len(first)
        if n not in TIPS_BY_BONE_COUNT:
            sys.exit(f"Got {n} hand points; don't know the fingertip indices. Pass --index-tip and --thumb-tip.")
        THUMB_TIP, INDEX_TIP = TIPS_BY_BONE_COUNT[n]
        print(f"{n} hand points: thumb tip = {THUMB_TIP}, index tip = {INDEX_TIP}")

    if cal is None:
        cal = calibrate(arm, receiver, args.hand, link)
        if link is not None:
            link.save_pairs(args.save_pairs)
            print(f"Saved raw calibration pairs to {args.save_pairs} (replay with --load-pairs)")
        with open(args.save_cal, 'w') as f:
            json.dump({'R': cal['R'].tolist(), 't': cal['t'].tolist(),
                       'residuals': cal['residuals'].tolist()}, f, indent=2)
        print(f"Saved calibration to {args.save_cal}")

    try:
        follow(arm, receiver, args.hand, cal, link=link)
    except KeyboardInterrupt:
        arm.move(HOME)
    finally:
        if bridge is not None:
            bridge.close()


if __name__ == '__main__':
    main()
