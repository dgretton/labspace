"""Nestbox aligner link and VR overlay for follow_hand.py.

NestboxLink feeds calibration touch pairs to a running nestbox daemon (old nestbox, branch `armdemo`) as
gripper-tip features seen in two coordinate systems: 'roarm' (commanded arm position) and 'vr' (index
fingertip, mirrored from Unity's left-handed frame). 'vr' is pinned; the aligner estimates where 'roarm'
sits in 'vr' and keeps refining it. NestboxLink polls that estimate.

OverlayServer streams ready-to-draw geometry in Unity world coordinates to the headset (NestboxOverlay.cs)
as one JSON line per update over plain TCP: the arm base box and axes at the current estimate, the VR touch
points, the arm points mapped through the estimate, and residual lines between them.

Daemon convention (docs/ARM_DEMO_INTEGRATION.md in the nestbox repo):
GET /transforms/convert/roarm/to/vr gives p_vr = R(q) p_roarm + origin, q in (w, x, y, z).
"""
import json
import socket
import threading
import time

import numpy as np

MIRROR = np.array([-1.0, 1.0, 1.0])
ARM_SIGMA = 0.005  # commanded arm position, meters
VR_SIGMA = 0.015   # fingertip touching the gripper; first lab Kabsch residuals were ~2 cm

# overlay palette indices (must match NestboxOverlay.cs)
WHITE, RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA = range(7)


def quat_wxyz_to_matrix(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class NestboxLink:
    def __init__(self, smoothing=0.25, poll_period=0.1):
        import nestbox
        from nestbox.api import _get_client
        self.nestbox = nestbox
        self.client = _get_client()
        nestbox.create_coordinate_system('vr')
        nestbox.create_coordinate_system('roarm')
        self.pairs = []  # (vr point in Unity coords, arm point)
        self.started = False
        self.lock = threading.Lock()
        self.raw = None       # latest (R, o) from the daemon
        self.smoothed = None  # display pose, eased toward raw
        self.smoothing = smoothing
        self.poll_period = poll_period
        self.target = None    # current arm target, arm frame, for the overlay
        self.last_error = None
        threading.Thread(target=self._poll, daemon=True).start()

    def add_pair(self, vr_unity, arm):
        k = len(self.pairs)
        feature = f'nestbox:feature/roarm/gripper-tip/cal/{k}'
        vr = np.asarray(vr_unity, dtype=float) * MIRROR
        self.nestbox.add_measurement(feature, 'roarm', list(map(float, arm)), (np.eye(3) * ARM_SIGMA ** 2).tolist())
        self.nestbox.add_measurement(feature, 'vr', vr.tolist(), (np.eye(3) * VR_SIGMA ** 2).tolist())
        with self.lock:
            self.pairs.append((np.asarray(vr_unity, dtype=float), np.asarray(arm, dtype=float)))
        if len(self.pairs) >= 3 and not self.started:
            self.nestbox.pin('vr')
            self.nestbox.start_alignment()
            self.started = True
            print("  nestbox aligner started; watch the box drift into place")

    def _poll(self):
        while True:
            time.sleep(self.poll_period)
            if not self.started:
                continue
            try:
                r = self.client.get_transform('roarm', 'vr')
                R, o = quat_wxyz_to_matrix(np.array(r.quaternion, dtype=float)), np.array(r.origin, dtype=float)
            except Exception as e:  # keep polling; report once per distinct error
                if str(e) != self.last_error:
                    print(f"  nestbox poll error: {e}")
                    self.last_error = str(e)
                continue
            with self.lock:
                self.raw = (R, o)
                if self.smoothed is None:
                    self.smoothed = (R, o)
                else:
                    a = self.smoothing
                    Rs = (1 - a) * self.smoothed[0] + a * R
                    U, _, Vt = np.linalg.svd(Rs)  # back onto a rotation
                    self.smoothed = (U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt, (1 - a) * self.smoothed[1] + a * o)

    def ready(self):
        return self.raw is not None

    def arm_from_unity(self, p_unity):
        """Arm-frame point for a Unity world point, using the latest raw estimate."""
        with self.lock:
            if self.raw is None:
                return None
            R, o = self.raw
        return R.T @ (np.asarray(p_unity, dtype=float) * MIRROR - o)

    def residual_rms(self):
        with self.lock:
            if self.raw is None or not self.pairs:
                return None
            R, o = self.raw
            res = [np.linalg.norm((R @ a + o) - v * MIRROR) for v, a in self.pairs]
        return float(np.sqrt(np.mean(np.square(res))))

    def save_pairs(self, path):
        with self.lock:
            d = {'vr': [v.tolist() for v, _ in self.pairs], 'arm': [a.tolist() for _, a in self.pairs]}
        with open(path, 'w') as f:
            json.dump(d, f, indent=2)

    def overlay_state(self):
        lines, line_colors, points, point_colors, point_sizes = [], [], [], [], []
        with self.lock:
            pairs = list(self.pairs)
            pose = self.smoothed
            target = self.target

        def seg(a, b, c):
            lines.extend(list(a) + list(b)); line_colors.append(c)

        def dot(p, c, s):
            points.extend(list(p)); point_colors.append(c); point_sizes.append(s)

        for v, _ in pairs:
            dot(v, GREEN, 0.018)  # where the fingertip touched (measurement in 'vr')
        if pose is not None:
            R, o = pose
            to_u = lambda p: (R @ np.asarray(p, dtype=float) + o) * MIRROR
            # arm base box (arm frame: x forward, z up) and axes
            c = np.array([[x, y, z] for x in (-0.3, 0.3) for y in (-0.3, 0.3) for z in (0.0, 0.5)])
            for i in range(8):
                for j in range(i + 1, 8):
                    if np.sum(np.abs(c[i] - c[j]) > 1e-9) == 1:
                        seg(to_u(c[i]), to_u(c[j]), WHITE)
            for axis, col in ((0, RED), (1, GREEN), (2, BLUE)):
                e = np.zeros(3); e[axis] = 0.15
                seg(to_u([0, 0, 0]), to_u(e), col)
            for v, a in pairs:
                pa = to_u(a)
                dot(pa, CYAN, 0.012)  # arm measurement mapped through the estimate
                seg(v, pa, YELLOW)    # residual
            if target is not None:
                dot(to_u(target), MAGENTA, 0.02)
                seg(to_u([0, 0, 0.10]), to_u(target), MAGENTA)
        return {'lines': [round(float(x), 5) for x in lines], 'lineColors': line_colors,
                'points': [round(float(x), 5) for x in points], 'pointColors': point_colors,
                'pointSizes': point_sizes}


class OverlayServer:
    """Sends link.overlay_state() as JSON lines to every connected headset client."""

    def __init__(self, link, port=8651, period=0.1):
        self.link, self.port, self.period = link, port, period
        self.clients = []
        self.lock = threading.Lock()
        threading.Thread(target=self._accept, daemon=True).start()
        threading.Thread(target=self._broadcast, daemon=True).start()

    def _accept(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', self.port))
        srv.listen(4)
        print(f"Overlay server on port {self.port}")
        while True:
            conn, addr = srv.accept()
            print(f"Overlay client connected from {addr[0]}")
            with self.lock:
                self.clients.append(conn)

    def _broadcast(self):
        while True:
            time.sleep(self.period)
            with self.lock:
                if not self.clients:
                    continue
                clients = list(self.clients)
            data = (json.dumps(self.link.overlay_state(), separators=(',', ':')) + '\n').encode()
            for c in clients:
                try:
                    c.sendall(data)
                except OSError:
                    with self.lock:
                        if c in self.clients:
                            self.clients.remove(c)
                    c.close()
