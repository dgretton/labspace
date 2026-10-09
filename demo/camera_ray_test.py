"""Handheld 'camera' ray test for the Gauss-Newton aligner, drawn live in the Quest.

Your right hand is the camera: it looks along wrist -> middle knuckle, image-up is the back of the hand, FOV 60 x 45 deg.
A cluster of test features is placed in the pinned 'vr' world in front of where the camera points at startup
(pinch your LEFT hand for ~0.5 s to re-place it). Every update, each feature inside the frustum gets a camera
bearing measurement with angular uncertainty (a couple of pixels), drawn as a cone.
Compare --depth-mode sampled (noisy true range) or focal (fixed-distance Gaussian).
The aligner (in-process, nestbox armdemo branch) back-solves the camera pose from those measurements alone.

Overlay (Unity world): white dots = features; green frustum = true camera (your hand); magenta frustum = back-solved
camera; cyan = each camera measurement's 1-sigma cone (or Gaussian ellipsoid), placed through the *estimated* pose (they
should converge on the features); translucent yellow = 2-sigma uncertainty of the solved camera position.

Run (Quest app running; nothing else on ports 12345/8651):
  adb reverse tcp:12345 tcp:12345 && adb reverse tcp:8651 tcp:8651
  PYTHONPATH=<nestbox repo>/python python camera_ray_test.py [--depth-mode bearing] [--pixel-sigma 2] [--fresh-noise]
"""
import argparse
import itertools
import threading
import time

import numpy as np

from follow_hand import HandReceiver
from nestbox_link import OverlayServer, MIRROR, WHITE, GREEN, MAGENTA
from nestbox.aligner.gauss_newton_aligner import GaussNewtonAligner, _quat_to_matrix, _matrix_to_quat
from nestbox.coordsystem import CoordinateSystem
from nestbox.measurement import NormalMeasurement, BearingMeasurement
from nestbox.feature import to_feature

# translucent palette entries (must match NestboxOverlay.cs)
T_CYAN, T_YELLOW, T_WHITE = 7, 8, 9

WRIST, INDEX1, MIDDLE1, PINKY1, THUMB_TIP, INDEX_TIP = 0, 6, 9, 16, 19, 20
HALF_FOV_X, HALF_FOV_Y = np.radians(30), np.radians(22.5)
# feature cluster in the camera frame at placement time (x right, y up, z forward), non-coplanar for a clean PnP
CLUSTER = np.array([[-0.20, -0.10, 0.70], [0.00, -0.12, 0.85], [0.20, -0.08, 0.75], [-0.15, 0.10, 1.00],
                    [0.10, 0.12, 0.95], [0.25, 0.05, 1.15], [-0.05, 0.00, 1.25], [-0.25, 0.02, 0.90]])
_ids = itertools.count()


def hand_camera(pts):
    """Right-hand camera pose in nestbox's right-handed world (Unity x mirrored): p_world = R p_cam + o."""
    p = np.asarray(pts, dtype=float) * MIRROR
    f = p[MIDDLE1] - p[WRIST]
    f /= np.linalg.norm(f)
    # back of the right hand; in the mirrored (right-handed) frame the cross product flips relative to Unity's
    up = np.cross(f, p[INDEX1] - p[PINKY1])
    up -= (up @ f) * f
    up /= np.linalg.norm(up)
    R = np.column_stack([np.cross(up, f), up, f])
    return R, (p[WRIST] + p[MIDDLE1]) / 2


def ellipsoid(center_rh, cov_rh, k, color, out):
    """Append a Unity-space ellipsoid: center, rotation (x,y,z,w), full axis lengths (2k sigma)."""
    c = center_rh * MIRROR
    C = np.diag(MIRROR) @ cov_rh @ np.diag(MIRROR)
    evals, V = np.linalg.eigh((C + C.T) / 2)
    if np.linalg.det(V) < 0:
        V[:, 0] *= -1
    w, x, y, z = _matrix_to_quat(V)
    size = np.maximum(2 * k * np.sqrt(np.maximum(evals, 0)), 0.002)
    out['ellipsoids'].extend([*c, x, y, z, w, *size])
    out['ellipsoidColors'].append(color)


def cone(apex, axis, half_angles, length, basis, ring_depth, color, out):
    """Eight generators and a twelve-segment uncertainty ring, mirrored into Unity."""
    def rim(theta, depth):
        offset = basis @ (np.tan(half_angles) * [np.cos(theta), np.sin(theta)])
        return apex + depth * (axis + offset)

    def line(a, b):
        out['lines'].extend([*(a * MIRROR), *(b * MIRROR)])
        out['lineColors'].append(color)

    for theta in np.linspace(0, 2*np.pi, 8, endpoint=False):
        line(apex, rim(theta, length))
    ring = [rim(theta, ring_depth) for theta in np.linspace(0, 2*np.pi, 12, endpoint=False)]
    for i in range(12):
        line(ring[i], ring[(i+1) % 12])


class CameraRayTest:
    def __init__(self, args):
        self.args = args
        self.rng = np.random.default_rng(0)
        self.sigma_theta = args.pixel_sigma / args.focal
        self.aligner = GaussNewtonAligner(convergence_time=args.convergence_time, step_interval=0.02)
        self.vr = CoordinateSystem(f'vr_{next(_ids)}')
        self.cam = CoordinateSystem(f'cam_{next(_ids)}')
        self.aligner.add_coordinate_system(self.vr)
        self.aligner.add_coordinate_system(self.cam)
        self.aligner.pin(self.vr)
        self.features = None   # world (right-handed) positions
        self.noise = {}        # feature index -> frozen noise while it stays in view
        self.true_pose = None
        self.visible = []
        self.lock = threading.Lock()
        threading.Thread(target=self._run_aligner, daemon=True).start()

    def _run_aligner(self):
        while True:
            self.aligner.gradient_descent_step()
            time.sleep(self.aligner.step_interval)

    def place_features(self, R, o):
        X = CLUSTER @ R.T + o
        with self.aligner.lock:
            self.vr.measurements = {}
            for j, x in enumerate(X):
                f = to_feature(f'testpoint/{j}')
                self.vr.measurements[f] = NormalMeasurement(f, x, np.eye(3) * 0.002 ** 2)
        self.features = X
        self.noise = {}
        print(f"Placed {len(X)} test features in front of the camera")

    def update(self, pts):
        R, o = hand_camera(pts)
        if self.features is None:
            self.place_features(R, o)
        meas, visible = {}, []
        for j, x in enumerate(self.features):
            pc = R.T @ (x - o)
            in_view = 0.15 < pc[2] < 4 and abs(np.arctan2(pc[0], pc[2])) < HALF_FOV_X and abs(np.arctan2(pc[1], pc[2])) < HALF_FOV_Y
            if not in_view:
                self.noise.pop(j, None)
                continue
            visible.append(j)
            if self.args.fresh_noise or j not in self.noise:
                self.noise[j] = self.rng.normal(size=3)
            n = self.noise[j]
            d = pc / np.linalg.norm(pc)
            e1 = np.cross(d, [0.0, 1.0, 0.0] if abs(d[1]) < 0.9 else [1.0, 0.0, 0.0])
            e1 /= np.linalg.norm(e1)
            e2 = np.cross(d, e1)
            d_obs = d + self.sigma_theta * (n[0] * e1 + n[1] * e2)
            d_obs /= np.linalg.norm(d_obs)
            f = to_feature(f'testpoint/{j}')
            if self.args.depth_mode == 'bearing':
                meas[f] = BearingMeasurement(f, d_obs, self.sigma_theta, depth_prior=self.args.depth_prior)
            else:
                r = (max(np.linalg.norm(pc) + self.args.depth_sigma * n[2], 0.05)
                     if self.args.depth_mode == 'sampled' else self.args.focal_distance)
                cov = ((r * self.sigma_theta) ** 2 * (np.eye(3) - np.outer(d_obs, d_obs))
                       + self.args.depth_sigma ** 2 * np.outer(d_obs, d_obs))
                meas[f] = NormalMeasurement(f, r * d_obs, cov)
        with self.aligner.lock:
            self.cam.measurements = meas
        with self.lock:
            self.true_pose, self.visible = (R, o), visible

    def estimated_pose(self):
        with self.aligner.lock:
            i = self.aligner.coordinate_systems.index(self.cam)
            R = _quat_to_matrix(self.aligner.current_orientations[i])
            o = np.array(self.aligner.current_origins[i], dtype=float)
            cov = self.aligner.pose_covariances.get(self.cam.name)
        return R, o, cov

    def overlay_state(self):
        out = {'lines': [], 'lineColors': [], 'points': [], 'pointColors': [], 'pointSizes': [],
               'ellipsoids': [], 'ellipsoidColors': []}
        if self.features is None:
            return out
        for x in self.features:
            out['points'].extend(list(x * MIRROR)); out['pointColors'].append(WHITE); out['pointSizes'].append(0.012)

        def frustum(R, o, color, depth=0.3):
            corners = [R @ (np.array([sx * np.tan(HALF_FOV_X), sy * np.tan(HALF_FOV_Y), 1.0]) * depth) + o
                       for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            for k, c in enumerate(corners):
                for a, b in ((o, c), (c, corners[(k + 1) % 4])):
                    out['lines'].extend(list(a * MIRROR) + list(b * MIRROR)); out['lineColors'].append(color)

        with self.lock:
            true_pose = self.true_pose
        if true_pose is not None:
            frustum(*true_pose, GREEN)
        if self.cam.measurements:
            R, o, cov = self.estimated_pose()
            frustum(R, o, MAGENTA)
            for _, _, c, C in self.aligner.measurement_ellipsoids([self.cam.name]):
                ellipsoid(c, C, 1.0, T_CYAN, out)
            with self.aligner.lock:
                cones = self.aligner.measurement_cones(cs_names=[self.cam.name], include_basis=True)
                for _, f, apex, axis, angles, length, basis in cones:
                    estimate = self.aligner.feature_estimates.get(f)
                    depth = max(float(axis @ (estimate[0] - apex)), 0.0) if estimate is not None else length
                    cone(apex, axis, angles, length, basis, depth, T_CYAN, out)
            if cov is not None and np.all(np.isfinite(cov)) and np.max(np.linalg.eigvalsh(cov[3:, 3:])) < 1.0:
                ellipsoid(o, cov[3:, 3:], 2.0, T_YELLOW, out)
        for key in ('lines', 'points', 'ellipsoids'):
            out[key] = [round(float(v), 5) for v in out[key]]
        return out


def depth_prior(value):
    try:
        rho, sigma = map(float, value.split(','))
        if not np.all(np.isfinite([rho, sigma])) or min(rho, sigma) <= 0:
            raise ValueError
        return rho, sigma
    except ValueError:
        raise argparse.ArgumentTypeError('expected positive rho0,sigma_log (e.g. 2.5,1)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pixel-sigma', type=float, default=2.0, help='angular noise, pixels')
    ap.add_argument('--focal', type=float, default=1100.0, help='focal length, pixels (1100 ~ 60 deg FOV at 1280 px)')
    ap.add_argument('--depth-mode', choices=('sampled', 'focal', 'bearing'), default='bearing',
                    help='sampled: noisy true range; focal: fixed-depth Gaussian; bearing: angular only (default)')
    ap.add_argument('--focal-distance', type=float, default=2.5, help='Gaussian focal range, meters')
    ap.add_argument('--depth-prior', type=depth_prior, help='bearing log-range prior: rho0,sigma_log (e.g. 2.5,1)')
    ap.add_argument('--depth-sigma', type=float, default=0.5, help='depth (along-ray) sigma, meters')
    ap.add_argument('--fresh-noise', action='store_true', help='resample noise every update (default: frozen per feature while in view)')
    ap.add_argument('--convergence-time', type=float, default=0.3, help='aligner time constant, seconds (0 = jump)')
    ap.add_argument('--hz', type=float, default=10.0, help='measurement update rate')
    ap.add_argument('--listen-port', type=int, default=12345)
    ap.add_argument('--overlay-port', type=int, default=8651)
    args = ap.parse_args()
    for name in ('pixel_sigma', 'focal', 'focal_distance', 'depth_sigma', 'hz'):
        if not np.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            ap.error(f'--{name.replace("_", "-")} must be positive and finite')
    if args.depth_prior is not None and args.depth_mode != 'bearing':
        ap.error('--depth-prior requires --depth-mode bearing')

    test = CameraRayTest(args)
    receiver = HandReceiver(args.listen_port)
    receiver.start()
    OverlayServer(test, port=args.overlay_port)
    print("Point your right hand like a camera at the features. Pinch your left hand ~0.5 s to re-place them. Ctrl-C to quit.")
    pinch_since, last_report = None, 0
    while True:
        time.sleep(1.0 / args.hz)
        right, left = receiver.get('righthand'), receiver.get('lefthand')
        if left is not None and np.linalg.norm(left[THUMB_TIP] - left[INDEX_TIP]) < 0.025 and right is not None:
            pinch_since = pinch_since or time.time()
            if time.time() - pinch_since > 0.5:
                test.place_features(*hand_camera(right))
                pinch_since = -1e9  # once per pinch
        else:
            pinch_since = None
        if right is None:
            continue
        test.update(right)
        if time.time() - last_report > 2 and test.cam.measurements:
            R, o, cov = test.estimated_pose()
            Rt, ot = test.true_pose
            ang = np.degrees(np.arccos(np.clip((np.trace(R.T @ Rt) - 1) / 2, -1, 1)))
            sd = np.sqrt(np.trace(cov[3:, 3:]) / 3) * 100 if cov is not None else float('nan')
            print(f"{len(test.visible)} features in view   solved vs true camera: {np.linalg.norm(o - ot) * 100:5.1f} cm, "
                  f"{ang:5.1f} deg   reported position sd {sd:6.1f} cm")
            last_report = time.time()


if __name__ == '__main__':
    main()
