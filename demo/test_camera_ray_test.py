"""Headless camera measurement/overlay regression: python test_camera_ray_test.py."""
from types import SimpleNamespace
from unittest.mock import patch
import json
import numpy as np
from camera_ray_test import CameraRayTest, MIRROR, T_CYAN, cone, depth_prior
from nestbox.measurement import BearingMeasurement, NormalMeasurement


def main():
    # Synthetic right hand points, in Unity coordinates, defining an oblique camera pose.
    pts = np.zeros((24, 3))
    pts[0] = [.4, 1.2, .3]
    pts[9] = [.45, 1.23, .38]
    pts[6] = [.40, 1.26, .36]
    pts[16] = [.49, 1.26, .30]
    for mode in ('sampled', 'focal', 'bearing'):
        args = SimpleNamespace(pixel_sigma=2., focal=1100., convergence_time=0., depth_mode=mode,
                               depth_sigma=.5, focal_distance=2.5, fresh_noise=False, depth_prior=None)
        with patch('threading.Thread.start'):
            test = CameraRayTest(args)
        test.update(pts)
        for _ in range(30):
            test.aligner.gradient_descent_step()
        measurements = list(test.cam.measurements.values())
        assert len(measurements) == 8
        R, o, cov = test.estimated_pose()
        state = test.overlay_state()
        json.dumps(state, allow_nan=False)
        assert len(state['lines']) == 6*len(state['lineColors'])
        assert len(state['ellipsoids']) == 10*len(state['ellipsoidColors'])
        if mode == 'bearing':
            assert all(isinstance(m, BearingMeasurement) and m.depth_prior is None for m in measurements)
            assert state['lineColors'].count(T_CYAN) == 8*20  # 8 generators + 12 ring segments per cone
            assert T_CYAN not in state['ellipsoidColors']
            error = np.linalg.norm(o-test.true_pose[1])
            assert error < .01, error
            before = o.copy()
            for _ in range(100):
                test.aligner.gradient_descent_step()
            drift = np.max(np.abs(test.estimated_pose()[1]-before))
            assert drift < 1e-9, drift
            print(f'bearing demo: {100*error:.2f} cm error, drift {drift:.1e} m, 160 cone segments')
        else:
            assert all(isinstance(m, NormalMeasurement) for m in measurements)
            assert state['ellipsoidColors'].count(T_CYAN) == 8
            if mode == 'focal':
                assert np.allclose([np.linalg.norm(m.mean) for m in measurements], 2.5)
            print(f'{mode} demo: Gaussian measurements and ellipsoids verified')
    # Inspect anisotropic cone extents and handedness independently of the solver.
    state = dict(lines=[], lineColors=[])
    cone(np.array([1., 2., 3.]), np.array([0., 0., 1.]), np.array([.1, .2]), 3.,
         np.eye(3)[:, :2], 2., T_CYAN, state)
    segments = np.array(state['lines']).reshape(-1, 2, 3) * MIRROR
    assert np.allclose(segments[0, 0], [1, 2, 3])
    assert np.allclose(segments[0, 1], [1+3*np.tan(.1), 2, 6])
    assert np.allclose(segments[8:, :, 2], 5)
    assert depth_prior('2.5,1') == (2.5, 1.)
    print('cone ring geometry, anisotropy, handedness, and JSON verified')


if __name__ == '__main__':
    main()
