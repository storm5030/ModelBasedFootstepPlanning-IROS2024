"""Numerical and gait invariants: python -m unittest discover -s LIPM/tests -p test_double_support.py"""
# Allow both direct script execution and python -m from the repository root.
if __package__ in (None, ''):
    import sys
    from pathlib import Path as _Path
    sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

import unittest
import numpy as np
from LIPM.models.LIPM_3D import LIPM3D
from LIPM.models.LIPM_3D_double_support import LIPM3DDoubleSupport
from LIPM.demos.demo_LIPM_3D_double_support import create_model, simulate


class DoubleSupportTests(unittest.TestCase):
    def test_linear_zmp_against_independent_rk4(self):
        p0, p1 = np.array([0.1, -0.2]), np.array([0.3, 0.2])
        c, v = np.array([0.05, 0.01]), np.array([0.4, -0.3])
        duration, omega, n = 0.08, np.sqrt(9.81/0.6), 1000
        expected = LIPM3DDoubleSupport.propagate(c, v, p0, p1, duration, omega)
        state = np.r_[c, v]
        dt = duration/n
        def derivative(t, x):
            p = p0 + (p1-p0)*t/duration
            return np.r_[x[2:], omega**2*(x[:2]-p)]
        for i in range(n):
            t = i*dt
            k1 = derivative(t, state)
            k2 = derivative(t+dt/2, state+dt*k1/2)
            k3 = derivative(t+dt/2, state+dt*k2/2)
            k4 = derivative(t+dt, state+dt*k3)
            state += dt*(k1+2*k2+2*k3+k4)/6
        np.testing.assert_allclose(state, np.r_[expected[0], expected[1]], atol=1e-11)

    def test_zero_dsp_matches_original_foothold(self):
        new = create_model(periodic_start=True, t_ds=0, velocity=(0.5, 0.2))
        initial = new.snapshot()
        heading = np.arctan2(0.2, 0.5)
        old = LIPM3D(dt=new.dt, T=new.T, s_d=new.s_d,
                     w_d=new.w_d, support_leg='left_leg')
        old.initializeModel([*initial["position"], new.zc], new.left_foot_pos.tolist(), new.right_foot_pos.tolist())
        old.x_0, old.y_0 = initial["position"]-new.left_foot_pos[:2]
        old.vx_0, old.vy_0 = initial["velocity"]
        old.calculateFootLocationForNextStepXcoMWorld(heading)
        np.testing.assert_allclose([new.u_x, new.u_y], [old.u_x, old.u_y], atol=1e-12)
        for _ in range(round(new.T/new.dt)):
            old.step()
            new.step()
        np.testing.assert_allclose(new.COM_pos[:2], np.array([old.x_t, old.y_t])+old.support_foot_pos[:2], atol=1e-12)
        np.testing.assert_allclose([new.vx_t, new.vy_t], [old.vx_t, old.vy_t], atol=1e-12)

    def test_periodic_gait_contacts_and_zmp(self):
        for t_ds in (0, 0.08, 0.16):
            for velocity in ((0.5, 0), (0.3, 0.2), (0, 0)):
                model = create_model(periodic_start=True, t_ds=t_ds, velocity=velocity)
                data = simulate(model, 20*model.T_d, theta=np.arctan2(velocity[1], velocity[0]))
                for i in range(1, len(data['time'])):
                    before_feet, feet = data['feet'][i-1], data['feet'][i]
                    if data['phase'][i-1] == 'DSP':
                        np.testing.assert_allclose(feet, before_feet, atol=1e-12)
                        np.testing.assert_allclose(feet[:, 2], 0, atol=1e-12)
                        a, b = feet[:, :2]
                        delta = b-a
                        zmp = data['zmp'][i]
                        alpha = np.dot(zmp-a, delta)/np.dot(delta, delta)
                        self.assertGreaterEqual(alpha, -1e-10)
                        self.assertLessEqual(alpha, 1+1e-10)
                        np.testing.assert_allclose(zmp, a+alpha*delta, atol=1e-10)
                    else:
                        support = data['support'][i-1]
                        np.testing.assert_allclose(feet[support], before_feet[support], atol=1e-12)
                self.assertTrue(np.isfinite(data['position']).all())
                np.testing.assert_allclose((data['position'][-1]-data['position'][0])/model.time, velocity, atol=1e-9)
                np.testing.assert_allclose(data['velocity'][-1], data['velocity'][0], atol=1e-9)

    def test_capture_target_after_perturbation(self):
        model = create_model(periodic_start=True, initial_velocity=(0.65, 0.1))
        simulate(model, model.T_d)
        relative_xi = np.array([model.x_t+model.vx_t/model.w_0, model.y_t+model.vy_t/model.w_0])
        E = np.exp(model.w_0*model.T_d)
        K = np.expm1(model.w_0*model.T_ds)/(model.w_0*model.T_ds)
        np.testing.assert_allclose(relative_xi, [K*model.s_d/(E-1), K*model.w_d/(E+1)], atol=1e-11)

    def test_current_initial_conditions_and_command_schedule(self):
        model = create_model()
        np.testing.assert_allclose(model.COM_pos, [0, 0, 0.6])
        np.testing.assert_allclose([model.vx_t, model.vy_t], [0.3, 0])
        np.testing.assert_allclose(model.left_foot_pos, [0., 0.2, 0])
        np.testing.assert_allclose(model.right_foot_pos, [0., -0.2, 0])
        self.assertAlmostEqual(model.s_d, model.T_d)
        data = simulate(model, 31*model.T_d, step_to_cmdv=[10, 20, 30],
                        COM_dvel_list=np.array([[1., 0.]]*4), w_d_list=[0.4, 0.8, 0.4, 0.4])
        steps = data['step_num']
        np.testing.assert_allclose(data['dstep_width'], np.where((steps >= 10) & (steps < 20), 0.8, 0.4))
        np.testing.assert_allclose(data['command'], np.tile([1., 0.], (len(steps), 1)))
        np.testing.assert_allclose(data['dstep_length'][steps == 0], model.T_d)
        np.testing.assert_allclose(data['dstep_length'][steps > 0], model.T_d)
        self.assertTrue(np.isfinite(data['velocity']).all())

    def test_reject_nonintegral_phase_duration(self):
        with self.assertRaises(ValueError):
            LIPM3DDoubleSupport(T_ds=0.085)


if __name__ == '__main__':
    unittest.main()
