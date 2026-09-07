# Allow both direct script execution and python -m from the repository root.
if __package__ in (None, ''):
    import sys
    from pathlib import Path as _Path
    sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

import unittest
import numpy as np
from LIPM.demos.demo_LIPM_3D_mass_switch import create_model, simulate
from LIPM.demos.demo_LIPM_3D_double_support import create_model as baseline_model, simulate as baseline_simulate


class MassSwitchTests(unittest.TestCase):
    def test_zero_mass_matches_baseline(self):
        for ss,ds in [(0.48,0.12),(0.6,0.)]:
            new=create_model(t_ss=ss,t_ds=ds,foot_mass=0.)
            old=baseline_model(t_ss=ss,t_ds=ds,velocity=(0.3,0.))
            a,b=simulate(new,3.),baseline_simulate(old,3.)
            for key in ['position','velocity','zmp','feet','target']:
                np.testing.assert_allclose(a[key],b[key],atol=1e-10)

    def test_touchdown_preserves_body_and_reweights_state(self):
        m=create_model()
        body=m.body_pos.copy(); velocity=m.body_velocity.copy()
        m.beginDoubleSupport()
        np.testing.assert_array_equal(m.body_pos,body)
        np.testing.assert_array_equal(m.body_velocity,velocity)
        self.assertEqual(m.active_foot,0)
        np.testing.assert_allclose(m.COM_pos,(1-m.alpha)*body+m.alpha*m.left_foot_pos)
        np.testing.assert_allclose([m.vx_t,m.vy_t],(1-m.alpha)*velocity[:2])

    def test_com_identity_height_and_switch_timing(self):
        for mass in [0.,3.,10.]:
            m=create_model(body_mass=30.,foot_mass=mass)
            d=simulate(m,3.)
            a=m.alpha
            np.testing.assert_allclose(d['effective_com'],(1-a)*d['body_position'][:,None,:]+a*d['feet'])
            np.testing.assert_allclose(d['whole_com'],(30*d['body_position']+mass*d['feet'].sum(axis=1))/(30+2*mass))
            np.testing.assert_allclose(d['planning_height'],(1-a)*0.6)
            np.testing.assert_allclose(d['body_position'][:,2],0.6)
            active=d['effective_com'][np.arange(len(d['time'])),d['active_foot']]
            np.testing.assert_allclose(active[:,:2],d['planning_com'][:,:2],atol=1e-12)
            for i in np.flatnonzero(np.diff(d['active_foot']))+1:
                self.assertEqual(d['phase'][i],'DSP')
                np.testing.assert_allclose(d['feet'][i,:,2],0.,atol=1e-12)
            self.assertTrue(np.isfinite(d['position']).all())

    def test_terminal_capture_constraint(self):
        m=create_model()
        theta=0.; side=-1
        K=m._double_support_gain(); E=np.exp(m.w_0*m.T_d)
        expected=np.array([K*m.s_d/(E-1),-side*K*m.w_d/(E+1)])
        simulate(m,m.T_d)
        np.testing.assert_allclose(np.array([m.x_0+m.vx_0/m.w_0,m.y_0+m.vy_0/m.w_0]),expected,atol=1e-11)

    def test_invalid_mass(self):
        for body,foot in [(0,3),(30,-1),(np.nan,3)]:
            with self.assertRaises(ValueError):
                create_model(body_mass=body,foot_mass=foot)


if __name__ == '__main__':
    unittest.main()
