# Allow both direct script execution and python -m from the repository root.
if __package__ in (None, ''):
    import sys
    from pathlib import Path as _Path
    sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

import unittest
import numpy as np
from LIPM.demos.demo_LIPM_3D_mass_switch import create_model,simulate
from LIPM.demos.demo_LIPM_3D_double_support import create_model as baseline,simulate as baseline_simulate


class PeriodicMassTests(unittest.TestCase):
    def test_full_trajectory_repeats_after_two_steps(self):
        for foot_mass in [0.,3.,10.]:
            for ss,ds in [(0.48,0.12),(0.6,0.)]:
                for velocity in [(0.3,0.),(0.3,0.2),(0.,0.)]:
                    with self.subTest(mass=foot_mass,ds=ds,velocity=velocity):
                        m=create_model(t_ss=ss,t_ds=ds,velocity=velocity,body_mass=40.,foot_mass=foot_mass,periodic_start=True)
                        expected=m.periodic_velocity.copy()
                        theta=np.arctan2(velocity[1],velocity[0])
                        d=simulate(m,4*(ss+ds),theta)
                        n=round(2*(ss+ds)/m.dt)
                        shift=expected*2*(ss+ds)
                        for key in ['position','zmp','target']:
                            np.testing.assert_allclose(d[key][n:]-d[key][:-n],np.broadcast_to(shift,d[key][n:].shape),atol=1e-9)
                        for key in ['body_position','whole_com','planning_com','feet','effective_com']:
                            delta=d[key][n:]-d[key][:-n]
                            np.testing.assert_allclose(delta,np.broadcast_to(np.r_[shift,0.],delta.shape),atol=1e-9)
                        for key in ['velocity','body_velocity','foot_velocity']:
                            np.testing.assert_allclose(d[key][n:],d[key][:-n],atol=1e-9)

    def test_zero_mass_matches_original_periodic(self):
        for ss,ds in [(0.48,0.12),(0.6,0.)]:
            m=create_model(t_ss=ss,t_ds=ds,foot_mass=0.,periodic_start=True)
            b=baseline(t_ss=ss,t_ds=ds,velocity=(0.3,0.),periodic_start=True)
            a,c=simulate(m,2.4),baseline_simulate(b,2.4)
            for key in ['position','velocity','feet','zmp','target']:
                np.testing.assert_allclose(a[key],c[key],atol=1e-9)

    def test_ordinary_start_preserved(self):
        m=create_model(body_mass=40.,foot_mass=10.)
        np.testing.assert_allclose(m.body_pos,[0,0,0.6])
        np.testing.assert_allclose(m.body_velocity,[0.3,0,0])
        np.testing.assert_allclose(m.feet(),[[0,0.2,0],[0,-0.2,0]])


if __name__ == '__main__':
    unittest.main()
