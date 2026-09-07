import unittest
import numpy as np
from LIPM.demos.demo_LIPM_3D_mass_switch import create_model,simulate
from LIPM.demos.demo_LIPM_3D_mass_switch_total_com import create_model as total_model


class TrackingTests(unittest.TestCase):
    def test_commanded_speed_and_step_geometry(self):
        for factory in [create_model,total_model]:
            for foot in [0.,3.,10.]:
                for ss,ds in [(0.48,0.12),(0.6,0.)]:
                    for velocity in [(0.3,0.),(0.3,0.2),(0.,0.)]:
                        with self.subTest(model=factory.__module__,foot=foot,ds=ds,velocity=velocity):
                            m=factory(t_ss=ss,t_ds=ds,body_mass=40.,foot_mass=foot,velocity=velocity,periodic_start=True)
                            theta=np.arctan2(velocity[1],velocity[0])
                            d=simulate(m,4*m.T_d,theta)
                            np.testing.assert_allclose((d['position'][-1]-d['position'][0])/(4*m.T_d),velocity,atol=1e-10)
                            switches=np.flatnonzero(np.diff(d['step_num']))+1
                            np.testing.assert_allclose(d['step_length'][switches],np.linalg.norm(velocity)*m.T_d,atol=1e-10)
                            np.testing.assert_allclose(d['step_width'][switches],0.4,atol=1e-10)

    def test_new_commands_update_next_capture_offset(self):
        m=create_model(periodic_start=True)
        old=m.calculateDesiredDCMOffset()
        m.s_d=0.24; m.w_d=0.6
        expected=m.calculateDesiredDCMOffset()
        self.assertFalse(np.allclose(old,expected))
        simulate(m,m.T_d)
        actual=np.array([m.x_0+m.vx_0/m.w_0,m.y_0+m.vy_0/m.w_0])
        np.testing.assert_allclose(actual,expected,atol=1e-10)


if __name__ == '__main__': unittest.main()
