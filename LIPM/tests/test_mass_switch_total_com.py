# Allow both direct script execution and python -m from the repository root.
if __package__ in (None, ''):
    import sys
    from pathlib import Path as _Path
    sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

import unittest
import numpy as np
from LIPM.demos.demo_LIPM_3D_mass_switch_total_com import create_model,simulate
from LIPM.demo_utils.playback import realtime_frames
from LIPM.demos.demo_LIPM_3D_mass_switch import create_model as body_model,simulate as body_simulate
from LIPM.demos.demo_LIPM_3D_double_support import create_model as baseline,simulate as baseline_simulate


class TotalCoMTests(unittest.TestCase):
    def test_realtime_playback_skips_delayed_frames_and_includes_end(self):
        clock_values = iter([100., 100.01, 100.26, 101.5])
        times = np.array([0., 0.1, 0.2, 0.3, 1.])
        frames = list(realtime_frames(times, clock=lambda: next(clock_values)))
        self.assertEqual(frames, [0, 0, 2, 4])

    def test_realtime_playback_handles_time_offset_and_single_frame(self):
        clock_values = iter([20., 20.25, 21.])
        frames = list(realtime_frames(np.array([5., 5.2, 5.5]),
                                      clock=lambda: next(clock_values)))
        self.assertEqual(frames, [0, 1, 2])
        self.assertEqual(list(realtime_frames(np.array([0.]), clock=lambda: 0.)), [0])

    def test_initial_height_and_velocity(self):
        m=create_model()
        d=m.snapshot()
        np.testing.assert_allclose(d['whole_com'],[0,0,0.6])
        np.testing.assert_allclose(d['velocity'],[0.3,0])
        self.assertAlmostEqual(m.body_height,0.9)
        self.assertAlmostEqual(m.zc,0.72)
        d=simulate(m,1.2)
        np.testing.assert_allclose(d['body_position'][:,2],0.9)
        np.testing.assert_allclose(d['planning_height'],0.72)
        np.testing.assert_allclose(d['whole_com'][:,2],0.6+d['feet'][:,:,2].sum(axis=1)/6)
        self.assertGreater(d['whole_com'][:,2].max(),0.6)

    def test_same_physics_as_body_variant_with_matched_state(self):
        m=create_model(periodic_start=True)
        b=body_model(height=0.9,periodic_start=True)
        a,c=simulate(m,2.4),body_simulate(b,2.4)
        for key in ['whole_com','whole_com_velocity','body_position','body_velocity','effective_com','planning_com','feet','target']:
            np.testing.assert_allclose(a[key],c[key],atol=1e-10)
        np.testing.assert_allclose(a['position'],a['whole_com'][:,:2])
        np.testing.assert_allclose(a['velocity'],a['whole_com_velocity'][:,:2])

    def test_periodic_and_zero_mass(self):
        for ss,ds in [(0.48,0.12),(0.6,0.)]:
            for mass in [0.,10.]:
                m=create_model(t_ss=ss,t_ds=ds,foot_mass=mass,periodic_start=True)
                expected=m.periodic_velocity*2*m.T_d
                d=simulate(m,4*m.T_d)
                n=round(2*m.T_d/m.dt)
                delta=d['whole_com'][n:]-d['whole_com'][:-n]
                np.testing.assert_allclose(delta,np.broadcast_to(np.r_[expected,0],delta.shape),atol=1e-9)
                np.testing.assert_allclose(d['velocity'][n:],d['velocity'][:-n],atol=1e-9)
                if mass==0:
                    b=baseline(t_ss=ss,t_ds=ds,velocity=(0.3,0),periodic_start=True)
                    old=baseline_simulate(b,4*m.T_d)
                    np.testing.assert_allclose(d['position'],old['position'],atol=1e-9)


if __name__ == '__main__': unittest.main()
