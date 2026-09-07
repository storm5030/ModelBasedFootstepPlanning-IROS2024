# Allow both direct script execution and python -m from the repository root.
if __package__ in (None, ''):
    import sys
    from pathlib import Path as _Path
    sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

import unittest
import numpy as np
from LIPM.demos.demo_LIPM_3D_double_support import create_model, simulate
from LIPM.analysis.demo_LIPM_3D_double_support_analysis import analyze

class AnalysisTests(unittest.TestCase):
    def test_forecasts_match_actual_endpoints(self):
        for ss,ds in [(0.48,0.12),(0.6,0.)]:
            for periodic in [False,True]:
                m=create_model(t_ss=ss,t_ds=ds,velocity=(0.3,0),periodic_start=periodic)
                d=simulate(m,1.8)
                a=analyze(d,0.6,ss,ds)
                self.assertLess(np.abs(a['dcm_residual']).max(),1e-11)
                for i in range(len(d['time'])):
                    n=int(d['step_num'][i])
                    stop=round((n+1)*(ss+ds)/m.dt)
                    if stop<len(d['time']):
                        np.testing.assert_allclose(a['predicted_step_end_icp'][i],a['icp'][stop],atol=1e-10)
                    stop_ss=round((n*(ss+ds)+ss)/m.dt)
                    if d['phase'][i]=='SSP' and stop_ss<len(d['time']):
                        np.testing.assert_allclose(a['predicted_ss_end_icp'][i],a['icp'][stop_ss],atol=1e-10)

if __name__ == '__main__':
    unittest.main()
