"""用已知系数和留出扰动验证单指数拟合，不连接设备。"""
import copy
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from fit_rebound_single_exponential import fit_channel, model, run


def known_curves(k=.12,tau=37.):
    out=[]
    for i,T in enumerate([2.,10.,60.]*3,1):
        t=np.arange(600)+.5;D=4000+i*10
        out.append(dict(session='synthetic',trial=i,channel=4,role='fit' if i<=6 else 'validation',
            repeat=(i-1)//3+1,T=T,hold_requested_s=T,D=D,t=t,y=model(t,D,T,k,tau),missing_bins=0,
            metrics={f'residual_{s}s':float(model([s],D,T,k,tau)[0]) for s in (1,5,10,30,60,120,300,600)}))
    return out


class FitTests(unittest.TestCase):
    def test_recovers_known_parameters_across_durations_and_missing_time_bins(self):
        curves=known_curves()[:6]
        curves[0]['t']=curves[0]['t'][::2];curves[0]['y']=curves[0]['y'][::2]
        r=fit_channel(curves)
        self.assertAlmostEqual(r['k'],.12,places=6)
        self.assertAlmostEqual(r['tau_seconds'],37,places=4)
        self.assertLess(r['normalized_mse'],1e-16)
        self.assertFalse(r['boundary'])

    def test_no_positive_residual_does_not_claim_identification(self):
        curves=known_curves()[:6]
        for c in curves: c['y']=np.zeros_like(c['y'])
        r=fit_channel(curves)
        self.assertEqual(r['k'],0)
        self.assertFalse(r['identifiable'])

    def test_validation_changes_do_not_change_fitted_coefficients_or_training_data(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'analysis.json').write_text('{}');(root/'recovery_1s.csv').write_text('test')
            report=dict(source_files=[],sessions=[],trials=[])
            curves=[]
            for channel in (4,5,6,7):
                for c in known_curves(): c['channel']=channel;curves.append(c)
            with contextlib.redirect_stdout(io.StringIO()):
                with patch('fit_rebound_single_exponential.load_curves',return_value=(report,curves,[])):
                    first,result1,_=run(root)
                changed=copy.deepcopy(curves)
                for c in changed:
                    if c['role']=='validation': c['y']+=100
                with patch('fit_rebound_single_exponential.load_curves',return_value=(report,changed,[])):
                    second,result2,_=run(root)
            self.assertNotEqual(first,second)
            self.assertEqual(result1['channels'],result2['channels'])
            self.assertGreater(result2['errors'][6]['rmse_counts'],99)
            self.assertEqual(json.loads((first/'fit.json').read_text())['channels']['4']['validation_trials'],[7,8,9])


if __name__=='__main__': unittest.main()
