"""Numerical checks independent of hardware and measured datasets."""
import unittest
import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from calibrate_hc_register_force import fit, metrics, rotation_to_base

class CalibrationMathTest(unittest.TestCase):
    def test_recovers_cross_axis_mapping_and_validates_unseen_vectors(self):
        a = np.array([[1.2,.2,0],[-.1,.8,.3],[.1,0,1.1]])
        rows = [dict(x=x,y=x@a,trial='synthetic',phase=str(i))
                for i,x in enumerate(np.vstack([np.eye(3)*10,-np.eye(3)*10]))]
        coef=fit(rows)
        test=[dict(x=x,y=x@a) for x in np.array([[5,8,-4],[-7,6,3]])]
        self.assertLess(metrics(test,coef)['vector_rmse_n'],1e-10)
        np.testing.assert_allclose(coef,a,atol=1e-10)
    def test_rejects_unexcited_axes(self):
        with self.assertRaises(ValueError):
            fit([dict(x=[1,0,0],y=[2,0,0],trial='a',phase='x')]*3)
    def test_mount_rotation_preserves_norm_and_expected_axis(self):
        r=rotation_to_base({'axia_sensor_link':('base_link',np.eye(3))})
        np.testing.assert_allclose(r@[1,0,0],[0,1,0],atol=1e-12)
        np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-12)

if __name__=='__main__': unittest.main()
