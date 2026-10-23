import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from geom_math import fit_plane, height_at
from evaluate import wall_eval, common_views, poses


class SlopeGeometryTest(unittest.TestCase):
    def setUp(self):
        x, y = np.meshgrid(np.linspace(-.32,.32,8),np.linspace(-.32,.32,8))
        self.p = np.c_[x.ravel(), y.ravel(), (.2*x-.1*y+2).ravel()]
        self.up = np.array([0.,0.,1.])

    def test_preserves_true_slope(self):
        f = fit_plane(self.p, self.up)
        self.assertIsNotNone(f)
        self.assertAlmostEqual(height_at(f, np.array([.1,.2,0.]), self.up),2.,places=10)
        self.assertAlmostEqual(abs(f['normal'][0]/f['normal'][2]),.2,places=10)

    def test_equal_horizontal_reference_ignores_centroid_displacement(self):
        q = self.p + np.array([.15,-.1,.04])
        # Same physical plane: 0.2*dx-0.1*dy = .04.
        f, g = fit_plane(self.p,self.up),fit_plane(q,self.up)
        ref=np.array([.05,-.1,0.])
        self.assertAlmostEqual(height_at(f,ref,self.up)-height_at(g,ref,self.up),0.,places=10)

    def test_vertical_offset_recovered_in_tilted_global_gauge(self):
        r=Rotation.from_euler('xy',[9.,-7.],degrees=True).as_matrix()
        up=r@self.up; p=self.p@r.T; q=p+.13*up
        f,g=fit_plane(p,up),fit_plane(q,up);ref=r@np.array([.1,.2,0.])
        self.assertAlmostEqual(height_at(g,ref,up)-height_at(f,ref,up),.13,places=10)

    def test_rejects_steep_plane_and_low_support(self):
        p=self.p.copy();p[:,2]=2*p[:,0]
        self.assertIsNone(fit_plane(p,self.up))
        self.assertIsNone(fit_plane(self.p[:7],self.up))

    def test_wall_distance_keeps_query_identity_and_measures_normal_shift(self):
        y,z=np.meshgrid(np.linspace(-1.,1.,21),np.linspace(-1.,1.,21))
        train=np.c_[np.zeros(y.size),y.ravel(),z.ravel()]
        query=train[::7].copy();query[:,0]=.03
        w=wall_eval(train,query,self.up)
        self.assertEqual(w['query_count'],len(query))
        self.assertGreater(w['planar_wall_supported_count'],25)
        self.assertAlmostEqual(w['point_to_local_plane_abs_m']['median'],.03,places=10)

    def test_pose_reader_rejects_missing_or_duplicate_ids(self):
        row=dict(id='0',tx='0',ty='0',tz='0',qx='0',qy='0',qz='0',qw='1')
        with self.assertRaises(ValueError):poses([row,row])
        with self.assertRaises(ValueError):poses([row],expected=[0,1])


if __name__=='__main__':unittest.main()
