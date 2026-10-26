"""Fixed-target and rotation-invariance contracts, not factory ground truth."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

PATH = Path(__file__).with_name("loop_layer_geometry.py")
layer = None
if PATH.exists():
    spec = importlib.util.spec_from_file_location("loop_layer_geometry", PATH)
    layer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(layer)


class LayerGeometryTest(unittest.TestCase):
    def impl(self):
        self.assertIsNotNone(layer, "layer evaluator implementation is absent")
        return layer

    def inputs(self):
        a=np.array([[0.,0,0],[1,0,1],[2,0,4]])
        b=a+np.array([.1,0,.2])
        normals=np.tile([0.,0,1],(3,1))
        return dict(a=a,b=b,na=normals,nb=normals,valid_a=np.ones(3,bool),
                    valid_b=np.ones(3,bool),ha=np.array([0.,1,4]),hb=np.array([0.,1,4]),
                    ab=np.arange(3),ba=np.arange(3))

    def test_global_rigid_rotation_does_not_improve_distances_or_normal_angles(self):
        impl=self.impl()
        data=self.inputs()
        before=impl.layer_metrics(**data)
        q=Rotation.from_euler("xyz",[.4,-.3,.8])
        data.update(a=q.apply(data["a"])+[4,-2,7],b=q.apply(data["b"])+[4,-2,7],
                    na=q.apply(data["na"]),nb=q.apply(data["nb"]))
        after=impl.layer_metrics(**data)
        for first,second in zip(before,after):
            for key in ("fixed_match_p95_m","nn_p95_m","point_to_plane_p95_m","normal_angle_p95_deg"):
                self.assertAlmostEqual(first[key],second[key],places=7)

    def test_far_points_remain_in_fixed_denominator_and_raise_max_error(self):
        impl=self.impl()
        data=self.inputs()
        data["a"][2]+=100
        all_points=impl.layer_metrics(**data)[0]
        self.assertEqual(all_points["directed_points"],6)
        self.assertEqual(all_points["fixed_correspondences"],6)
        self.assertGreater(all_points["fixed_match_max_m"],100)
        self.assertLess(all_points["fixed_coverage_under_0_5m"],1)

    def test_proxy_layers_partition_all_points_without_changing_frozen_pairs(self):
        impl=self.impl()
        data=self.inputs()
        result=impl.layer_metrics(**data)
        self.assertEqual(sum(r["directed_points"] for r in result[1:]),result[0]["directed_points"])
        self.assertEqual([r["directed_points"] for r in result],[6,2,2,2])
        np.testing.assert_array_equal(data["ab"],np.arange(3))

    def test_ground_proxy_retains_slope_and_rejects_unreliable_reference(self):
        impl=self.impl()
        points=np.array([[0.,0,-1],[2,0,-.8],[4,0,3.4]])
        normal=np.array([-.1,0,1]);normal/=np.linalg.norm(normal)
        observation=dict(reliable=True,normal=normal.tolist(),d=1/np.sqrt(1.01))
        original=points.copy()
        heights=impl.local_ground_heights(points,observation)
        np.testing.assert_allclose(heights[:2],0,atol=1e-12)
        np.testing.assert_array_equal(points,original)
        self.assertGreater(heights[2],3)
        with self.assertRaises(ValueError):
            impl.local_ground_heights(points,{**observation,"reliable":False})

    def test_ground_plane_pair_metrics_are_global_rotation_invariant(self):
        impl=self.impl()
        poses=np.zeros((2,7));poses[:,6]=1;poses[1,2]=.2
        pairs=[dict(i=0,j=1,na=[0,0,1],nb=[0,0,1],da=1,db=1)]
        anchors=np.array([[[0.,0,-1],[0,0,-1]]])
        before=impl.ground_invariant_rows(poses,pairs,anchors)
        q=Rotation.from_euler("xyz",[.3,.2,.8])
        poses[:,:3]=q.apply(poses[:,:3])+[3,2,1]
        poses[:,3:]=q.as_quat()
        after=impl.ground_invariant_rows(poses,pairs,anchors)
        np.testing.assert_allclose(before[0][2:],after[0][2:],atol=1e-12)


if __name__=="__main__":
    unittest.main()
