import unittest
import csv
import json
import tempfile
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import ground_eval as ge
from pcd_io import read_cloud
import verify_baseline_identity as vi
import route_profile as rp
import native_fixedup_views as nv
import backend_exact_subset as be


class FixedGroundTest(unittest.TestCase):
    def setUp(self):
        self.up = np.array([0., 0., 1.])
        self.basis = np.eye(3)
        self.axis = np.array([0., 1., 0.])
        x, y = np.meshgrid(np.linspace(-.36, .36, 9), np.linspace(-.36, .36, 9))
        self.p = np.c_[x.ravel(), y.ravel(), (.02*x + .04*y + 1.2).ravel()]

    def test_real_ramp_height_and_grade_are_retained(self):
        v = ge.measure_patch(self.p, np.zeros(3), self.up, self.basis, self.axis)
        self.assertEqual(v['status'], 'SUPPORTED')
        self.assertAlmostEqual(v['height_m'], 1.2)
        self.assertAlmostEqual(v['longitudinal_grade_pct'], 4.)
        self.assertGreater(v['normal_angle_to_up_deg'], 2.)

    def test_same_horizontal_position_avoids_centroid_height_bias(self):
        q = self.p + [.1, -.1, -.002]
        a = ge.measure_patch(self.p, np.zeros(3), self.up, self.basis, self.axis)
        b = ge.measure_patch(q, np.zeros(3), self.up, self.basis, self.axis)
        self.assertAlmostEqual(a['height_m'], b['height_m'])

    def test_fixed_up_does_not_remove_candidate_global_tilt(self):
        q = self.p @ Rotation.from_euler('x', 3, degrees=True).as_matrix().T
        b = ge.measure_patch(q, np.zeros(3), self.up, self.basis, self.axis)
        self.assertGreater(abs(b['longitudinal_grade_pct'] - 4), 5)

    def test_missing_and_displaced_support_stay_unknown(self):
        a = ge.measure_patch(self.p[:7], np.zeros(3), self.up, self.basis, self.axis)
        self.assertEqual(a['status'], 'UNKNOWN')
        b = ge.measure_patch(self.p + [1, 0, 0], np.zeros(3), self.up, self.basis, self.axis)
        self.assertEqual(b['status'], 'UNKNOWN')
        self.assertIn('outside', b['reason'])

    def test_profile_uses_fixed_signed_stations_and_keeps_missing_segment(self):
        regions = [dict(region='a', anchor_reference=[0,0,0]),
                   dict(region='b', anchor_reference=[0,50,0]),
                   dict(region='c', anchor_reference=[0,100,0])]
        patches = {'a': dict(status='SUPPORTED',height_m=1.),
                   'b': dict(status='SUPPORTED',height_m=3.),
                   'c': dict(status='UNKNOWN',reason='missing')}
        profile = ge.profile_segments(regions, patches, self.axis)
        self.assertEqual(len(profile), 2)
        self.assertAlmostEqual(profile[0]['height_difference_m'], 2.)
        self.assertAlmostEqual(profile[0]['longitudinal_grade_pct'], 4.)
        self.assertEqual(profile[1]['status'], 'UNKNOWN')

    def test_quantized_timestamp_join_is_unique_and_never_milliseconds(self):
        keys = np.array([1792724421199894905, 1792724421299894980], dtype=np.int64)
        ix, delta = ge.unique_quantized_match(1792724421199894784, keys)
        self.assertEqual(ix, 0)
        self.assertEqual(delta, 121)
        with self.assertRaises(ValueError):
            ge.unique_quantized_match(int(keys[0])+1000000, keys)
        with self.assertRaises(ValueError):
            ge.unique_quantized_match(100, np.array([99,101]))

    def test_missing_point_identity_does_not_compact_the_frozen_selection(self):
        body, valid = ge.match_point_ids(np.array([2, 8, 19]), np.array([19, 2]),
                                        np.array([[19,0,0],[2,0,0]], float))
        self.assertEqual(len(body), 3)
        self.assertEqual(valid.tolist(), [True, False, True])
        self.assertTrue(np.isnan(body[1]).all())
        np.testing.assert_array_equal(body[[0,2],0], [2,19])
        with self.assertRaises(ValueError):
            ge.match_point_ids(np.array([2]), np.array([2,2]), np.zeros((2,3)))

    def test_historical_regrid_preserves_legacy_labels_and_point_identity(self):
        data = {g+'_cell':np.array([[80,80]]) for g in ge.GROUPS}
        data.update({g+'_point_index':np.array([143]) for g in ge.GROUPS})
        world = {g:np.array([[.4,-.4,3.]]) for g in ge.GROUPS}
        result = ge.freeze_baseline_cell_tags(data,world,self.basis)
        np.testing.assert_array_equal(result['train_cell'],[[0,-1]])
        np.testing.assert_array_equal(result['train_legacy_cell'],[[80,80]])
        np.testing.assert_array_equal(result['train_point_index'],[143])

    def test_scan_mapping_retains_unrecorded_sources_instead_of_nearest_scan(self):
        with tempfile.TemporaryDirectory() as temp:
            temp = Path(temp)
            def put(name,values):
                with (temp/name).open('w') as f:
                    w=csv.DictWriter(f,fieldnames=list(values[0]))
                    w.writeheader();w.writerows(values)
                return temp/name
            timing=put('timing.csv',[dict(raw_scan_id=0,header_ns=1000000000),dict(raw_scan_id=1,header_ns=1200000000)])
            index=put('old.csv',[dict(id=0,stamp_ns=1050000000),dict(id=1,stamp_ns=1150000000)])
            trace=put('trace.csv',[dict(stage='POST_LIDAR_UPDATE',scan_begin_ns=1000000128,scan_end_ns=1050000000),dict(stage='POST_LIDAR_UPDATE',scan_begin_ns=1100000000,scan_end_ns=1150000000)])
            mapped=ge.scan_mapping(timing,index,trace)
            self.assertEqual(mapped[0]['source_scan_id'],0)
            self.assertEqual(mapped[1]['status'],'UNMATCHED_SOURCE_NOT_RECORDED')
            self.assertIsNone(mapped[1]['source_scan_id'])

    def test_pose_only_index_is_accepted_for_historical_rigid_retransform(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'poses.csv'
            path.write_text('id,tx,ty,tz,qx,qy,qz,qw\n0,0,0,0,0,0,0,1\n')
            self.assertEqual(len(ge.load_index(path)),1)

    def test_frozen_source_mutation_fails_closed(self):
        with tempfile.TemporaryDirectory() as temp:
            temp=Path(temp)
            source=temp/'input.txt';source.write_text('original')
            manifest=dict(protocol=ge.PROTOCOL,input_sha256={str(source):ge.sha(source)},selection_sha256={})
            ge.write_json(temp/'manifest.json',manifest)
            ge.verify_frozen(temp)
            source.write_text('changed')
            with self.assertRaises(ValueError):ge.verify_frozen(temp)

    def test_existing_freeze_cannot_silently_reuse_another_input_index(self):
        with tempfile.TemporaryDirectory() as temp:
            temp=Path(temp)
            source=temp/'one.csv';source.write_text('first')
            other=temp/'two.csv';other.write_text('second')
            ge.write_json(temp/'manifest.json',dict(protocol=ge.PROTOCOL,input_sha256={str(source):ge.sha(source)},
                selection_sha256={},mode='complete_original_baseline',index_path=str(source)))
            with self.assertRaises(ValueError):
                ge.freeze_regions(temp,'complete_original_baseline',[],None,np.eye(3),[other],dict(index_path=str(other)))

    def test_fixed_structure_measures_normal_separation_without_rematching(self):
        y,z=np.meshgrid(np.linspace(-1,1,21),np.linspace(-1,1,21))
        train=np.c_[np.zeros(y.size),y.ravel(),z.ravel()]
        query=train[::7]+[.03,0,0]
        old=ge.fixed_structure(train,query,self.up)
        new=ge.fixed_structure(train,query+[.1,0,0],self.up,old['neighbors'])
        self.assertGreater(old['support'].sum(),25)
        self.assertAlmostEqual(np.median(new['distance'][old['support']]),.13)
        np.testing.assert_array_equal(new['neighbors'],old['neighbors'])

    def test_seam_protection_never_hides_missing_frozen_cells(self):
        baseline=[dict(return_minus_train_m=.01),dict(return_minus_train_m=.02)]
        candidate=[dict(return_minus_train_m=.01),dict(status='UNKNOWN')]
        self.assertEqual(ge.seam_protection(baseline,candidate)['status'],'UNKNOWN')
        candidate=[dict(return_minus_train_m=.20),dict(return_minus_train_m=.20)]
        self.assertEqual(ge.seam_protection(baseline,candidate)['status'],'FAIL')

    def test_sorted_sidecar_point_ids_are_complete_and_match_by_identity(self):
        with tempfile.TemporaryDirectory() as temp:
            temp=Path(temp)
            cloud=temp/'scan.npz';sidecar=temp/'point_ids.bin'
            np.savez(cloud,body=np.array([[5.,0,0],[3.,0,0],[4.,0,0]]),point_id=np.arange(3))
            np.array([2,0,1],dtype='<u4').tofile(sidecar)
            body,ids=read_cloud(cloud,sidecar)
            self.assertEqual(ids.tolist(),[2,0,1])
            reordered,valid=ge.match_point_ids(np.arange(3),ids,body)
            self.assertTrue(valid.all())
            np.testing.assert_array_equal(reordered[:,0],[3,4,5])
            np.array([2,0,0],dtype='<u4').tofile(sidecar)
            with self.assertRaises(ValueError):read_cloud(cloud,sidecar)

    def test_preprocess_fingerprint_change_is_rejected(self):
        baseline={3:dict(header_ns=100,point_count=81,fingerprint='ABCD')}
        current={3:dict(header_ns=100,point_count=81,fingerprint='ABCD')}
        ge.verify_preprocess_identity(3,baseline,current)
        current[3]['fingerprint']='FFFF'
        with self.assertRaises(ValueError):ge.verify_preprocess_identity(3,baseline,current)

    def test_complete_modern_adapter_freezes_before_candidate_and_recovers_height_change(self):
        with tempfile.TemporaryDirectory() as temp:
            temp=Path(temp)
            contract=dict(schema='field_ground_identity_contract_v1',point_identity='stable_preprocess_record_index',
                          stable_across_runs=True,implementation_evidence='Synthetic fixture with explicit stable IDs',
                          preprocessing_identity_sha256='synthetic_fixture_only')
            ge.write_json(temp/'contract.json',contract)
            base_rows=[];pre_rows=[];cloud_meta={}
            definitions=[dict(name='corridor_A',center=[0,0,1.2]),dict(name='corridor_B',center=[0,50,3.2])]
            data_world={}
            for i,reg in enumerate(definitions):
                data={};world={}
                for j,g in enumerate(ge.GROUPS):
                    sid=3*i+j
                    cloud=temp/('base_'+str(sid)+'.npz')
                    np.savez(cloud,body=self.p,point_id=np.arange(len(self.p)))
                    row=dict(source_scan_id=sid,source_header_stamp_ns=10000+sid,
                             cloud=str(cloud),tx=0,ty=50*i,tz=2*i,qx=0,qy=0,qz=0,qw=1)
                    base_rows.append(row)
                    pre_rows.append(dict(source_scan_id=sid,header_ns=10000+sid,point_count=len(self.p),
                                         xyz_intensity_curvature_fnv1a64='fixed_'+str(sid)))
                    q=ge.transform(self.p,row)
                    data.update({g+'_body':self.p.copy(),g+'_scan_id':np.full(len(self.p),sid),
                                 g+'_point_index':np.arange(len(self.p)),g+'_floor':np.ones(len(self.p),bool),
                                 g+'_cell':np.floor(q[:,:2]/.75).astype(np.int64)})
                    world[g]=q
                    cloud_meta[str(cloud)]=dict(source_scan_id=sid,source_header_stamp_ns=10000+sid,
                                               point_count=len(self.p),sha256=ge.sha(cloud))
                data_world[reg['name']]=(data,world)
            def csvfile(name,rs):
                path=temp/name
                with path.open('w') as f:
                    w=csv.DictWriter(f,fieldnames=list(rs[0]));w.writeheader();w.writerows(rs)
                return path
            baseline=csvfile('baseline.csv',base_rows)
            pre=csvfile('pre.csv',pre_rows)
            frozen=temp/'frozen'
            ge.freeze_regions(frozen,'complete_original_baseline',definitions,
                              lambda r:data_world[r['name']],self.basis,[baseline,pre,temp/'contract.json'],
                              dict(index_path=str(baseline),identity_contract=contract,
                                   selected_clouds=cloud_meta,preprocess_identity=ge.load_preprocess(pre)))
            candidate_rows=[]
            for row in base_rows:
                row=dict(row)
                cloud=temp/('candidate_'+str(row['source_scan_id'])+'.npz')
                np.savez(cloud,body=self.p+[0,0,.1],point_id=np.arange(len(self.p)))
                row['cloud']=str(cloud);candidate_rows.append(row)
            candidate=csvfile('candidate.csv',candidate_rows)
            report=ge.evaluate(frozen,candidate,temp/'evaluated',temp/'contract.json',pre)
            actual=report['longitudinal_profiles']['train']['anchors'][0]['height_m']
            expected=report['baseline_longitudinal_profiles']['train']['anchors'][0]['height_m']
            self.assertAlmostEqual(actual-expected,.1)
            self.assertEqual(report['absolute_accuracy'],'UNKNOWN_NO_INDEPENDENT_MEASUREMENT')
            self.assertTrue((temp/'evaluated/ground_profile.png').exists())

    def test_runtime_fnv_matches_known_byte_vectors(self):
        self.assertEqual(vi.fnv1a64(b''),14695981039346656037)
        self.assertEqual(vi.fnv1a64(b'a'),0xaf63dc4c8601ec8c)

    def test_identity_function_hash_extraction_ignores_other_functions(self):
        text='void first() { if (true) { return; } }\nvoid second() {}'
        self.assertEqual(vi.function_text(text,'void first'),'void first() { if (true) { return; } }')

    def test_route_profile_uses_frozen_noncollinear_segment_distance_and_keeps_unknown(self):
        route=dict(fixed_up=[0,0,1],anchors=[dict(region='entry',route_station_m=0),dict(region='near',route_station_m=10),dict(region='far',route_station_m=30)],segments=[dict(from_region='entry',to_region='near',fixed_horizontal_distance_m=10,fixed_horizontal_direction=[1,0,0]),dict(from_region='near',to_region='far',fixed_horizontal_distance_m=20,fixed_horizontal_direction=[0,1,0])])
        report=dict(regions=[dict(region=name,groups={g:dict(anchor=dict(status='SUPPORTED',height_m=h,normal=[-.1,-.1,1])) for g in ge.GROUPS}) for name,h in [('entry',1.),('near',2.),('far',4.)]])
        result=rp.measured(route,report)
        self.assertAlmostEqual(result['train']['segments'][0]['signed_segment_grade_pct'],10.)
        self.assertAlmostEqual(result['train']['segments'][1]['signed_segment_grade_pct'],10.)
        report['regions'][1]['groups']['train']['anchor']=dict(status='UNKNOWN')
        self.assertTrue(all(s['status']=='UNKNOWN' for s in rp.measured(route,report)['train']['segments']))

    def test_native_and_fixed_up_views_use_the_same_frozen_plane_intersection(self):
        up=np.array([0,np.sin(.1),np.cos(.1)])
        refs=[np.zeros(3),np.array([0,10,-10*np.tan(.1)])]
        route=dict(fixed_up=up.tolist(),anchors=[dict(region=n,route_station_m=i*10,fixed_reference=refs[i].tolist()) for i,n in enumerate(['a','b'])],segments=[dict(fixed_horizontal_distance_m=np.linalg.norm(refs[1]))])
        report=dict(regions=[dict(region=n,groups={g:dict(anchor=dict(status='SUPPORTED',height_m=i+1.,normal=up.tolist())) for g in ge.GROUPS}) for i,n in enumerate(['a','b'])])
        locations=dict(locations=[dict(region=n,group=g,baseline_ground_xyz=(refs[i]+up*(i+1.)).tolist()) for i,n in enumerate(['a','b']) for g in ge.GROUPS])
        result=nv.coordinate_profile(route,report,locations)['train']['segments'][0]
        self.assertAlmostEqual(result['fixed_up_height_difference_m'],1.)
        self.assertAlmostEqual(result['native_z_height_difference_m'],up[2]+refs[1][2])
        self.assertAlmostEqual(result['native_z_reference_projection_term_m'],refs[1][2])

    def test_backend_adapter_keeps_missing_exact_nodes_unknown_without_interpolation(self):
        with tempfile.TemporaryDirectory() as temp:
            temp=Path(temp);run=temp/'run';bundle=temp/'bundle';frozen=temp/'frozen'
            (run/'reconstruction').mkdir(parents=True);(bundle/'keyframes').mkdir(parents=True);frozen.mkdir()
            post=[dict(source_scan_id=s,source_header_stamp_ns=t,scan_begin_ns=t,scan_end_ns=t+100,cloud='dummy.pcd',point_ids='dummy.ids',tx=0,ty=0,tz=0,qx=0,qy=0,qz=0,qw=1) for s,t in [(10,1000),(20,2000)]]
            def write(path,records):
                with path.open('w') as f:
                    w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
            write(run/'reconstruction/post_index.csv',post)
            raw=dict(id=0,stamp_ns=1100,cloud='dummy.pcd',tx=0,ty=0,tz=0,qx=0,qy=0,qz=0,qw=1)
            write(bundle/'keyframes/index.csv',[raw]);write(bundle/'optimized_poses.csv',[{k:v for k,v in raw.items() if k not in ['stamp_ns','cloud']}]);(bundle/'metadata.yaml').write_text('map_frame: map\n')
            ge.write_json(frozen/'manifest.json',dict(protocol=ge.PROTOCOL,input_sha256={},selection_sha256={},selected_clouds={str(s):dict(source_scan_id=s) for s in [10,20]},regions=[dict(region='one',groups={g:dict(scan_ids=[10,20]) for g in ge.GROUPS})]))
            result=be.build(run,bundle,frozen,temp/'output')
            self.assertEqual(result['exact_frozen_source_scan_count'],1)
            self.assertEqual(result['missing_frozen_source_scan_ids'],[20])
            self.assertEqual(result['interpolated_poses'],0)
            self.assertEqual(result['regions'][0]['groups']['train']['status'],'UNKNOWN_INCOMPLETE_FROZEN_NODE_SUPPORT')


if __name__ == '__main__':
    unittest.main()
