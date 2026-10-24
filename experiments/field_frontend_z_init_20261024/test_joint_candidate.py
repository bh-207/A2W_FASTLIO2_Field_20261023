import pathlib
import subprocess
import tempfile
import unittest


class JointCandidateTest(unittest.TestCase):
    def test_covariance_domain_mapping_and_rejections(self):
        include=pathlib.Path(__file__).parent/'source/frontend/include'
        code='''#include "joint_candidate_checks.hpp"
#include <cassert>
int main(){
 using namespace frontend_joint;
 Eigen::Matrix3d R=Eigen::Matrix3d::Identity();Eigen::Vector3d g(0,0,-9.809),b=Eigen::Vector3d::Zero();
 Cov18 P=Cov18::Identity()*.001;P.block<3,3>(15,15)*=.001;
 assert(validate(R,g,b,b,P).accepted);
 auto bad=P;bad(0,0)=-.1;assert(!validate(R,g,b,b,bad).accepted);
 bad=P;bad(9,9)=4;assert(!validate(R,g,b,b,bad).accepted);
 assert(!validate(R,g*1.01,b,b,P).accepted);
 R(0,0)=2;assert(!validate(R,g,b,b,P).accepted);
 P(0,9)=P(9,0)=.0001;
 Eigen::Matrix<double,2,3> N;N<<.1,0,0,0,.1,0;
 auto K=map_covariance(P,N);
 assert(std::abs(K(0,21)-.00001)<1e-15);
 assert(std::abs(K(15,15)-P(15,15))<1e-15);
 assert(std::abs(K(18,18)-P(12,12))<1e-15);
 assert(std::abs(K(12,12)-P(6,6))<1e-15);
 assert((K-K.transpose()).norm()<1e-15);
}'''
        with tempfile.TemporaryDirectory() as temp:
            path=pathlib.Path(temp)/'test.cpp';path.write_text(code)
            r=subprocess.run(['g++','-std=c++17','-O0','-I'+str(include),'-I/usr/include/eigen3',str(path),'-o',temp+'/test'],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stderr)
            self.assertEqual(subprocess.run([temp+'/test']).returncode,0)
