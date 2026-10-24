// Actual Process/covariance/deskew boundary regression. Synthetic evidence only.
#include <rclcpp/rclcpp.hpp>
#include "IMU_Processing.hpp"
#include <filesystem>
void observe(state_ikfom&,esekfom::dyn_share_datastruct<double>&d){d.valid=false;}
void check(bool x,const char*s){if(!x)throw std::runtime_error(s);}
int main(int argc,char**argv){try{
 check(argc==4,"candidate output mode");std::filesystem::path root(argv[2]);check(!std::filesystem::exists(root),"fresh test output");std::filesystem::create_directories(root);
 frontend_init_v4::GateConfig gc;gc.enabled=true;frontend_init_v4::InputConfig ic;ic.enabled=true;ic.gyro_unit="rad/s";ic.accel_unit="m/s^2";ic.frame_id="imu";ic.source_provenance="synthetic";ic.gyro_evidence=ic.accel_evidence=ic.time_evidence="synthetic test";
 ImuProcess process;process.set_frontend_init_v4(gc,ic,(root/"quality.csv").string());process.set_extrinsic(V3D(-.00862,.00914,.03916),Eye3d);
 process.set_acc_cov(V3D(.1,.1,.1));process.set_gyr_cov(V3D(.1,.1,.1));process.set_acc_bias_cov(V3D(.0001,.0001,.0001));process.set_gyr_bias_cov(V3D(.0001,.0001,.0001));process.configure_joint(argv[1],(root/"receipt.json").string());
 esekfom::esekf<state_ikfom,12,input_ikfom> kf;double eps[23];std::fill(eps,eps+23,.001);kf.init_dyn_share(get_f,df_dx,df_dw,observe,4,eps);
 auto imu=[](std::int64_t t){auto m=std::make_shared<sensor_msgs::msg::Imu>();m->header.stamp.sec=t/1000000000LL;m->header.stamp.nanosec=t%1000000000LL;m->header.frame_id="imu";m->linear_acceleration.z=9.81;m->angular_velocity.x=.001;m->angular_velocity.y=-.002;m->angular_velocity.z=.003;return m;};
 std::size_t emitted=0;
 for(int scan=0;scan<18;++scan){const auto begin=1000000000000LL+scan*100000000LL;const auto end=begin+99900000LL;MeasureGroup m;m.lidar_beg_time=begin*1e-9;m.lidar_end_time=end*1e-9;m.lidar=std::make_shared<PointCloudXYZI>();PointType a{},b{};a.x=b.x=2;b.curvature=99.9;m.lidar->push_back(a);m.lidar->push_back(b);
 for(int j=0;j<20;++j)m.imu.push_back(imu(begin+j*5000000LL));
 if(scan==15&&std::string(argv[3])=="exact")m.imu.push_back(imu(end));
 auto future=imu(begin+100000000LL);process.joint_context(scan,begin,scan==15&&std::string(argv[3])=="exact"?nullptr:future);
 auto cloud=std::make_shared<PointCloudXYZI>();process.Process(m,kf,cloud);
 if(scan<15)check(!process.frontend_init_v4_initialized(),"early batch commit");
 if(scan==15){check(process.joint_committed(),"no actual joint commit");check(cloud->empty(),"commit scan reused as independent ordinary update");auto x=kf.get_x();check((x.ba-V3D(.03,.04,-.02)).norm()<1e-12,"ba not committed");check((x.pos-V3D(.2,-.1,.4)).norm()<1e-12,"pose not committed");check((x.offset_T_L_I-V3D(-.00862,.00914,.03916)).norm()<1e-12,"extrinsic changed");check(std::abs(process.acceleration_scale()-1)<1e-12,"scale changed");Eigen::LDLT<Eigen::Matrix<double,23,23>> factor(kf.get_P());check(factor.vectorD().minCoeff()>0,"mapped covariance not PD");}
 if(scan>15){check(cloud->size()==2,"first real deskew missing");for(auto&p:*cloud)check(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z),"nonfinite deskew");++emitted;check((kf.get_x().ba-V3D(.03,.04,-.02)).norm()<1e-12,"legacy init overwrote joint ba");}
 }
 check(emitted==2,"deskew count");std::ofstream(root/"PASS.json")<<"{\"actual_joint_commit\":true,\"actual_deskewed\":2,\"synthetic_only\":true}\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 2;}}
