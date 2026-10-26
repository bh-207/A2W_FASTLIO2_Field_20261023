// Actual isolated FAST-LIO initialization/ESKF/deskew. No rclcpp::init,
// node, transport, executor, publisher, registration or robot connection.
#include <rclcpp/rclcpp.hpp>
#include "IMU_Processing.hpp"
#include "preprocess.h"
#include <filesystem>
#include <sstream>
#include <iomanip>

namespace fs=std::filesystem;
void no_measurement(state_ikfom &,esekfom::dyn_share_datastruct<double> &d){d.valid=false;}
void require(bool ok,const std::string &reason){if(!ok)throw std::runtime_error(reason);}
std::vector<double> values(std::string line){for(char &c:line)if(c==',')c=' ';std::istringstream s(line);std::vector<double> out;double v;while(s>>v)out.push_back(v);return out;}
sensor_msgs::msg::Imu::SharedPtr message(const frontend_init_v4::Sample &s){
  auto m=std::make_shared<sensor_msgs::msg::Imu>();m->header.frame_id=s.frame_id;
  m->header.stamp.sec=s.stamp_ns/1000000000LL;m->header.stamp.nanosec=s.stamp_ns%1000000000LL;
  m->linear_acceleration.x=s.acc[0];m->linear_acceleration.y=s.acc[1];m->linear_acceleration.z=s.acc[2];
  m->angular_velocity.x=s.gyro[0];m->angular_velocity.y=s.gyro[1];m->angular_velocity.z=s.gyro[2];return m;
}

int main(int argc,char **argv){try{
  require(argc==3||argc==4,"CASE OUTPUT [FROZEN_FIXTURE]");const std::string name=argv[1];const fs::path out=argv[2];
  require(!fs::exists(out),"output exists; refuse overwrite");fs::create_directories(out);
  const bool frozen=name=="frozen_terrain",enabled=name!="off";
  const bool bad_input=name=="late_bad_input_nan"||name=="late_bad_input_stamp";
  const bool buffered_backtrack=name=="positive_backtrack_buffered"||name=="positive_backtrack_legacy_drop";
  require(frozen||bad_input||buffered_backtrack||name=="tilted_static"||name=="moving_then_static"||name=="moving_never_static"||name=="late_bad_group"||name=="off","unknown case");
  frontend_init_v4::InputConfig ic;ic.enabled=true;ic.gyro_unit="rad/s";ic.accel_unit="m/s^2";ic.frame_id="imu";
  ic.gyro_evidence=ic.accel_evidence=ic.time_evidence="explicit synthetic generator / frozen fixture manifest";
  ic.source_provenance="synthetic";
  frontend_init_v4::InputAdapter adapter(ic);frontend_init_v4::GateConfig gc;gc.enabled=enabled;
  ImuProcess process;process.set_frontend_init_v4(gc,ic,(out/"init_quality.csv").string());
  V3D ext(.06,-.02,.04);M3D er=Eye3d;
  if(frozen){require(argc==4,"frozen fixture required");std::ifstream f(fs::path(argv[3])/"extrinsic.txt");require(bool(f),"extrinsic missing");for(int i=0;i<3;++i)for(int j=0;j<3;++j)f>>er(i,j);for(int i=0;i<3;++i)f>>ext[i];require(bool(f),"extrinsic malformed");}
  process.set_extrinsic(ext,er);process.set_acc_cov(V3D(.1,.1,.1));process.set_gyr_cov(V3D(.1,.1,.1));
  process.set_acc_bias_cov(V3D(.0001,.0001,.0001));process.set_gyr_bias_cov(V3D(.0001,.0001,.0001));
  esekfom::esekf<state_ikfom,12,input_ikfom> filter;double eps[23];std::fill(eps,eps+23,.001);filter.init_dyn_share(get_f,df_dx,df_dw,no_measurement,4,eps);
  std::vector<frontend_init_v4::Sample> samples;
  if(frozen){std::ifstream f(fs::path(argv[3])/"imu.csv");require(bool(f),"frozen imu missing");std::string line;std::getline(f,line);
    while(std::getline(f,line)){auto v=values(line);require(v.size()==7,"imu CSV width");frontend_init_v4::Sample s;s.stamp_ns=std::llround(v[0]*1e9);s.acc={v[1],v[2],v[3]};s.gyro={v[4],v[5],v[6]};samples.push_back(s);}}
  else for(int i=0;i<800;++i){frontend_init_v4::Sample s;s.stamp_ns=1000000000000LL+i*5000000LL;s.acc={0,4.905,8.495709211};s.gyro={.001,-.002,.003};
    if(name=="off"||name=="moving_never_static"||(name=="moving_then_static"&&i<160)||(name=="late_bad_group"&&i>=219)||((bad_input||buffered_backtrack)&&i>219))s.gyro={1,0,0};
    if(i==219&&name=="late_bad_input_nan")s.acc[2]=std::numeric_limits<double>::quiet_NaN();
    if(i==219&&name=="late_bad_input_stamp")s.stamp_ns=-1;samples.push_back(s);}
  if(buffered_backtrack)samples[201].stamp_ns=1000999000000LL; // raw1.000 then.999; Process previously saw only.995.
  for(auto &s:samples){s.frame_id="imu";s.gyro_unit="rad/s";s.accel_unit="m/s^2";}
  std::ofstream states(out/"actual_states.csv");states<<std::setprecision(17)<<"scan,scan_end_s,initialized,points,gx,gy,gz,bgx,bgy,bgz,bax,bay,baz,px,py,pz\n";
  std::ofstream groups(out/"synchronized_groups.csv");groups<<"scan,begin_ns,end_ns,input_count,first_imu_ns,last_imu_ns\n";
  std::size_t cursor=0,deskewed=0;int first=-1;double first_end=-1;std::size_t accepted=0;std::int64_t window_first=-1;bool consistency=true;
  Preprocess preprocess;preprocess.feature_enabled=false;preprocess.lidar_type=JT128;preprocess.N_SCANS=128;preprocess.time_unit=SEC;preprocess.point_filter_num=6;preprocess.blind=.5;
  const int scans=frozen?30:40;
  std::int64_t last_callback_stamp=-1;
  for(int scan=0;scan<scans;++scan){MeasureGroup group;group.lidar_beg_time=1000.+scan*.1;group.lidar=std::make_shared<PointCloudXYZI>();
    if(frozen){std::ostringstream filename;filename<<"scan_"<<std::setw(4)<<std::setfill('0')<<scan<<".csv";std::ifstream f(fs::path(argv[3])/filename.str());require(bool(f),"frozen scan missing");std::string line;std::getline(f,line);pcl::PointCloud<hesai_point_type::Point> cloud;
      while(std::getline(f,line)){auto v=values(line);require(v.size()==6,"point CSV width");hesai_point_type::Point p{};p.x=v[0];p.y=v[1];p.z=v[2];p.intensity=v[3];p.ring=v[4];p.timestamp=v[5];cloud.push_back(p);}
      auto msg=std::make_unique<sensor_msgs::msg::PointCloud2>();pcl::toROSMsg(cloud,*msg);preprocess.process(msg,group.lidar);require(group.lidar->size()>1,"preprocess no timed points");
    }else{PointType a{},b{};a.x=b.x=2;a.curvature=0;b.curvature=99.9;group.lidar->push_back(a);group.lidar->push_back(b);}
    group.lidar_end_time=group.lidar_beg_time+group.lidar->back().curvature/1000.;
    const auto end_ns=std::llround(group.lidar_end_time*1e9);
    while(cursor<samples.size()&&samples[cursor].stamp_ns<=end_ns){
      try{const auto si=adapter.adapt(samples[cursor]);
        // Same pre-Process buffer-clear semantics as the real callback. The
        // explicit legacy-drop case is a retained unsafe RED control only.
        if(si.stamp_ns<last_callback_stamp){group.imu.clear();if(name!="positive_backtrack_legacy_drop")process.notify_frontend_input_rejection_v4();}
        last_callback_stamp=si.stamp_ns;group.imu.push_back(message(si));}
      catch(const std::invalid_argument &){process.notify_frontend_input_rejection_v4();}
      ++cursor;
    }
    require(!group.imu.empty(),"empty synchronized group");
    groups<<scan<<','<<std::llround(group.lidar_beg_time*1e9)<<','<<end_ns<<','<<group.imu.size()<<','
      <<rclcpp::Time(group.imu.front()->header.stamp).nanoseconds()<<','<<rclcpp::Time(group.imu.back()->header.stamp).nanoseconds()<<'\n';
    auto cloud=std::make_shared<PointCloudXYZI>();process.Process(group,filter,cloud);const auto s=filter.get_x();const bool init=process.frontend_init_v4_initialized();
    if(init&&first<0){first=scan;first_end=group.lidar_end_time;const auto &st=process.frontend_init_v4_stats();accepted=st.count;window_first=st.first_stamp_ns;
      if(enabled){const V3D a(st.mean_acc[0],st.mean_acc[1],st.mean_acc[2]),g(st.mean_gyro[0],st.mean_gyro[1],st.mean_gyro[2]);
        consistency=V3D(s.grav).cross(-a).norm()<1e-8&&(s.bg-g).norm()<1e-12&&(s.rot.toRotationMatrix()-Eye3d).norm()<1e-12&&(s.offset_T_L_I-ext).norm()<1e-12&&(s.offset_R_L_I.toRotationMatrix()-er).norm()<1e-12;}}
    if(!cloud->empty()){require(init,"deskew before gate commit");++deskewed;for(const auto &p:*cloud)require(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z),"nonfinite deskew");}
    states<<scan<<','<<group.lidar_end_time<<','<<init<<','<<cloud->size();
    for(int k=0;k<3;++k)states<<','<<s.grav[k];for(const auto &v:std::array<V3D,3>{V3D(s.bg),V3D(s.ba),V3D(s.pos)})for(int k=0;k<3;++k)states<<','<<v[k];states<<'\n';
    if(name=="off")break; // Legacy positive control: one real initialization group, no unsupported post-init assumptions.
  }
  const bool expected_ready=name=="tilted_static"||name=="moving_then_static"||name=="off"||frozen;
  require((first>=0)==expected_ready,"initialization acceptance differs from mechanism expectation");require(consistency,"actual estimator state differs from exact accepted window / extrinsics");
  if(enabled&&expected_ready){require(accepted>=200&&deskewed>0,"count or real deskew missing");require(first_end>=1001.,"duration gate bypass");}
  if(name=="moving_then_static")require(window_first>=1000800000000LL,"moving prefix leaked into estimator window");
  if(name=="tilted_static")require(std::abs(filter.get_x().grav[1])>4,"tilt wrongly leveled to world Z");
  std::ofstream result(out/"result.json");result<<std::setprecision(17)<<"{\"schema\":\"frontend_init_v4_actual_process_v1\",\"case\":\""<<name<<"\",\"source_provenance\":\"synthetic\",\"quality_gate_enabled\":"<<(enabled?"true":"false")
    <<",\"mechanism_checks_passed\":true,\"ros_started\":false,\"ros_published\":false,\"laser_measurement_update_exercised\":false,\"first_initialized_scan\":"<<first<<",\"first_initialized_scan_end_s\":"<<first_end<<",\"accepted_samples\":"<<accepted<<",\"window_first_ns\":"<<window_first<<",\"actual_deskewed_scans\":"<<deskewed<<",\"frames\":\"W=camera_init,I=body,L=LiDAR; no world leveling\"}\n";
  return 0;
}catch(const std::exception &e){std::cerr<<"frontend_init_v4: "<<e.what()<<std::endl;return 2;}}
