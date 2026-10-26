#pragma once
// Explicit offline observation/export. No estimator mathematics or ROS input.
#include "offline_consume_policy.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <memory>
#include <vector>
#include <stdexcept>
#include <Eigen/Core>
#include <numeric>
#include <pcl/io/pcd_io.h>
namespace offline_consume {
class Output {
 public:
  explicit Output(const std::filesystem::path& directory,double export_seconds=60):root(directory),export_ns(std::llround(export_seconds*1e9)){
    if(std::filesystem::exists(root))throw std::invalid_argument("fresh offline reconstruction directory required");
    std::filesystem::create_directories(root/"body_clouds");
    std::filesystem::create_directories(root/"preprocess");
    index.open(root/"post_index.csv");preindex.open(root/"preprocess_index.csv");
    input.open(root/"consumed_raw.csv");fingerprints.open(root/"preprocess_identity.csv");state.open(root/"z_stages.csv");
    for(auto*f:{&index,&preindex,&input,&fingerprints,&state}){*f<<std::setprecision(17);if(!*f)throw std::runtime_error("offline output open failed");}
    index<<"source_scan_id,source_header_stamp_ns,scan_begin_ns,scan_end_ns,cloud,point_ids,tx,ty,tz,qx,qy,qz,qw\n";
    preindex<<"scan_id,stamp_ns,path\n";
    input<<"topic,id,record_ns,header_ns\n";
    fingerprints<<"source_scan_id,header_ns,point_count,xyz_intensity_curvature_fnv1a64\n";
    state<<"stage,source_scan_id,source_header_stamp_ns,scan_begin_ns,scan_end_ns,group_last_imu_ns,state_time_ns,tx,ty,tz,vx,vy,vz,qx,qy,qz,qw,gx,gy,gz,bax,bay,baz,bgx,bgy,bgz,acc_scale,mean_raw_ax,mean_raw_ay,mean_raw_az,effective_features\n";
  }
  template<class Cloud>void preprocess(std::uint64_t id,std::int64_t stamp,Cloud& cloud){
    if(!first_ns)first_ns=stamp;
    std::uint64_t hash=14695981039346656037ULL;
    for(std::size_t i=0;i<cloud.size();++i){
      if(i>=16777216)throw std::runtime_error("offline point identity exceeds exact float integer range");
      auto&p=cloud.points[i];
      for(float value:{p.x,p.y,p.z,p.intensity,p.curvature}){
        unsigned char bytes[4];std::memcpy(bytes,&value,4);for(auto byte:bytes){hash^=byte;hash*=1099511628211ULL;}}
      // Unused normal slot carries the pre-deskew record identity through sort.
      // It is never used by propagation or the point-to-plane observation.
      p.normal_x=static_cast<float>(i);
    }
    fingerprints<<id<<','<<stamp<<','<<cloud.size()<<','<<hash<<'\n';
    if(stamp-first_ns<=export_ns){
      const auto file=std::string("preprocess/")+std::to_string(id)+".pcd";
      if(pcl::io::savePCDFileBinary((root/file).string(),cloud)!=0)throw std::runtime_error("preprocess export failed");
      preindex<<id<<','<<stamp<<','<<file<<'\n';preindex.flush();
    }
  }
  template<class State>void stage(const char* name,std::uint64_t id,std::int64_t stamp,
    double begin,double end,std::int64_t lastimu,const State& x,double scale,const Eigen::Vector3d& mean,std::int64_t state_time,int features=-1){
    state<<name<<','<<id<<','<<stamp<<','<<std::llround(begin*1e9)<<','<<std::llround(end*1e9)<<','<<lastimu<<','<<state_time;
    for(const auto&v:{Eigen::Vector3d(x.pos),Eigen::Vector3d(x.vel)})for(int k=0;k<3;++k)state<<','<<v[k];
    for(int k=0;k<4;++k)state<<','<<x.rot.coeffs()[k];
    for(const auto&v:{Eigen::Vector3d(x.grav),Eigen::Vector3d(x.ba),Eigen::Vector3d(x.bg)})for(int k=0;k<3;++k)state<<','<<v[k];
    state<<','<<scale;for(int k=0;k<3;++k)state<<','<<mean[k];state<<','<<features<<'\n';
  }
  template<class State,class Cloud>void post(std::uint64_t id,std::int64_t stamp,double begin,double end,const State&x,const Cloud&lidar){
    pcl::PointCloud<pcl::PointXYZI> body;body.reserve(lidar.size());std::vector<std::uint32_t>ids;ids.reserve(lidar.size());
    for(const auto&p:lidar){const Eigen::Vector3d v=x.offset_R_L_I*Eigen::Vector3d(p.x,p.y,p.z)+x.offset_T_L_I;
      pcl::PointXYZI q;q.x=v.x();q.y=v.y();q.z=v.z();q.intensity=p.intensity;body.push_back(q);ids.push_back(static_cast<std::uint32_t>(std::llround(p.normal_x)));}
    const auto file=std::string("body_clouds/")+std::to_string(id)+".pcd",idfile=file+".point_ids_u32";
    if(pcl::io::savePCDFileBinary((root/file).string(),body)!=0)throw std::runtime_error("body export failed");
    std::ofstream points(root/idfile,std::ios::binary);points.write(reinterpret_cast<const char*>(ids.data()),ids.size()*sizeof(std::uint32_t));points.close();if(!points)throw std::runtime_error("point identity export failed");
    index<<id<<','<<stamp<<','<<std::llround(begin*1e9)<<','<<std::llround(end*1e9)<<','<<file<<','<<idfile;
    for(int k=0;k<3;++k)index<<','<<x.pos[k];for(int k=0;k<4;++k)index<<','<<x.rot.coeffs()[k];index<<'\n';++posts;
  }
  void close(){for(auto*f:{&index,&preindex,&input,&fingerprints,&state}){f->flush();f->close();if(!*f)throw std::runtime_error("offline output close failed");}}
  std::filesystem::path root;std::ofstream index,preindex,input,fingerprints,state;std::int64_t first_ns{0},export_ns{0};std::size_t posts{0};
};
inline std::unique_ptr<Output> output;
}
