// Standalone export only: never optimizes, re-registers, changes gauge, or overwrites.
#define LOOP_CONSTRAINT_ABLATION_NO_MAIN
#include "loop_constraint_ablation.cpp"
#include "a2w_fastlio_mapping/optimized_map_builder.hpp"
#include "a2w_fastlio_map/map_bundle_reader.hpp"
#include "a2w_fastlio_map/map_bundle_writer.hpp"
#include <array>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <random>
#include <sys/syscall.h>
#include <unistd.h>
#ifndef RENAME_NOREPLACE
#define RENAME_NOREPLACE (1 << 0)
#endif
namespace field_bundle {
namespace fs=std::filesystem;
namespace mapping=a2w_fastlio_mapping;
namespace map=a2w_fastlio_map;
class Provider final:public a2w_fastlio_common::KeyFrameProvider {
public:
 explicit Provider(const std::vector<a2w_fastlio_common::KeyFrame>&frames):frames_(frames){}
 std::optional<a2w_fastlio_common::KeyFrame> get(std::uint64_t id)const override {
  if(id>=frames_.size()){return std::nullopt;}
  return frames_[id];
 }
 std::optional<std::pair<std::uint64_t,std::uint64_t>> bounds()const override {
  if(frames_.empty()){return std::nullopt;}
  return std::pair<std::uint64_t,std::uint64_t>{0,frames_.size()-1};
 }
private:const std::vector<a2w_fastlio_common::KeyFrame>&frames_;
};
bool within(const fs::path&child,const fs::path&parent){
 auto i=child.begin();for(auto j=parent.begin();j!=parent.end();++j,++i)if(i==child.end()||*i!=*j)return false;return true;
}
std::string uuid(){
 std::array<unsigned char,16>bytes;std::random_device random;
 for(auto&b:bytes)b=static_cast<unsigned char>(random());
 bytes[6]=(bytes[6]&0x0fU)|0x40U;bytes[8]=(bytes[8]&0x3fU)|0x80U;
 std::ostringstream out;out<<std::hex<<std::setfill('0');
 for(std::size_t i=0;i<bytes.size();++i){if(i==4||i==6||i==8||i==10)out<<'-';out<<std::setw(2)<<unsigned(bytes[i]);}return out.str();
}
std::string utc(){
 const auto now=std::time(nullptr);std::tm tm{};if(!gmtime_r(&now,&tm))throw std::runtime_error("UTC conversion failed");
 std::ostringstream out;out<<std::put_time(&tm,"%Y-%m-%dT%H:%M:%SZ");return out.str();
}
int run(int argc,char**argv){
 if(argc!=4)throw std::runtime_error("usage: field_bundle_from_graph ORIGINAL_BUNDLE OPTIMIZED_POSES_CSV NEW_OUTPUT_BUNDLE");
 const auto original=fs::canonical(argv[1]),posefile=fs::canonical(argv[2]);
 const auto requested=fs::absolute(argv[3]);
 if(fs::is_symlink(fs::symlink_status(requested)))throw std::runtime_error("output symlink forbidden");
 const auto output=fs::weakly_canonical(requested);
 if(fs::exists(output))throw std::runtime_error("refusing to overwrite existing output");
 if(within(output,original)||within(original,output)||within(posefile,output))throw std::runtime_error("output overlaps read-only inputs");
 auto source=map::MapBundleReader{}.read(original);auto&data=source.data;
 const auto original_uuid=data.metadata.bundle_uuid;
 // Reader normalizes poses for safe consumption; restore the exact saved raw
 // quaternion components so an export does not alter original odometry bytes.
 const auto raw_rows=ablation::readCsv(original/"keyframes/index.csv");
 if(raw_rows.size()!=data.keyframes.size())throw std::runtime_error("original raw pose count mismatch");
 for(std::size_t i=0;i<raw_rows.size();++i){const auto&r=raw_rows[i];
  if(ablation::id(r.at("id"))!=i)throw std::runtime_error("original raw IDs mismatch");
  data.keyframes[i].odom_pose.rotation=Eigen::Quaterniond{std::stod(r.at("qw")),std::stod(r.at("qx")),std::stod(r.at("qy")),std::stod(r.at("qz"))};
 }
 const auto rows=ablation::readCsv(posefile);
 if(rows.size()!=data.keyframes.size())throw std::runtime_error("pose count differs from original keyframe count");
 mapping::OptimizedPoseSnapshot snapshot;snapshot.revision=1;
 for(std::size_t i=0;i<rows.size();++i){if(ablation::id(rows[i].at("id"))!=i)throw std::runtime_error("pose IDs must be contiguous and ordered from0");
  const auto pose=ablation::readPose(rows[i]);data.keyframes[i].optimized_pose=pose;snapshot.poses.push_back({i,pose});}
 const auto config=YAML::Load(data.config_snapshots.at("mapping_effective.yaml"));
 const double leaf=data.metadata.global_map_leaf_m;
 if(!std::isfinite(leaf)||leaf<0)throw std::runtime_error("invalid original full-map leaf");
 if(config["full_map_voxel_leaf_m"]&&std::abs(config["full_map_voxel_leaf_m"].as<double>()-leaf)>1e-12)throw std::runtime_error("original leaf config disagrees with bundle metadata");
 const double preview_leaf=config["preview_voxel_leaf_m"]?config["preview_voxel_leaf_m"].as<double>():.5;
 const auto preview_max=config["preview_max_points"]?config["preview_max_points"].as<std::size_t>():100000U;
 data.global_map=mapping::OptimizedMapBuilder{{leaf,preview_leaf,preview_max}}.buildFull(Provider{data.keyframes},snapshot);
 data.metadata.bundle_uuid=uuid();data.metadata.created_utc=utc();
 data.metadata.creation_status="offline_experiment_no_hardware";
 data.metadata.hardware_validation_status="not_validated_no_hardware";
 fs::create_directories(output.parent_path());
 const auto staging=output.parent_path()/("."+output.filename().string()+".field-export-"+data.metadata.bundle_uuid);
 if(fs::exists(staging)||fs::is_symlink(fs::symlink_status(staging)))throw std::runtime_error("unique staging collision");
 const auto result=map::MapBundleWriter{}.write(staging,data);
 if(!result.success)throw std::runtime_error("MapBundleWriter failed:"+result.message);
 if(::syscall(SYS_renameat2,AT_FDCWD,staging.c_str(),AT_FDCWD,output.c_str(),RENAME_NOREPLACE)!=0){
  const auto reason=std::string(std::strerror(errno));std::error_code ignored;fs::remove_all(staging,ignored);
  throw std::runtime_error("atomic no-overwrite publish failed:"+reason);
 }
 const auto verified=map::MapBundleReader{}.read(output);
 if(verified.data.keyframes.size()!=snapshot.poses.size()||verified.data.metadata.bundle_uuid==original_uuid||verified.data.metadata.bundle_uuid!=data.metadata.bundle_uuid)throw std::runtime_error("output verification count/UUID mismatch");
 double pose_translation_error=0.,pose_rotation_error=0.;
 for(std::size_t i=0;i<snapshot.poses.size();++i){
  pose_translation_error=std::max(pose_translation_error,(snapshot.poses[i].pose.translation-verified.data.keyframes[i].optimized_pose.translation).norm());
  pose_rotation_error=std::max(pose_rotation_error,Eigen::AngleAxisd(snapshot.poses[i].pose.rotation.inverse()*verified.data.keyframes[i].optimized_pose.rotation).angle());
 }
 if(pose_translation_error>1e-12||pose_rotation_error>1e-12)throw std::runtime_error("published poses changed during serialization");
 std::cout<<"status=verified\noutput="<<output.string()<<"\nbundle_uuid="<<verified.data.metadata.bundle_uuid
  <<"\nkeyframes="<<verified.data.keyframes.size()<<"\nglobal_map_points="<<verified.data.global_map->size()
  <<"\nfull_map_voxel_leaf_m="<<std::setprecision(17)<<leaf
  <<"\ncreation_status="<<verified.data.metadata.creation_status<<"\nhardware_validation_status="<<verified.data.metadata.hardware_validation_status
  <<"\npose_translation_roundtrip_max_m="<<pose_translation_error<<"\npose_rotation_roundtrip_max_rad="<<pose_rotation_error<<'\n';
 return 0;
}
}
#ifndef FIELD_BUNDLE_FROM_GRAPH_NO_MAIN
int main(int argc,char**argv){try{return field_bundle::run(argc,argv);}catch(const std::exception&e){std::cerr<<"field_bundle_from_graph: "<<e.what()<<std::endl;return 1;}}
#endif
