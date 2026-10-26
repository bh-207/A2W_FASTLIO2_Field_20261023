"""Extend the frozen v9 generator with default-off runtime observation only."""
from pathlib import Path
import difflib
import frontend_init_v4_state_patchgen as previous

ROOT=Path(__file__).resolve().parents[2]
once=previous.once

CONFIG=r'''
        frontend_runtime_v4::Config runtime_state_config_v4;
        runtime_strict_gate_v4_=v4_gate.enabled;
        runtime_state_log_enabled_v4_=this->declare_parameter<bool>("frontend_init_v4.runtime_state_log_enabled",false);
        runtime_state_config_v4.enabled=runtime_state_log_enabled_v4_;
        runtime_state_config_v4.path=this->declare_parameter<string>("frontend_init_v4.runtime_state_log_path","");
        const auto runtime_state_capacity_v4=this->declare_parameter<int>("frontend_init_v4.runtime_state_log_capacity",1024);
        if(runtime_state_log_enabled_v4_ && runtime_state_capacity_v4<=0)
          throw std::invalid_argument("runtime state log capacity must be positive");
        runtime_state_config_v4.capacity=static_cast<std::size_t>(runtime_state_capacity_v4);
        runtime_state_config_v4.source_provenance=frontend_input_config_v4.source_provenance;
        if(runtime_state_log_enabled_v4_)
          runtime_state_writer_v4_=std::make_unique<frontend_runtime_v4::Writer>(runtime_state_config_v4);
'''

INIT=r'''
            const bool runtime_was_initialized_v4=runtime_state_log_enabled_v4_ ? p_imu->frontend_init_v4_initialized() : true;
            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            if(runtime_state_log_enabled_v4_ && !runtime_was_initialized_v4 && p_imu->frontend_init_v4_initialized())
            {
                const auto group_last_v4=static_cast<std::int64_t>(Measures.imu.back()->header.stamp.sec)*1000000000LL+Measures.imu.back()->header.stamp.nanosec;
                const auto &window_v4=p_imu->frontend_init_v4_stats();
                // Strict gate uses exactly its committed accepted last IMU.
                // Legacy init has no exact gate window and is explicitly unverified.
                const bool strict_v4=runtime_strict_gate_v4_;
                const auto accepted_last_v4=strict_v4?window_v4.last_stamp_ns:0;
                auto row_v4=frontend_runtime_v4::capture(frontend_runtime_v4::Stage::InitCommit,
                    std::llround(Measures.lidar_beg_time*1e9),std::llround(Measures.lidar_end_time*1e9),
                    strict_v4?accepted_last_v4:group_last_v4,state_point);
                row_v4.accepted_window_last_imu_ns=accepted_last_v4;
                row_v4.group_last_imu_ns=group_last_v4;
                row_v4.init_stamp_consistent=strict_v4 && accepted_last_v4==group_last_v4;
                if(!row_v4.init_stamp_consistent)
                    RCLCPP_ERROR(this->get_logger(),"Runtime INIT_COMMIT source window not verified; do not treat as strict-ready evidence");
                runtime_state_writer_v4_->push(row_v4);
            }
'''

POST=r'''
            if(runtime_state_log_enabled_v4_)
            {
                // Capture immediately after actual lidar update and get_x,
                // before any publish/map operation or next Process can change it.
                auto row_v4=frontend_runtime_v4::capture(frontend_runtime_v4::Stage::PostLidarUpdate,
                    std::llround(Measures.lidar_beg_time*1e9),std::llround(Measures.lidar_end_time*1e9),
                    std::llround(Measures.lidar_end_time*1e9),state_point);
                row_v4.lidar_effective_features=effct_feat_num;
                runtime_state_writer_v4_->push(row_v4);
            }
'''

def modified_sources(source):
    result=previous.modified_sources(source)
    name='src/laserMapping.cpp';old,text=result[name]
    text=once(text,'#include "IMU_Processing.hpp"','#include "IMU_Processing.hpp"\n#include "frontend_runtime_state_v4.hpp"')
    text=once(text,'        this->get_parameter_or<bool>("publish.path_en", path_en, true);',CONFIG+'\n        this->get_parameter_or<bool>("publish.path_en", path_en, true);')
    # InputConfig is configured later in the constructor. Move writer setup to
    # immediately after the complete v9 config rather than reading stale defaults.
    text=once(text,CONFIG,'')
    marker='        p_imu->set_frontend_init_v4(v4_gate,frontend_input_config_v4,v4_trace,v4_state);'
    text=once(text,marker,marker+'\n'+CONFIG)
    text=once(text,'            p_imu->Process(Measures, kf, feats_undistort);\n            state_point = kf.get_x();',INIT)
    marker='            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);\n            state_point = kf.get_x();'
    text=once(text,marker,marker+'\n'+POST)
    text=once(text,'    ~LaserMappingNode()\n    {','    ~LaserMappingNode()\n    {\n        if(runtime_state_writer_v4_)runtime_state_writer_v4_->close();')
    text=once(text,'    FILE *fp = NULL;','    bool runtime_state_log_enabled_v4_{false},runtime_strict_gate_v4_{false};\n    std::unique_ptr<frontend_runtime_v4::Writer> runtime_state_writer_v4_;\n    FILE *fp = NULL;')
    result[name]=(old,text)
    result['include/frontend_runtime_state_v4.hpp']=('',(ROOT/'patches/fast_lio_v4/frontend_runtime_state_v4.hpp').read_text())
    return result

def render_patch(source):
    chunks=[]
    for name,(old,new) in modified_sources(Path(source)).items():
        chunks.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='a/'+name if old else '/dev/null',tofile='b/'+name))
    return ''.join(chunks)

if __name__=='__main__':print(render_patch(ROOT/'src/FAST_LIO_Hesai'),end='')
