#pragma once
#include <cstdint>
#include <string>
namespace offline_consume {
inline bool isolated(const std::string& mode,const std::string& domain,const std::string& local){
  return mode=="offline-test"&&domain=="87"&&local=="1";
}
inline bool full_counts(std::uint64_t imu,std::uint64_t cloud,
    std::uint64_t expected_imu=166023,std::uint64_t expected_cloud=8188){
  return expected_imu>0&&expected_cloud>0&&imu==expected_imu&&cloud==expected_cloud;
}
}
