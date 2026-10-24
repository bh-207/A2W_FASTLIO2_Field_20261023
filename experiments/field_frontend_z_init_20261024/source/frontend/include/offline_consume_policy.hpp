#pragma once
#include <cstdint>
#include <string>
namespace offline_consume {
inline bool isolated(const std::string& mode,const std::string& domain,const std::string& local){
  return mode=="offline-test"&&domain=="87"&&local=="1";
}
inline bool full_counts(std::uint64_t imu,std::uint64_t cloud){return imu==166023&&cloud==8188;}
}
