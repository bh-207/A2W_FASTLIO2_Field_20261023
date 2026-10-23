#include "startup_trace.hpp"
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

static const std::string owner="/field_capture_observer_00112233445566778899aabbccddeeff";
static const std::string debug="/field_capture_observer_ffeeddccbbaa99887766554433221100";

static std::string configured_observer() {
#ifdef LEGACY_REQUIRED_OBSERVER_PATTERN
  // The baseline runner extracts this literal from the actual frontend source.
  // No substitute matching algorithm is used: both versions call real matched().
  return LEGACY_REQUIRED_OBSERVER_PATTERN;
#else
  return offline_handoff::required_observer_name();
#endif
}

int main(int argc,char **argv) {
  if(argc!=2)return 2;
  const std::string mode=argv[1];
  setenv("A2W_REQUIRED_OBSERVER_NAME",owner.c_str(),1);
  if(mode=="replacement") {
    frontend_contract::Lifecycle life;life.run();
    const bool ready=offline_handoff::matched(3,
      {"/mapping_ingress_node",debug,"/rosbag2_recorder"},
      {"/mapping_ingress_node",configured_observer(),"/rosbag2_recorder"});
    try {life.require_outputs(ready,"required owned observer is missing");}
    catch(const std::exception& error) {life.fail(error.what());}
    if(life.exit_code()!=2) {
      std::cerr<<"lost owned observer was replaced by prefix-named debug consumer without exit code 2\n";
      return 1;
    }
  } else if(mode=="extra_debug") {
    if(!offline_handoff::matched(5,
      {"/mapping_ingress_node",owner,debug,"/debug_view","/rosbag2_recorder"},
      {"/mapping_ingress_node",configured_observer(),"/rosbag2_recorder"})) {
      std::cerr<<"owned observer plus arbitrary debug consumers must remain valid\n";
      return 1;
    }
  } else if(mode=="configuration") {
#ifdef LEGACY_REQUIRED_OBSERVER_PATTERN
    std::cerr<<"baseline has no exact observer environment validation\n";
    return 1;
#else
    if(configured_observer()!=owner)return 1;
    for(const std::string bad:{"", "field_capture_observer_00112233445566778899aabbccddeeff",
      "/field_capture_observer_*", "/other/field_capture_observer_00112233445566778899aabbccddeeff",
      "/field_capture_observer_bad-uuid", "/field_capture_observer_00112233445566778899aabbccddeeff/"}) {
      setenv("A2W_REQUIRED_OBSERVER_NAME",bad.c_str(),1);
      bool rejected=false;
      try {(void)configured_observer();}catch(const std::invalid_argument&) {rejected=true;}
      if(!rejected){std::cerr<<"invalid observer identity was accepted: "<<bad<<'\n';return 1;}
    }
    unsetenv("A2W_REQUIRED_OBSERVER_NAME");
    bool rejected=false;
    try {(void)configured_observer();}catch(const std::invalid_argument&) {rejected=true;}
    if(!rejected)return 1;
#endif
  } else return 2;
  std::cout<<mode<<" passed\n";
}
