#include "startup_trace.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

int main(int argc, char** argv) {
  using namespace offline_handoff;
  assert(!matched(2, {"mapping_ingress_node", "wrong"}, {"mapping_ingress_node", "offline_mapping_save_observer_*"}));
  assert(!matched(1, {"mapping_ingress_node", "offline_mapping_save_observer_42"}, {"mapping_ingress_node", "offline_mapping_save_observer_*"}));
  assert(!matched(3, {"mapping_ingress_node", "offline_mapping_save_observer_42", "extra"}, {"mapping_ingress_node", "offline_mapping_save_observer_*"}));
  assert(matched(2, {"mapping_ingress_node", "offline_mapping_save_observer_42"}, {"mapping_ingress_node", "offline_mapping_save_observer_*"}));
  assert(!matched(2, {"offline_mapping_save_observer_42", "offline_mapping_save_observer_43"}, {"mapping_ingress_node", "offline_mapping_save_observer_*"}));
  auto dir = std::filesystem::path(argv[1]);
  bool rejected=false;
  try { Trace bad(dir, "invalid", 0); } catch(const std::invalid_argument&) { rejected=true; }
  assert(rejected);
  const std::int64_t stamp=(std::int64_t(1)<<60)+3;
  { Trace trace(dir,"fixture",2048);for(int i=0;i<1000;++i) trace.record("test",stamp+i,i,"exact_integer"); trace.close(); }
  std::ifstream f(dir/"fixture_handoff.csv");std::stringstream all;all<<f.rdbuf();
  assert(all.str().find(std::to_string(stamp))!=std::string::npos);
  std::ifstream s(dir/"fixture_handoff_trace_status.json");std::stringstream status;status<<s.rdbuf();
  assert(status.str().find("\"written\":1000")!=std::string::npos);
  assert(status.str().find("\"closed\":true")!=std::string::npos);
  rejected=false;try {Trace same(dir,"fixture",8);}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
}
