#include "startup_trace.hpp"
#include <iostream>

int main() {
  const std::vector<std::string> required{
    "/mapping_ingress_node", "/field_capture_observer_*", "/rosbag2_recorder"};
  const std::vector<std::string> with_debug{
    "/mapping_ingress_node", "/field_capture_observer_42", "/rosbag2_recorder", "/debug_view"};
  if (!offline_handoff::matched(4, with_debug, required)) {
    std::cerr << "required endpoints plus an unrelated debug consumer must be accepted\n";
    return 1;
  }
  if (offline_handoff::matched(3, {"/mapping_ingress_node", "/rosbag2_recorder", "/debug_view"}, required)) {
    std::cerr << "same count cannot substitute for missing observer identity\n";
    return 1;
  }
  if (offline_handoff::matched(2, with_debug, required)) {
    std::cerr << "unmatched required output edges must fail\n";
    return 1;
  }
  if (offline_handoff::matched(4, {"/mapping_ingress_node", "/field_capture_observer_1",
      "/field_capture_observer_2", "/rosbag2_recorder"}, required)) {
    std::cerr << "ambiguous required wildcard identity must fail\n";
    return 1;
  }
  if (offline_handoff::matched(3, {"/other/mapping_ingress_node", "/field_capture_observer_42", "/rosbag2_recorder"}, required)) {
    std::cerr << "another namespace must not satisfy required identity\n";
    return 1;
  }
  std::cout << "required endpoint contract passed\n";
}
