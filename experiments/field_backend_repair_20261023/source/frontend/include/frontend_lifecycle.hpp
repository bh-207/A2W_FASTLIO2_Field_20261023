#pragma once
#include <stdexcept>
#include <string>

namespace frontend_contract {
// Owned only by the SingleThreadedExecutor/main thread. The signal handler
// writes a separate sig_atomic_t flag; it never touches this state machine.
class Lifecycle {
public:
  enum class Phase { Starting, Running, Stopping, Stopped };
  bool accepts_work() const noexcept { return phase_ == Phase::Running; }
  void run() {
    if (phase_ != Phase::Starting) throw std::logic_error("invalid frontend RUNNING transition");
    phase_ = Phase::Running;
  }
  void request_stop(const std::string &reason, int signal = 0) {
    if (phase_ == Phase::Stopped) return;
    if (reason_.empty()) reason_ = reason;
    if (!signal_) signal_ = signal;
    phase_ = Phase::Stopping;
  }
  void fail(const std::string &error) {
    if (error_.empty()) error_ = error;
    exit_code_ = 2;
    request_stop("error");
  }
  void require_outputs(bool ready, const std::string &detail) const {
    if (phase_ == Phase::Running && !ready) throw std::runtime_error(detail);
  }
  void stopped() noexcept { phase_ = Phase::Stopped; }
  const char *phase_name() const noexcept {
    switch (phase_) {
      case Phase::Starting: return "STARTING";
      case Phase::Running: return "RUNNING";
      case Phase::Stopping: return "STOPPING";
      case Phase::Stopped: return "STOPPED";
    }
    return "INVALID";
  }
  int exit_code() const noexcept { return exit_code_; }
  int signal() const noexcept { return signal_; }
  const std::string &reason() const noexcept { return reason_; }
  const std::string &error() const noexcept { return error_; }
private:
  Phase phase_{Phase::Starting};
  int exit_code_{0}, signal_{0};
  std::string reason_, error_;
};
}
