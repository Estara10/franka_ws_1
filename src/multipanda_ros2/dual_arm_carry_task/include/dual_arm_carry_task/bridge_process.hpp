#pragma once

#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <signal.h>
#include <spawn.h>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

extern char **environ;

namespace dual_arm_carry_task
{
// Owns exactly one bridge process. poll() reaps completed children; stop() is
// bounded and escalates only for this PID. No controller/service lock is held.
class BridgeProcess
{
public:
  ~BridgeProcess() { if (pid_ > 0) stop(); }
  BridgeProcess() = default;
  BridgeProcess(const BridgeProcess&) = delete;
  BridgeProcess& operator=(const BridgeProcess&) = delete;

  bool start(const std::string& executable, const std::vector<std::string>& args,
             std::string& error)
  {
    if (pid_ > 0 || started_) { error = "bridge already started"; return false; }
    started_ = true;
    std::vector<std::string> storage;
    storage.reserve(args.size() + 1);
    storage.push_back(executable);
    storage.insert(storage.end(), args.begin(), args.end());
    std::vector<char*> argv;
    argv.reserve(storage.size() + 1);
    for (auto& item : storage) argv.push_back(item.data());
    argv.push_back(nullptr);
    const int rc = posix_spawn(&pid_, executable.c_str(), nullptr, nullptr, argv.data(), environ);
    if (rc != 0) {
      pid_ = -1;
      status_ = -2;
      error = std::string("posix_spawn: ") + std::strerror(rc);
      return false;
    }
    return true;
  }

  // Returns -1 while live, a wait status after exit, or -2 on wait failure.
  int poll()
  {
    if (pid_ <= 0) return status_;
    int status = 0;
    const pid_t rc = waitpid(pid_, &status, WNOHANG);
    if (rc == pid_) { pid_ = -1; status_ = status; return status; }
    if (rc < 0 && errno != EINTR) { pid_ = -1; status_ = -2; return status_; }
    return -1;
  }

  bool stop()
  {
    if (pid_ <= 0) return true;
    kill(pid_, SIGINT);
    if (waitBounded(std::chrono::milliseconds(500))) return true;
    kill(pid_, SIGTERM);
    if (waitBounded(std::chrono::milliseconds(1000))) return true;
    kill(pid_, SIGKILL);
    return waitBounded(std::chrono::milliseconds(1000));
  }

private:
  bool waitBounded(std::chrono::milliseconds budget)
  {
    const auto until = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < until) {
      if (poll() != -1) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return poll() != -1;
  }
  pid_t pid_{-1};
  int status_{-1};
  bool started_{false};
};

enum class ProtectedChildResult { COMPLETED, PROTECTION_FAILED, CHILD_FAILED, TIMEOUT };

// Shared production/test loop: protection callback runs while the child is
// active; a failed callback stops and reaps the publisher before returning.
template<typename Monitor, typename Complete, typename Success>
ProtectedChildResult runProtectedChild(
  BridgeProcess& child, Monitor monitor, Complete complete, Success success,
  std::chrono::milliseconds wall_budget, int& child_status, bool& reaped)
{
  const auto deadline = std::chrono::steady_clock::now() + wall_budget;
  bool child_finished = false;
  child_status = -1;
  reaped = true;
  while (true) {
    if (!monitor()) {
      reaped = child.stop();
      return ProtectedChildResult::PROTECTION_FAILED;
    }
    if (!child_finished) {
      child_status = child.poll();
      if (child_status != -1) {
        child_finished = true;
        if (!success(child_status)) return ProtectedChildResult::CHILD_FAILED;
      }
    }
    if (child_finished && complete()) return ProtectedChildResult::COMPLETED;
    if (std::chrono::steady_clock::now() >= deadline) {
      if (!child_finished) reaped = child.stop();
      return ProtectedChildResult::TIMEOUT;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
}  // namespace dual_arm_carry_task
