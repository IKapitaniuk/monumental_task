#pragma once

#include <functional>

namespace assignment {

class ITimer {
 public:
  using TimerCallback = std::function<void()>;
  virtual ~ITimer() {}
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void add_callback(TimerCallback&&) = 0;
};

}  // namespace assignment