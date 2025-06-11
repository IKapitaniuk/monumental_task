#pragma once
#include <timer_if.h>

#include <boost/asio.hpp>
#include <functional>

namespace assignment {

class Timer final : public ITimer {
 public:
  Timer(boost::asio::io_context& ioc, boost::posix_time::millisec interval)
      : m_interval{interval}, m_timer{ioc, interval}, m_stop{false} {
    m_callback = []() {};
  }

  void start() override {
    m_timer.expires_from_now(m_interval);
    restart();
  }

  void stop() override { m_stop = true; }

  void add_callback(TimerCallback&& cb) override { m_callback = std::move(cb); }

 private:
  void on_tick() {
    if (m_stop) {
      m_stop = false;
      return;
    }
    m_callback();
    m_timer.expires_at(m_timer.expires_at() + m_interval);
    restart();
  }

  void restart() {
    m_timer.async_wait(
        [this](const boost::system::error_code& /*e*/) { this->on_tick(); });
  }

  boost::posix_time::millisec m_interval;
  boost::asio::deadline_timer m_timer;
  bool m_stop;
  TimerCallback m_callback;
};

}  // namespace assignment