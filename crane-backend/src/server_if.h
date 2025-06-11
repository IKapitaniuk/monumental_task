#pragma once
#include <functional>
#include <vector>

namespace assignment {

template <typename MsgType>
class IServer {
 public:
  using MsgCallback = std::function<void(const MsgType&)>;
  virtual ~IServer(){};
  virtual void run() = 0;
  virtual void publish(const MsgType&) = 0;
  virtual void on_open() = 0;
  virtual void on_close() = 0;
  virtual void add_message_callback(MsgCallback&&) = 0;
};

}  // namespace assignment