#include <server_if.h>

#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <functional>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <unordered_set>

namespace assignment {

using MsgType = std::string;

class WebsocketSession;

class WebsocketServer final : public IServer<MsgType> {
 public:
  WebsocketServer(boost::asio::io_context& ioc,
                  const boost::asio::ip::address& address, unsigned short port)
      : m_ioc{ioc}, m_adress{address}, m_port{port} {
    m_callback = [](const MsgType&) {};
  }

  void run() override;

  void on_open() override { std::cout << "connected\n"; }

  void on_close() override { std::cout << "disconnected\n"; }

  void add_message_callback(MsgCallback&& cb) override {
    m_callback = std::move(cb);
  }

  void publish(const MsgType& msg) override;

  void on_message(const MsgType& msg) { m_callback(msg); }

  void attach_new_session(
      const std::shared_ptr<WebsocketSession>& new_session) {
    m_active_sessions.insert(new_session);
  }

  void remove_session(const std::shared_ptr<WebsocketSession>& session) {
    m_active_sessions.erase(session);
  }

 private:
  boost::asio::io_context& m_ioc;
  boost::asio::ip::address m_adress;
  unsigned short m_port;
  MsgCallback m_callback;
  std::unordered_set<std::shared_ptr<WebsocketSession>> m_active_sessions;
};

class WebsocketSession final
    : public std::enable_shared_from_this<WebsocketSession> {
  using MsgBuffer = std::queue<MsgType>;

 public:
  WebsocketSession(boost::asio::ip::tcp::socket&& socket,
                   WebsocketServer& server)
      : m_ws{std::move(socket)}, m_server{server} {
    m_ws.text(true);
  }

  ~WebsocketSession() { m_server.on_close(); }
  void run();
  void send(MsgType msg);

 private:
  void on_run();
  void fail(boost::beast::error_code ec, char const* what);
  void on_accept(boost::beast::error_code ec);
  void on_read(boost::beast::error_code ec, std::size_t bytes_transferred);
  void on_write(boost::beast::error_code ec, std::size_t bytes_transferred);

  boost::beast::flat_buffer m_buffer;
  boost::beast::websocket::stream<boost::asio::ip::tcp::socket> m_ws;
  WebsocketServer& m_server;
  MsgBuffer m_queue;
};

class Listener : public std::enable_shared_from_this<Listener> {
 public:
  Listener(boost::asio::io_context& ioc,
           boost::asio::ip::tcp::endpoint endpoint, WebsocketServer& server);

  void run() { do_accept(); }

 private:
  void fail(boost::beast::error_code ec, char const* what);

  void do_accept();

  void on_accept(boost::beast::error_code ec,
                 boost::asio::ip::tcp::socket socket);

  boost::asio::io_context& m_ioc;
  boost::asio::ip::tcp::acceptor m_acceptor;
  WebsocketServer& m_server;
};

}  // namespace assignment