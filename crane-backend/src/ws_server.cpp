#include <ws_server.h>

#include <memory>

namespace assignment {

void WebsocketServer::run() {
  std::cout << "Running the server IP adress: " << m_adress
            << " Port: " << m_port << '\n';
  std::make_shared<Listener>(
      m_ioc, boost::asio::ip::tcp::endpoint{m_adress, m_port}, *this)
      ->run();
}

void WebsocketServer::publish(const MsgType& msg) {
  for (auto& session : m_active_sessions) {
    session->send(msg);
  }
}

Listener::Listener(boost::asio::io_context& ioc,
                   boost::asio::ip::tcp::endpoint endpoint,
                   WebsocketServer& server)
    : m_ioc{ioc}, m_acceptor{ioc}, m_server{server} {
  boost::beast::error_code ec;
  m_acceptor.open(endpoint.protocol(), ec);
  if (ec) {
    fail(ec, "open");
    return;
  }

  m_acceptor.set_option(boost::asio::socket_base::reuse_address(true), ec);
  if (ec) {
    fail(ec, "set_option");
    return;
  }

  m_acceptor.bind(endpoint, ec);
  if (ec) {
    fail(ec, "bind");
    return;
  }

  m_acceptor.listen(boost::asio::socket_base::max_listen_connections, ec);
  if (ec) {
    fail(ec, "listen");
    return;
  }
}

void Listener::fail(boost::beast::error_code ec, char const* what) {
  if (ec == boost::asio::error::operation_aborted ||
      ec == boost::beast::websocket::error::closed)
    return;

  std::cerr << what << ": " << ec.message() << "\n";
}

void Listener::do_accept() {
  m_acceptor.async_accept(boost::asio::make_strand(m_ioc),
                          boost::beast::bind_front_handler(&Listener::on_accept,
                                                           shared_from_this()));
}

void Listener::on_accept(boost::beast::error_code ec,
                         boost::asio::ip::tcp::socket socket) {
  if (ec) {
    fail(ec, "accept");
  } else {
    std::make_shared<WebsocketSession>(std::move(socket), m_server)->run();
  }
  do_accept();
}

void WebsocketSession::run() {
  boost::asio::dispatch(m_ws.get_executor(),
                        [sp = shared_from_this()]() { sp->on_run(); });
}

void WebsocketSession::send(std::string msg) {
  m_queue.push(std::move(msg));

  if (m_queue.size() > 1) return;

  m_ws.async_write(boost::asio::buffer(m_queue.front()),
                   [sp = shared_from_this()](boost::beast::error_code ec,
                                             std::size_t bytes) {
                     sp->on_write(ec, bytes);
                   });
}

void WebsocketSession::on_run() {
  m_ws.set_option(boost::beast::websocket::stream_base::timeout::suggested(
      boost::beast::role_type::server));

  m_ws.set_option(boost::beast::websocket::stream_base::decorator(
      [](boost::beast::websocket::response_type& res) {
        res.set(boost::beast::http::field::server,
                std::string(BOOST_BEAST_VERSION_STRING) +
                    " websocket-server-async");
      }));

  m_ws.async_accept([sp = shared_from_this()](boost::beast::error_code ec) {
    sp->on_accept(ec);
  });
}

void WebsocketSession::fail(boost::beast::error_code ec, char const* what) {
  if (ec == boost::asio::error::operation_aborted ||
      ec == boost::asio::error::eof ||
      ec == boost::beast::websocket::error::closed) {
    auto sp = shared_from_this();
    m_server.remove_session(sp);
    return;
  }

  std::cerr << what << ": " << ec.message() << "\n";
}

void WebsocketSession::on_accept(boost::beast::error_code ec) {
  if (ec) return fail(ec, "accept");
  m_server.on_open();

  auto sp = shared_from_this();
  m_server.attach_new_session(sp);

  m_ws.async_read(m_buffer,
                  [sp](boost::beast::error_code ec, std::size_t bytes) {
                    sp->on_read(ec, bytes);
                  });
}

void WebsocketSession::on_read(boost::beast::error_code ec,
                               std::size_t bytes_transferred) {
  boost::ignore_unused(bytes_transferred);
  if (ec) return fail(ec, "read");

  MsgType msg = boost::beast::buffers_to_string(m_buffer.data());
  // std::cout << msg;
  m_server.on_message(msg);

  m_buffer.consume(m_buffer.size());
  m_ws.async_read(m_buffer, [sp = shared_from_this()](
                                boost::beast::error_code ec,
                                std::size_t bytes) { sp->on_read(ec, bytes); });
}

void WebsocketSession::on_write(boost::beast::error_code ec,
                                std::size_t bytes_transferred) {
  boost::ignore_unused(bytes_transferred);
  if (ec) return fail(ec, "write");

  m_queue.pop();

  if (!m_queue.empty())
    m_ws.async_write(boost::asio::buffer(m_queue.front()),
                     [sp = shared_from_this()](boost::beast::error_code ec,
                                               std::size_t bytes) {
                       sp->on_write(ec, bytes);
                     });
}

}  // namespace assignment