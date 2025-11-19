#pragma once
#include <mutex>
#include <set>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

using server = websocketpp::server<websocketpp::config::asio>;
using websocketpp::connection_hdl;

class TelemetryServer {
    server ws_server;
    std::set<connection_hdl, std::owner_less<connection_hdl>> connections;
    std::mutex mutex;

  public:
    TelemetryServer() {
        ws_server.init_asio();

        ws_server.set_open_handler([this](connection_hdl hdl) {
            std::lock_guard<std::mutex> lock(mutex);
            connections.insert(hdl);
        });

        ws_server.set_close_handler([this](connection_hdl hdl) {
            std::lock_guard<std::mutex> lock(mutex);
            connections.erase(hdl);
        });

        ws_server.set_message_handler([](connection_hdl, server::message_ptr) {});
    }

    void run(uint16_t port = 9002) {
        ws_server.listen(port);
        ws_server.start_accept();
        ws_server.run();
    }

    void broadcast(const std::string &msg) {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto &c : connections)
            ws_server.send(c, msg, websocketpp::frame::opcode::text);
    }
};
