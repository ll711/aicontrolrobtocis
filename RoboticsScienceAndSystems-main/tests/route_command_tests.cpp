#include "RouteCommandParser.h"
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {
int checks = 0;
void require(bool value, const char* message) {
    ++checks;
    if (!value) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
struct Sink : route_commands::Sink {
    bool stop_ok = true, start_ok = true, reset_ok = true, commit_ok = true;
    int stops = 0, starts = 0, resets = 0, commits = 0;
    std::vector<tree1_embedded::Point> route;
    bool stop() override { ++stops; return stop_ok; }
    bool start() override { ++starts; return start_ok; }
    bool resetPose() override { ++resets; return reset_ok; }
    bool commitRoute(const tree1_embedded::Point* points, std::size_t count) override {
        ++commits;
        if (!commit_ok) return false;
        route.assign(points, points + count);
        return true;
    }
};
struct Fixture {
    route_commands::Parser parser;
    Sink sink;
    std::vector<std::string> responses;
    route_commands::Result feed(const std::string& bytes) {
        auto result = route_commands::Result::None;
        for (const char byte : bytes) {
            const auto current = parser.feed(byte, sink);
            if (current != route_commands::Result::None) {
                result = current;
                responses.emplace_back(parser.lastResponse());
            }
        }
        return result;
    }
    void accepted(const std::string& command) {
        require(feed(command + "\n") == route_commands::Result::Accepted, "valid command accepted");
    }
    void rejected(const std::string& command) {
        require(feed(command + "\n") == route_commands::Result::Rejected, "invalid command rejected");
        require(!parser.uploading(), "error cancels incomplete upload");
    }
};
void uploadAndBoundaries() {
    Fixture f;
    require(f.feed("ROUTE_BE") == route_commands::Result::None, "TCP fragment waits for complete line");
    require(f.feed("GIN 3\r\n") == route_commands::Result::Accepted, "CRLF and fragmented command accepted");
    require(f.sink.stops == 1 && f.parser.uploading(), "upload starts by stopping hardware");
    require(f.responses.back() == "ACK ROUTE_BEGIN 3", "begin ACK includes declared point count");
    f.accepted("ROUTE_POINT +0e0 -0.0");
    f.accepted("PING");
    require(f.parser.uploading() && f.parser.receivedCount() == 1, "heartbeat preserves staged points");
    f.accepted("ROUTE_POINT .25 0");
    require(f.responses.back() == "ACK ROUTE_POINT 2", "point ACK includes received count");
    require(f.sink.commits == 0, "staged points never replace active route early");
    f.accepted("ROUTE_POINT 0.25 2.5e-1");
    f.accepted("ROUTE_END");
    require(f.responses.back() == "ACK ROUTE_END 3", "end ACK includes committed count");
    require(f.sink.route.size() == 3 && f.sink.route.back().y == 0.25, "complete route copied in metre coordinates");
    require(!f.parser.uploading() && f.sink.starts == 0, "commit never auto-starts hardware");
    f.accepted("START");
    require(f.sink.starts == 1, "explicit START delegates once");
    f.accepted("RESET_POSE");
    require(f.sink.stops == 2 && f.sink.resets == 1, "pose reset requires successful stop first");

    Fixture boundary;
    boundary.accepted("ROUTE_BEGIN 2");
    boundary.accepted("ROUTE_POINT 6 10");
    boundary.accepted("ROUTE_POINT 10 10");
    boundary.accepted("ROUTE_END");
    require(boundary.sink.route.size() == 2, "exact coordinate and length boundaries accepted");
    boundary.accepted("ROUTE_BEGIN 256");
    for (int i = 0; i < 256; ++i) boundary.accepted("ROUTE_POINT " + std::to_string(i * 0.001) + " 0");
    boundary.accepted("ROUTE_END");
    require(boundary.sink.route.size() == 256, "maximum vertex count accepted");
}
void malformedUploads() {
    for (const std::string count : {"0", "1", "257", "-2", "+2", "2.0", "99999999999999999999"}) {
        Fixture f;
        f.rejected("ROUTE_BEGIN " + count);
        require(f.sink.commits == 0, "invalid count cannot commit");
    }
    for (const std::string number : {"nan", "inf", "0x1p0", "1e999", "1e-999", "10.0001", "1oops", ".", "1e+"}) {
        Fixture f;
        f.accepted("ROUTE_BEGIN 2");
        f.rejected("ROUTE_POINT " + number + " 0");
        require(f.sink.commits == 0, "invalid coordinate cannot commit");
    }
    for (const std::string command : {"START extra", "PING extra", "STOP extra", "RESET_POSE extra", "UNKNOWN", ""}) {
        Fixture f;
        f.rejected(command);
    }
    Fixture f;
    f.accepted("ROUTE_BEGIN 2");
    f.accepted("ROUTE_POINT 0 0");
    f.rejected("ROUTE_END");
    f.accepted("ROUTE_BEGIN 2");
    f.accepted("ROUTE_POINT 0 0");
    f.accepted("ROUTE_POINT 0 0");
    f.rejected("ROUTE_END");
    f.accepted("ROUTE_BEGIN 2");
    f.accepted("ROUTE_POINT 0 0");
    f.rejected("ROUTE_POINT 4.001 0");
    f.accepted("ROUTE_BEGIN 2");
    f.rejected("START");
    require(f.sink.starts == 0, "START cannot execute a partial route");
    f.accepted("ROUTE_BEGIN 2");
    f.rejected("RESET_POSE");
    require(f.sink.resets == 0, "pose reset cannot alter coordinate frame during upload");
}
void streamRecoveryAndFailures() {
    Fixture f;
    f.accepted("ROUTE_BEGIN 2");
    require(f.feed(std::string(128, 'X')) == route_commands::Result::Rejected, "overlong line rejected immediately");
    require(f.feed("START\n") == route_commands::Result::None, "overlong suffix discarded through newline");
    require(f.sink.starts == 0, "malformed line suffix cannot inject START");
    f.accepted("PING");
    f.rejected(std::string("PI\0NG", 5));
    f.accepted("PING");
    f.rejected("PING\rSTART");
    f.accepted("PING");
    f.accepted("ROUTE_BEGIN 2");
    f.feed("ROUTE_POINT 0");
    f.parser.disconnect();
    f.rejected(" 0");
    require(!f.parser.uploading() && f.sink.commits == 0, "disconnect discards fragments and staged route");
    f.accepted("ROUTE_BEGIN 2");
    f.accepted("STOP");
    require(!f.parser.uploading(), "STOP cancels upload");
    f.sink.stop_ok = false;
    f.rejected("ROUTE_BEGIN 2");
    f.rejected("RESET_POSE");
    require(f.sink.resets == 0, "failed stop prevents pose reset");
    f.sink.stop_ok = true;
    f.sink.commit_ok = false;
    f.accepted("ROUTE_BEGIN 2");
    f.accepted("ROUTE_POINT 0 0");
    f.accepted("ROUTE_POINT 0.2 0");
    f.rejected("ROUTE_END");
    require(f.sink.route.empty(), "failed commit leaves destination route unchanged");
}
}  // namespace
int main() {
    uploadAndBoundaries();
    malformedUploads();
    streamRecoveryAndFailures();
    std::cout << "PASS: " << checks << " route protocol checks\n";
}
