#pragma once

#include "Tree1PathController.h"
#include <cstddef>

namespace route_commands {

enum class Result { None, Accepted, Rejected };

// Callbacks execute synchronously. commitRoute must copy the supplied points
// before returning. A failed commit must leave motion disabled; the sink may
// unload the old route. The parser only calls it after a complete, validated
// upload, while the robot is stopped.
class Sink {
public:
    virtual ~Sink() = default;
    virtual bool stop() = 0;
    virtual bool commitRoute(const tree1_embedded::Point* points, std::size_t count) = 0;
    virtual bool start() = 0;
    virtual bool resetPose() = 0;
};

// Bounded ASCII line protocol; no heap allocation, exceptions, or Arduino APIs.
// LF and CRLF are accepted. Numbers use decimal notation with optional exponent.
// ROUTE_BEGIN n / ROUTE_POINT x_m y_m / ROUTE_END upload a complete route.
// START, STOP, RESET_POSE, and PING take no arguments. PING never changes a route.
class Parser {
public:
    static constexpr std::size_t kMaxPoints = 256;
    static constexpr std::size_t kMaxLineLength = 127;
    static constexpr double kMaxAbsCoordinateMetres = 10.0;
    // Leave room for the endpoint and floating-point resampling differences.
    static constexpr double kSampleCapacityMetres =
        (tree1_embedded::Tree1PathController::kMaxResampledPoints - 2)
        * tree1::Config{}.sample_spacing;
    static constexpr double kMaxRouteLengthMetres =
        kSampleCapacityMetres < 4.0 ? kSampleCapacityMetres : 4.0;

    // Supply each received byte. Accepted/Rejected means lastResponse() has one
    // complete ACK/ERR line, without a newline, for the network layer to send.
    // An overlong or malformed byte sequence is discarded through the next LF.
    Result feed(char byte, Sink& sink);
    const char* lastResponse() const { return response_; }
    bool uploading() const { return uploading_; }
    std::size_t receivedCount() const { return received_; }

    // Drop any partial line and uncommitted upload. Transport/controller code
    // remains responsible for immediately stopping the robot on disconnection.
    void reset();
    void disconnect() { reset(); }

private:
    tree1_embedded::Point points_[kMaxPoints]{};
    char line_[kMaxLineLength + 1]{};
    char response_[96]{};
    std::size_t line_length_ = 0;
    std::size_t expected_ = 0;
    std::size_t received_ = 0;
    double route_length_ = 0;
    bool uploading_ = false;
    bool saw_cr_ = false;
    bool discard_line_ = false;
    bool distinct_ = false;

    void cancelUpload();
    Result execute(Sink& sink);
    Result reject(const char* command, const char* reason);
    Result accept(const char* command);
    Result accept(const char* command, std::size_t count);
};

}  // namespace route_commands
