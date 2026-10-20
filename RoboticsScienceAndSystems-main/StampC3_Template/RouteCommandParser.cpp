#include "RouteCommandParser.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace route_commands {
namespace {
bool digit(char value) { return value >= '0' && value <= '9'; }
bool whitespace(char value) { return value == ' ' || value == '\t'; }

bool pointCount(const char* text, std::size_t& value) {
    if (!*text) return false;
    value = 0;
    for (; *text; ++text) {
        if (!digit(*text)) return false;
        value = value * 10 + static_cast<unsigned>(*text - '0');
        if (value > Parser::kMaxPoints) return false;
    }
    return value >= 2;
}

bool coordinate(const char* text, double& value) {
    // strtod alone would accept NaN, infinity, hexadecimal, and trailing text.
    // Validate the entire decimal grammar before converting the token.
    const char* cursor = text;
    if (*cursor == '+' || *cursor == '-') ++cursor;
    bool digits = false;
    while (digit(*cursor)) { digits = true; ++cursor; }
    if (*cursor == '.') {
        ++cursor;
        while (digit(*cursor)) { digits = true; ++cursor; }
    }
    if (!digits) return false;
    if (*cursor == 'e' || *cursor == 'E') {
        ++cursor;
        if (*cursor == '+' || *cursor == '-') ++cursor;
        if (!digit(*cursor)) return false;
        while (digit(*cursor)) ++cursor;
    }
    if (*cursor) return false;
    errno = 0;
    char* end = nullptr;
    value = std::strtod(text, &end);
    return errno != ERANGE && end && *end == '\0' && std::isfinite(value)
        && std::abs(value) <= Parser::kMaxAbsCoordinateMetres;
}
}  // namespace

void Parser::cancelUpload() {
    expected_ = 0;
    received_ = 0;
    route_length_ = 0;
    uploading_ = false;
    distinct_ = false;
}

void Parser::reset() {
    cancelUpload();
    line_length_ = 0;
    saw_cr_ = false;
    discard_line_ = false;
    line_[0] = '\0';
    response_[0] = '\0';
}

Result Parser::reject(const char* command, const char* reason) {
    cancelUpload();
    std::snprintf(response_, sizeof(response_), "ERR %s %s", command, reason);
    return Result::Rejected;
}

Result Parser::accept(const char* command) {
    std::snprintf(response_, sizeof(response_), "ACK %s", command);
    return Result::Accepted;
}

Result Parser::accept(const char* command, std::size_t count) {
    std::snprintf(response_, sizeof(response_), "ACK %s %u", command,
                  static_cast<unsigned>(count));
    return Result::Accepted;
}

Result Parser::feed(char byte, Sink& sink) {
    if (discard_line_) {
        if (byte == '\n') {
            discard_line_ = false;
            saw_cr_ = false;
            line_length_ = 0;
        }
        return Result::None;
    }
    if (byte == '\n') {
        saw_cr_ = false;
        line_[line_length_] = '\0';
        line_length_ = 0;
        return execute(sink);
    }
    if (saw_cr_) {
        discard_line_ = true;
        line_length_ = 0;
        return reject("INPUT", "invalid_line_ending");
    }
    if (byte == '\r') {
        saw_cr_ = true;
        return Result::None;
    }
    const unsigned char value = static_cast<unsigned char>(byte);
    if ((value < 32 && byte != '\t') || value > 126) {
        discard_line_ = true;
        line_length_ = 0;
        return reject("INPUT", "invalid_character");
    }
    if (line_length_ >= kMaxLineLength) {
        discard_line_ = true;
        line_length_ = 0;
        return reject("INPUT", "line_too_long");
    }
    line_[line_length_++] = byte;
    return Result::None;
}

Result Parser::execute(Sink& sink) {
    char* tokens[4]{};
    std::size_t count = 0;
    char* cursor = line_;
    while (*cursor) {
        while (whitespace(*cursor)) ++cursor;
        if (!*cursor) break;
        if (count == 4) return reject("INPUT", "too_many_parameters");
        tokens[count++] = cursor;
        while (*cursor && !whitespace(*cursor)) ++cursor;
        if (*cursor) *cursor++ = '\0';
    }
    if (count == 0) return reject("INPUT", "empty_command");
    const char* command = tokens[0];

    if (std::strcmp(command, "ROUTE_BEGIN") == 0) {
        if (count != 2) return reject(command, "expected_count");
        std::size_t expected = 0;
        if (!pointCount(tokens[1], expected)) return reject(command, "invalid_count");
        if (uploading_) return reject(command, "upload_in_progress");
        if (!sink.stop()) return reject(command, "stop_failed");
        cancelUpload();
        expected_ = expected;
        uploading_ = true;
        return accept(command, expected_);
    }

    if (std::strcmp(command, "ROUTE_POINT") == 0) {
        if (count != 3) return reject(command, "expected_xy");
        if (!uploading_) return reject(command, "no_upload");
        if (received_ >= expected_) return reject(command, "too_many_points");
        tree1_embedded::Point point{};
        if (!coordinate(tokens[1], point.x) || !coordinate(tokens[2], point.y))
            return reject(command, "invalid_coordinate");
        if (received_ > 0) {
            const auto previous = points_[received_ - 1];
            const double length = route_length_
                + std::hypot(point.x - previous.x, point.y - previous.y);
            // One nanometre of tolerance only accounts for floating-point sums.
            if (!std::isfinite(length) || length > kMaxRouteLengthMetres + 1e-9)
                return reject(command, "route_too_long");
            route_length_ = length;
            if (std::hypot(point.x - points_[0].x, point.y - points_[0].y) > 1e-12)
                distinct_ = true;
        }
        points_[received_++] = point;
        return accept(command, received_);
    }

    if (std::strcmp(command, "ROUTE_END") == 0) {
        if (count != 1) return reject(command, "unexpected_parameters");
        if (!uploading_) return reject(command, "no_upload");
        if (received_ != expected_) return reject(command, "incomplete_upload");
        if (!distinct_) return reject(command, "distinct_points_required");
        if (!sink.commitRoute(points_, received_)) return reject(command, "commit_failed");
        const std::size_t committed = received_;
        cancelUpload();
        return accept(command, committed);
    }

    if (std::strcmp(command, "PING") == 0) {
        if (count != 1) return reject(command, "unexpected_parameters");
        return accept(command);
    }
    if (std::strcmp(command, "STOP") == 0) {
        if (count != 1) return reject(command, "unexpected_parameters");
        cancelUpload();
        if (!sink.stop()) return reject(command, "stop_failed");
        return accept(command);
    }
    if (std::strcmp(command, "START") == 0) {
        if (count != 1) return reject(command, "unexpected_parameters");
        if (uploading_) return reject(command, "upload_in_progress");
        if (!sink.start()) return reject(command, "start_failed");
        return accept(command);
    }
    if (std::strcmp(command, "RESET_POSE") == 0) {
        if (count != 1) return reject(command, "unexpected_parameters");
        if (uploading_) return reject(command, "upload_in_progress");
        if (!sink.stop()) return reject(command, "stop_failed");
        if (!sink.resetPose()) return reject(command, "reset_failed");
        return accept(command);
    }
    return reject("INPUT", "unknown_command");
}

}  // namespace route_commands
