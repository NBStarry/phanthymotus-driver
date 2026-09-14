// Read-only SDK 5.1.1 probe. No ROS output, actions, heartbeat or configuration.
// Host/port are explicit; finite probes default to 10 samples, capped at 100.
// --stream-scan reads scans only and is supervised by navigation_sensors.py.
#include <rpos/robot_platforms/slamware_core_platform.h>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <thread>

using Clock = std::chrono::steady_clock;

static long long unix_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

static long long monotonic_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        Clock::now().time_since_epoch()).count();
}

static void number(std::ostream& out, double value) {
    if (std::isfinite(value)) out << value;
    else out << "null";  // Preserve invalidity; never fabricate a zero reading.
}

static void pose(std::ostream& out, const rpos::core::Pose& value) {
    out << '[';
    number(out, value.x()); out << ','; number(out, value.y()); out << ',';
    number(out, value.z()); out << ','; number(out, value.yaw()); out << ',';
    number(out, value.pitch()); out << ','; number(out, value.roll()); out << ']';
}

template <typename Read, typename Write>
static bool sample(const char* kind, int index, Read read, Write write) {
    const auto request_time = unix_ns();
    const auto request_mono = monotonic_ns();
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17) << "{\"endpoint\":\"" << kind
        << "\",\"poll_index\":" << index
        << ",\"request_unix_ns\":" << request_time
        << ",\"request_monotonic_ns\":" << request_mono;
    try {
        const auto value = read();
        const auto receive_time = unix_ns();
        const auto receive_mono = monotonic_ns();
        out << ",\"receive_unix_ns\":" << receive_time
            << ",\"receive_monotonic_ns\":" << receive_mono;
        write(out, value);
        out << '}';
        std::cout << out.str() << std::endl;
        return true;
    } catch (...) {
        // No partial JSON or vendor exception strings containing private hosts.
        std::cout << "{\"endpoint\":\"" << kind << "\",\"poll_index\":"
                  << index << ",\"error\":\"sdk_read_failed\"}" << std::endl;
        return false;
    }
}

static int integer(const char* value, int limit) {
    const std::string text(value);
    std::size_t used = 0;
    const int result = std::stoi(text, &used);
    if (used != text.size() || result < 1 || result > limit)
        throw std::invalid_argument("invalid integer");
    return result;
}

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cerr << "usage: navigation_sdk_probe HOST SDK_PORT [SAMPLES=10|--stream-scan]\n";
        return 2;
    }
    const bool stream_scan = argc == 4 && std::string(argv[3]) == "--stream-scan";
    int port, count;
    try {
        port = integer(argv[2], 65535);
        count = stream_scan ? 0 : (argc == 4 ? integer(argv[3], 100) : 10);
    } catch (...) {
        std::cerr << "invalid port or sample count\n";
        return 2;
    }
    try {
        auto platform = rpos::robot_platforms::SlamwareCorePlatform::connect(argv[1], port, 3000);
        bool ok = true;
        for (int i = 0; stream_scan || i < count; i = (i + 1) % 1000000000) {
            const auto cycle = Clock::now();
            ok &= sample("raw_scan", i, [&] { return platform.getRawLaserScan(); },
                [](std::ostream& out, const rpos::features::system_resource::LaserScan& scan) {
                    out << ",\"start_stamp_raw\":" << scan.getStartTimestamp()
                        << ",\"end_stamp_raw\":" << scan.getEndTimestamp()
                        << ",\"source_clock_domain\":null,\"source_time_unit\":null"
                        << ",\"scan_pose_xyz_yaw_pitch_roll\":";
                    if (scan.getHasPose()) pose(out, scan.getLaserPointsPose());
                    else out << "null";
                    out << ",\"points_angle_distance_valid\":[";
                    bool first = true;
                    for (const auto& p : scan.getLaserPoints()) {
                        if (!first) out << ',';
                        first = false;
                        out << '['; number(out, p.angle()); out << ',';
                        number(out, p.distance()); out << ',' << (p.valid() ? "true" : "false") << ']';
                    }
                    out << ']';
                });
            if (stream_scan) {
                if (!ok) break;
                std::this_thread::sleep_until(cycle + std::chrono::milliseconds(50));
                continue;
            }
            ok &= sample("odometry_pose", i, [&] { return platform.getOdoPose(); },
                [](std::ostream& out, const rpos::core::Pose& value) {
                    out << ",\"source_stamp_ns\":null,\"pose_xyz_yaw_pitch_roll\":";
                    pose(out, value);
                });
            ok &= sample("speed", i, [&] { return platform.getSpeed(); },
                [](std::ostream& out, const rpos::message::base::MotionRequest& value) {
                    out << ",\"source_stamp_ns\":null,\"actual_feedback_verified\":false,\"velocity_x_y_yaw\":[";
                    number(out, value.vx()); out << ','; number(out, value.vy()); out << ',';
                    number(out, value.omega()); out << ']';
                });
            if (i + 1 < count) std::this_thread::sleep_until(cycle + std::chrono::milliseconds(200));
        }
        platform.disconnect();
        return ok ? 0 : 2;
    } catch (...) {
        std::cerr << "SDK connection/session failed\n";
        return 2;
    }
}
