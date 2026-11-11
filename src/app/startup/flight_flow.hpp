#pragma once

#include <cstdint>

namespace pure_uav {
class Communicator;
}

namespace pure_uav::app {

class Logger;
class OffboardZeroStreamer;
struct MainConfig;

struct FlightState {
    std::uint64_t reconnect_count = 0;
    double ground_z = 0.0;
    bool airborne = false;
};

bool waitLink(Communicator& comm, const MainConfig& cfg, Logger& log, FlightState& state);
bool runPreflight(Communicator& comm, const MainConfig& cfg, Logger& log, const FlightState& state);
bool captureGroundReference(Communicator& comm, const MainConfig& cfg, Logger& log, FlightState& state);

bool arm(Communicator& comm, const MainConfig& cfg, Logger& log);
bool takeoff(Communicator& comm, const MainConfig& cfg, Logger& log, FlightState& state);
bool waitAltitude(Communicator& comm, const MainConfig& cfg, Logger& log, const FlightState& state);
bool hover(Communicator& comm, const MainConfig& cfg, Logger& log, const FlightState& state);

bool enterOffboard(Communicator& comm, const MainConfig& cfg, Logger& log, OffboardZeroStreamer& streamer);
bool holdOffboard(Communicator& comm, const MainConfig& cfg, Logger& log, OffboardZeroStreamer& streamer, const FlightState& state);

}  // namespace pure_uav::app
