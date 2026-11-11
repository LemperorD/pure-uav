#include "mission/mission_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace pure_uav::app::mission {
bool loadRuntimeConfig(const std::string& path, RuntimeConfig& out, std::string& error) {
    try {
        RuntimeConfig config;
        const auto root = YAML::LoadFile(path);
        if (!root.IsMap() || root.size() != 1 || !root["mission"].IsMap()) {
            throw std::invalid_argument("expected a single mission mapping");
        }
        auto& m = config.mission;
        const std::pair<const char*, double*> doubles[] = {
            {"takeoff_height_m", &m.takeoff_height_m},           {"position_tolerance_m", &m.position_tolerance_m},   {"settled_speed_mps", &m.settled_speed_mps},     {"settle_time_s", &m.settle_time_s},         {"pose_max_age_s", &m.pose_max_age_s},       {"target_max_age_s", &m.target_max_age_s},
            {"target_lost_timeout_s", &m.target_lost_timeout_s}, {"min_target_confidence", &m.min_target_confidence}, {"preflight_timeout_s", &m.preflight_timeout_s}, {"setup_timeout_s", &m.setup_timeout_s},     {"takeoff_timeout_s", &m.takeoff_timeout_s}, {"search_timeout_s", &m.search_timeout_s},
            {"strike_timeout_s", &m.strike_timeout_s},           {"return_timeout_s", &m.return_timeout_s},           {"landing_timeout_s", &m.landing_timeout_s},     {"mission_timeout_s", &m.mission_timeout_s}, {"request_retry_s", &m.request_retry_s},     {"prestream_s", &config.prestream_s},
            {"max_setpoint_gap_s", &config.max_setpoint_gap_s}};
        for (const auto& entry : root["mission"]) {
            const auto key = entry.first.as<std::string>();
            bool known = false;
            for (const auto& field : doubles) {
                if (key == field.first) {
                    *field.second = entry.second.as<double>();
                    known = true;
                    break;
                }
            }
            if (key == "target_class") {
                m.target_class = entry.second.as<int>();
                known = true;
            }
            if (key == "target_confirm_frames") {
                const int frames = entry.second.as<int>();
                if (frames < 1 || frames > 100) {
                    throw std::invalid_argument("target_confirm_frames must be in 1..100");
                }
                m.target_confirm_frames = static_cast<unsigned>(frames);
                known = true;
            }
            if (!known) {
                throw std::invalid_argument("unknown mission setting: " + key);
            }
        }
        const MissionController validate(m);
        (void)validate;
        if (m.pose_max_age_s > std::numeric_limits<int>::max() / 1000.0 || m.target_max_age_s > 1e9 || !std::isfinite(config.prestream_s) || !std::isfinite(config.max_setpoint_gap_s) || config.max_setpoint_gap_s <= 0 || config.prestream_s <= config.max_setpoint_gap_s ||
            config.prestream_s >= m.setup_timeout_s) {
            throw std::invalid_argument("invalid mission prestream_s or max_setpoint_gap_s");
        }
        out = config;
        error.clear();
        return true;
    } catch (const std::exception& exception) {
        error = path + ": " + exception.what();
        return false;
    }
}
}  // namespace pure_uav::app::mission
