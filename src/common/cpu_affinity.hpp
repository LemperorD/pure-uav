#pragma once

#include <cctype>
#include <cerrno>
#include <charconv>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string_view>

#if defined(__linux__)
#include <sched.h>
#endif

namespace pure_uav {

#if defined(__linux__)
inline bool parseCpuSet(std::string_view spec, cpu_set_t& out) noexcept {
    CPU_ZERO(&out);
    int added = 0;
    size_t pos = 0;
    while (pos < spec.size()) {
        while (pos < spec.size() && (std::isspace(static_cast<unsigned char>(spec[pos])) != 0 || spec[pos] == ',')) {
            ++pos;
        }
        if (pos >= spec.size()) {
            break;
        }
        size_t next = pos;
        while (next < spec.size() && spec[next] != ',' && std::isspace(static_cast<unsigned char>(spec[next])) == 0) {
            ++next;
        }
        const std::string_view token = spec.substr(pos, next - pos);
        pos = next;

        const size_t dash = token.find('-');
        int lo = -1;
        int hi = -1;
        if (dash == std::string_view::npos) {
            const auto res = std::from_chars(token.data(), token.data() + token.size(), lo);
            if (res.ec != std::errc{} || res.ptr != token.data() + token.size()) {
                return false;
            }
            hi = lo;
        } else {
            const std::string_view left = token.substr(0, dash);
            const std::string_view right = token.substr(dash + 1);
            if (left.empty() || right.empty()) {
                return false;
            }
            const auto r1 = std::from_chars(left.data(), left.data() + left.size(), lo);
            const auto r2 = std::from_chars(right.data(), right.data() + right.size(), hi);
            if (r1.ec != std::errc{} || r1.ptr != left.data() + left.size() ||
                r2.ec != std::errc{} || r2.ptr != right.data() + right.size()) {
                return false;
            }
        }
        if (lo < 0 || hi < lo || hi >= CPU_SETSIZE) {
            return false;
        }
        for (int cpu = lo; cpu <= hi; ++cpu) {
            CPU_SET(cpu, &out);
            ++added;
        }
    }
    return added > 0;
}
#endif

inline bool applyCpuAffinityFromEnv(const char* env_name, const char* tag) noexcept {
    if (env_name == nullptr) {
        return false;
    }
    const char* raw = std::getenv(env_name);
    if (raw == nullptr || raw[0] == '\0') {
        return false;
    }
#if defined(__linux__)
    cpu_set_t set;
    if (!parseCpuSet(raw, set)) {
        std::fprintf(stderr, "[CPU] %s 忽略无效的 %s='%s'\n", tag ? tag : "proc", env_name, raw);
        return false;
    }
    if (sched_setaffinity(0, sizeof(set), &set) != 0) {
        std::fprintf(stderr, "[CPU] %s 绑核失败 (%s='%s'): %s\n", tag ? tag : "proc", env_name, raw, std::strerror(errno));
        return false;
    }
    std::fprintf(stderr, "[CPU] %s 已绑核: %s (%s)\n", tag ? tag : "proc", raw, env_name);
    return true;
#else
    (void)tag;
    return false;
#endif
}

}  // namespace pure_uav
