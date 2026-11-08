#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace pure_uav::detection {
enum class InputElement { uint8, int8, fp32 };
enum class InputQuantization { none, asymmetric, dynamic_fixed };

bool buildInputLut(InputElement element, InputQuantization quantization, float scale, int zero_point, int fixed_point, std::array<uint8_t, 256>& lut, std::string& error);
}  // namespace pure_uav::detection
