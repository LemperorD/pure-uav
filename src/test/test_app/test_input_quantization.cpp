#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "input_quantization.hpp"

using namespace pure_uav::detection;
namespace {
void check(bool condition, const char* reason) {
    if (!condition) throw std::runtime_error(reason);
}
}  // namespace
int main() {
    try {
        std::array<uint8_t, 256> lut{};
        std::string error;
        check(buildInputLut(InputElement::uint8, InputQuantization::none, 0, 0, 0, lut, error), "UINT8/NONE");
        for (int pixel = 0; pixel < 256; ++pixel) check(lut[static_cast<size_t>(pixel)] == pixel, "UINT8/NONE must preserve every pixel");
        const auto previous = lut;
        check(!buildInputLut(InputElement::int8, InputQuantization::none, 0, 0, 0, lut, error) && !error.empty() && lut == previous, "INT8/NONE must fail without changing LUT");
        check(buildInputLut(InputElement::fp32, InputQuantization::none, 0, 0, 0, lut, error) && lut == previous, "FP32 must bypass LUT");
        check(buildInputLut(InputElement::uint8, InputQuantization::asymmetric, 1.0f / 255.0f, 0, 0, lut, error), "UINT8 asymmetric");
        for (int pixel = 0; pixel < 256; ++pixel) check(lut[static_cast<size_t>(pixel)] == pixel, "UINT8 asymmetric identity scale");
        check(buildInputLut(InputElement::int8, InputQuantization::asymmetric, 1.0f / 255.0f, -128, 0, lut, error), "INT8 asymmetric");
        check(lut[0] == 128 && lut[128] == 0 && lut[255] == 127, "INT8 byte representation");
        check(buildInputLut(InputElement::uint8, InputQuantization::dynamic_fixed, 0, 0, 7, lut, error) && lut[0] == 0 && lut[255] == 128, "UINT8 dynamic fixed");
        check(buildInputLut(InputElement::int8, InputQuantization::dynamic_fixed, 0, 0, 8, lut, error) && lut[255] == 127, "INT8 saturation");
        check(buildInputLut(InputElement::uint8, InputQuantization::dynamic_fixed, 0, 0, -1, lut, error), "negative fixed point must not use an undefined negative bit shift");
        check(!buildInputLut(InputElement::uint8, InputQuantization::asymmetric, NAN, 0, 0, lut, error), "NaN scale");
        check(!buildInputLut(InputElement::uint8, InputQuantization::asymmetric, 0, 0, 0, lut, error), "zero scale");
        check(!buildInputLut(InputElement::uint8, InputQuantization::dynamic_fixed, 0, 0, 32, lut, error), "invalid fixed point");
        std::puts("PASS: UINT8/NONE identity, INT8/NONE rejection, FP32 bypass, asymmetric/fixed-point LUTs");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
