#include "pure_math_tools.hpp"
#include <iostream>

int main() {
    double a = 1.0;
    double b = 2.0;

    double result_a = ReLU(a);
    std::cout << "Result of ReLU: " << result_a << std::endl;

    double result_b = SiLU(b);
    std::cout << "Result of SiLU: " << result_b << std::endl;

    return 0;
}