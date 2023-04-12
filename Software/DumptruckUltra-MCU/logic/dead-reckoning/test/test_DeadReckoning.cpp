#include "DeadReckoning.hpp"
#include "cy_retarget_io.h"
#include "proc_setup.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>

auto main() -> int {
    Hardware::Processor::setupProcessor();

    Logic::DeadReckoning::DeadReckoning uut{};

    constexpr uint32_t TEST_UPDATE_PERIOD_MS{10};
    constexpr uint32_t MS_PER_S{1000};
    float elapsedTime{0};

    cy_retarget_io_init(P5_1, P5_0, 9600);

    while (true) {
        uut.sendGyroscopeMessage({.Gx = 0, .Gy = 0, .Gz = std::cos(elapsedTime), .Gts = static_cast<float>(TEST_UPDATE_PERIOD_MS) / MS_PER_S});
        const auto currentPosition{uut.getCurrentPose()};
        std::cout << elapsedTime << "," << std::cos(elapsedTime) << "," << currentPosition.heading << "\r\n"; // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg); required b/c embedded
        cyhal_system_delay_ms(TEST_UPDATE_PERIOD_MS);
        elapsedTime += static_cast<float>(TEST_UPDATE_PERIOD_MS) / MS_PER_S;
    }
}