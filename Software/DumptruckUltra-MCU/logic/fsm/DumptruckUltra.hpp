#pragma once

#include "DeadReckoning.hpp"
#include "DrivingAlgorithm.hpp"
#include "Motor.hpp"
#include "i2cBusManager.hpp"
#include "imu.hpp"
#include <functional>
#include <map>

namespace Logic {
namespace FSM {
class DumptruckUltra {
    enum class FSMStates {
        INIT,
        DRIVE_TO_SEARCH,
        LOCAL_SEARCH,
        DISPENSE,
        APPROACH,
        PICKUP,
        DRIVE_TO_START
    };

public:
    DumptruckUltra();
    void addToStateTable(FSMStates state, std::function<void()> stateAction);

private:
    std::map<FSMStates, std::function<FSMStates()>> stateActionMap;
    auto fsmTask() -> void;
    FSMStates currState;
};
} // namespace FSM
} // namespace Logic