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
    enum class FSMState {
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
    void addToStateTable(FSMState state, std::function<FSMState()> stateAction);

private:
    // Each action function should return the next state
    // Could be the same state as current if there is no state change
    std::map<FSMState, std::function<FSMState()>> stateActionMap;
    auto fsmTask() -> void;
    FSMState currState;
    TaskHandle_t fsmTaskHandle;

    static constexpr FSMState INITIAL_STATE{FSMState::INIT};
};
} // namespace FSM
} // namespace Logic