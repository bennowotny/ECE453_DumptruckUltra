#pragma once
#include "RGBLED.hpp"
#include "imu.hpp"
#include <functional>
#include <map>

namespace Logic {
namespace FSM {
class DumptruckUltra {
public:
    enum class FSMState {
        INIT,
        DRIVE_TO_SEARCH,
        LOCAL_SEARCH,
        APPROACH,
        PICKUP,
        DRIVE_TO_START,
        DISPENSE
    };

    explicit DumptruckUltra(Hardware::RGB_LED::RGBLED led);

    struct StateRepresentation {
        std::function<FSMState()> stateAction;
        Hardware::RGB_LED::Color color;
    };

    void addToStateTable(FSMState state, StateRepresentation repr);

private:
    // Each action function should return the next state
    // Could be the same state as current if there is no state change

    std::map<FSMState, StateRepresentation> stateActionMap;

    Hardware::RGB_LED::RGBLED led;

    auto fsmTask() -> void;
    FSMState currState;
    TaskHandle_t fsmTaskHandle;

    static constexpr FSMState INITIAL_STATE{FSMState::INIT};
};
} // namespace FSM
} // namespace Logic