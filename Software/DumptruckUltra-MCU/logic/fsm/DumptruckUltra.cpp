#include "DumptruckUltra.hpp"

#include "FreeRTOSConfig.h"
#include "RGBLED.hpp"
#include "fsm/DumptruckUltra.hpp"
#include "imu.hpp"
#include <utility>

namespace Logic {
namespace FSM {
using namespace Hardware;
using namespace Logic;

DumptruckUltra::DumptruckUltra(RGB_LED::RGBLED led) : led{led},
                                                      currState{INITIAL_STATE},
                                                      fsmTaskHandle{} {
    // Create FSM task
    xTaskCreate(
        [](void *obj) -> void { static_cast<DumptruckUltra *>(obj)->fsmTask(); },
        "DumpTruckUltraFSM",
        configMINIMAL_STACK_SIZE,
        this,
        tskIDLE_PRIORITY + 1,
        &fsmTaskHandle);
}

auto DumptruckUltra::fsmTask() -> void {
    led.turnOn();
    while (true) {
        led.setColor(stateActionMap.at(currState).color);
        FSMState nextState = stateActionMap[currState].stateAction();
        currState = nextState;
    }
}

void DumptruckUltra::addToStateTable(FSMState state, StateRepresentation repr) {
    stateActionMap[state] = std::move(repr);
}
} // namespace FSM
} // namespace Logic
