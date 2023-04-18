#include "DumptruckUltra.hpp"
#include "FreeRTOSConfig.h"
#include "Motor.hpp"
#include "fsm/DumptruckUltra.hpp"
#include "i2cBusManager.hpp"
#include "imu.hpp"
#include "proc_setup.hpp"

namespace Logic {
namespace FSM {
using namespace Hardware;
using namespace Logic;

DumptruckUltra::DumptruckUltra() : currState{INITIAL_STATE} {
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
    FSMState nextState = stateActionMap[currState]();
    currState = nextState;
}

void DumptruckUltra::addToStateTable(FSMState state, std::function<void()> stateAction) {
    stateActionMap[state] = stateAction;
}
} // namespace FSM
} // namespace Logic
