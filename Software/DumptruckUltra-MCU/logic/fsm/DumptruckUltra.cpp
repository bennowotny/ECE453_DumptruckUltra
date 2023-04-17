#include "DumptruckUltra.hpp"
#include "Motor.hpp"
#include "fsm/DumptruckUltra.hpp"
#include "i2cBusManager.hpp"
#include "imu.hpp"
#include "proc_setup.hpp"

namespace Logic {
namespace FSM {
using namespace Hardware;
using namespace Logic;

DumptruckUltra::DumptruckUltra() {
}

auto DumptruckUltra::fsmTask() -> void {
    FSMStates nextState = stateActionMap.find(currState)->first;
    currState = nextState;
}
} // namespace FSM
} // namespace Logic
