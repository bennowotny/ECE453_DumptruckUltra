#include "fsmStates.hpp"
#include "DumptruckUltra.hpp"
#include "projdefs.h"

namespace Logic {
namespace FSM {

auto initStateAction() -> DumptruckUltra::FSMState {
    printf("%s", "Doing init state\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    return DumptruckUltra::FSMState::DRIVE_TO_SEARCH;
}

auto driveToSearchAction() -> DumptruckUltra::FSMState {
    printf("%s", "Doing driveToSearch state\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    return DumptruckUltra::FSMState::LOCAL_SEARCH;
}

auto localSearchAction() -> DumptruckUltra::FSMState {
    printf("%s", "Doing localSearch state\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    return DumptruckUltra::FSMState::APPROACH;
}

auto approachAction() -> DumptruckUltra::FSMState {
    printf("%s", "Doing approach state\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    return DumptruckUltra::FSMState::PICKUP;
}

auto pickupAction() -> DumptruckUltra::FSMState {
    printf("%s", "Doing pickup state\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    return DumptruckUltra::FSMState::DRIVE_TO_START;
}

auto driveToStartAction() -> DumptruckUltra::FSMState {
    printf("%s", "Doing driveToStart state\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    return DumptruckUltra::FSMState::DISPENSE;
}

auto dispenseAction() -> DumptruckUltra::FSMState {
    printf("%s", "Doing dispense state\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    return DumptruckUltra::FSMState::DRIVE_TO_SEARCH;
}

} // namespace FSM
} // namespace Logic