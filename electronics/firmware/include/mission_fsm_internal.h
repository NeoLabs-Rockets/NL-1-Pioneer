#pragma once
#include "mission_fsm.h"
#include <stddef.h>

// Extra hooks used by main/storage (not part of public BLE API surface).
namespace MissionFsm {
bool consumeStartRecording(char* reason_out, size_t n);
bool consumeStopRecording();
void markRecordingActive();
void markRecordingStopped();
void markRecordingFailed();
void advanceFromBoot();
void advanceSelfTestOk();
}
