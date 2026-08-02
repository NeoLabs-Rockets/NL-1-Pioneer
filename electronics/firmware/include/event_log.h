#pragma once
#include "types.h"

namespace EventLog {

void begin();

// Queue an event for asynchronous write to events.jsonl (non-blocking).
// Storage task drains the queue; if full, increments drop counter.
void emit(EventType type, const char* message, const char* extra_json = nullptr);

void emitStateChange(MissionState from, MissionState to, const char* cause);
void emitCommand(const char* cmd, uint32_t ext_msg_id, const char* payload,
                 CmdResult result, const char* detail = nullptr);

uint32_t droppedCount();

// Called by storage task only.
bool pop(char* out, size_t out_len);

}  // namespace EventLog
