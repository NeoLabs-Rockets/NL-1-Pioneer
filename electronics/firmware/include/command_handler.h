#pragma once
#include <stdint.h>
#include <stddef.h>
#include "types.h"

namespace CommandHandler {

// Parse JSON command body (Launch-System style: {"cmd":"...","sid":"...","seq":N,...})
// Returns result; writes ACK/NACK response into response_out if provided.
CmdResult handleJson(const char* body, char* response_out, size_t response_len);

// Idempotency cache for message IDs / seq
bool alreadyProcessed(uint32_t msg_id);
void rememberProcessed(uint32_t msg_id);

}  // namespace CommandHandler
