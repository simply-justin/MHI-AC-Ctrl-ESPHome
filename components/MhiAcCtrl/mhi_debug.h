#pragma once
#include "esphome/core/log.h"
#include <cstring>

static const char *const MHI_DEBUG_RAW_TAG = "mhi.raw";
#define MHI_DEBUG_RAW true   // <-- zet op false als je klaar bent

inline void mhi_debug_raw(const uint8_t *buf, size_t len) {
  if (!MHI_DEBUG_RAW) return;
  char hex[300];
  hex[0] = 0;
  for (size_t i = 0; i < len && i < sizeof(hex) - 4; i++) {
    sprintf(hex + strlen(hex), "%02X ", buf[i]);
  }
  ESP_LOGI(MHI_DEBUG_RAW_TAG, "RAW[%u]: %s", (unsigned)len, hex);
}
