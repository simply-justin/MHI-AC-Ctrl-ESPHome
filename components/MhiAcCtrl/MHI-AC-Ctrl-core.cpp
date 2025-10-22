// MHI-AC-Ctrol-core
// implements the core functions (read & write SPI)

#include "MHI-AC-Ctrl-core.h"
#include "esphome/core/log.h"
#include <cstring>

static const char *const MHI_RAW_TAG = "mhi.raw";
#define MHI_DEBUG_RAW true  // zet op false als je klaar bent

inline void mhi_log_raw(const uint8_t *buf, size_t len) {
  if (!MHI_DEBUG_RAW) return;
  char hex[300];
  hex[0] = 0;
  for (size_t i = 0; i < len && i < sizeof(hex) - 4; i++) {
    sprintf(hex + strlen(hex), "%02X ", buf[i]);
  }
  ESP_LOGI(MHI_RAW_TAG, "RAW[%u]: %s", (unsigned)len, hex);
}

uint16_t calc_checksum(byte* frame) {
  uint16_t checksum = 0;
  for (int i = 0; i < CBH; i++)
    checksum += frame[i];
  return checksum;
}

uint16_t calc_checksumFrame33(byte* frame) {
  uint16_t checksum = 0;
  for (int i = 0; i < CBL2; i++)
    checksum += frame[i];
  return checksum;
}

void MHI_AC_Ctrl_Core::reset_old_values() {  // used e.g. when MQTT connection to broker is lost, to re-output data
  // old status
  status_power_old = 0xff;
  status_mode_old = 0xff;
  status_fan_old = 0xff;
  status_vanes_old = 0xff;
  status_troom_old = 0xfe;
  status_tsetpoint_old = 0x00;
  status_errorcode_old = 0xff;
  status_vanesLR_old = 0xff;
  status_3Dauto_old = 0xff;

  // old operating data
  op_kwh_old = 0xffff;
  op_mode_old = 0xff;
  op_settemp_old = 0xff;
  op_return_air_old = 0xff;
  op_iu_fanspeed_old = 0xff;
  op_thi_r1_old = 0x00;
  op_thi_r2_old = 0x00;
  op_thi_r3_old = 0x00;
  op_total_iu_run_old = 0;
  op_outdoor_old = 0xff;
  op_tho_r1_old = 0x00;
  op_total_comp_run_old = 0;
  op_ct_old = 0xff;
  op_tdsh_old = 0xff;
  op_protection_no_old = 0xff;
  op_ou_fanspeed_old = 0xff;
  op_defrost_old = 0x00;
  op_comp_old = 0xffff;
  op_td_old  = 0x00;
  op_ou_eev1_old = 0xffff;
}

void MHI_AC_Ctrl_Core::init() {
  //MeasureFrequency(m_cbiStatus);
  pinMode(SCK_PIN, INPUT);
  pinMode(MOSI_PIN, INPUT);
  pinMode(MISO_PIN, OUTPUT);
  MHI_AC_Ctrl_Core::reset_old_values();
}

void MHI_AC_Ctrl_Core::set_power(boolean power) {
  new_Power = 0b10 | power;
  pending_cmd_ = true;   // <-- PATCH
}

void MHI_AC_Ctrl_Core::set_mode(ACMode mode) {
  new_Mode = 0b00100000 | mode;
  pending_cmd_ = true;   // <-- PATCH
}

void MHI_AC_Ctrl_Core::set_tsetpoint(uint tsetpoint) {
  new_Tsetpoint = 0b10000000 | tsetpoint;
  pending_cmd_ = true;   // <-- PATCH
}

void MHI_AC_Ctrl_Core::set_fan(uint fan) {
  new_Fan = 0b00001000 | fan;
  pending_cmd_ = true;   // <-- PATCH
}

void MHI_AC_Ctrl_Core::set_3Dauto(AC3Dauto Dauto) {
  new_3Dauto = 0b00001010 | Dauto;
  pending_cmd_ = true;   // <-- PATCH
}

void MHI_AC_Ctrl_Core::set_vanes(uint vanes) {
  if (vanes == vanes_swing) {
    new_Vanes0 = 0b11000000; // enable swing
  }
  else {
    new_Vanes0 = 0b10000000; // disable swing
    new_Vanes1 = 0b10000000 | ((vanes - 1) << 4);
  }

  pending_cmd_ = true;   // <-- PATCH
}

void MHI_AC_Ctrl_Core::set_vanesLR(uint vanesLR) {
  if (vanesLR == vanesLR_swing) {
    new_VanesLR0 = 0b00001011; // enable swing
  }
  else {
    new_VanesLR0 = 0b00001010; // disable swing
    new_VanesLR1 = 0b00010000 | (vanesLR - 1);
  }

  pending_cmd_ = true;   // <-- PATCH
}

void MHI_AC_Ctrl_Core::request_ErrOpData() {
  request_erropData = true;
}

void MHI_AC_Ctrl_Core::set_troom(byte troom) {
  //Serial.printf("MHI_AC_Ctrl_Core::set_troom %i\n", troom);
  new_Troom = troom;
}

float MHI_AC_Ctrl_Core::get_troom_offset() {
  return Troom_offset;
}

void MHI_AC_Ctrl_Core::set_troom_offset(float offset) {
  Troom_offset = offset;
}

void MHI_AC_Ctrl_Core::set_frame_size(byte framesize) {
  if (framesize == 20 || framesize == 33) {
    frameSize = framesize;
    // --- PATCH: 33B commands aan wanneer 33 is gekozen
    wf_rac_enabled_ = (framesize == 33);
  }
}

inline bool mhi_is_valid_header(const uint8_t *buf) {
  // MHI signature: 6D 80 04
  return (buf[0] == 0x6D && buf[1] == 0x80 && buf[2] == 0x04);
}

int MHI_AC_Ctrl_Core::loop(uint max_time_ms) {
  const byte opdataCnt = sizeof(opdata) / sizeof(byte) / 2;
  static byte opdataNo = 0;
  const unsigned long startMillis = millis();

  byte MOSI_byte = 0;
  bool new_datapacket_received = false;
  static byte erropdataCnt = 0;
  static bool doubleframe = false;
  static int frame = 1;

  static byte MOSI_frame[33];
  static byte MISO_frame[] = {
    0xA9, 0x00, 0x07, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,
    0xff, 0xff, 0x22
  };

  static uint call_counter = 0;
  static unsigned long lastTroomInternalMillis = 0;

  if (frameSize == 33) MISO_frame[0] = 0xAA;

  call_counter++;

  // Wacht op stabiele SCK-high om frame start te detecteren
  unsigned long sckHighStart = millis();
  while (millis() - sckHighStart < 5) {
    if (!digitalRead(SCK_PIN)) sckHighStart = millis();
    if (millis() - startMillis > max_time_ms) return err_msg_timeout_SCK_low;
  }

  // Volgende MISO frame opbouwen
  doubleframe = !doubleframe;
  MISO_frame[DB14] = doubleframe << 2;

  // OpData cyclus spreiden
  if ((frame > (NoFramesPerOpDataCycle / opdataCnt)) && doubleframe) frame = 1;

  if (frame++ <= 2) {
    if (doubleframe) {
      if (erropdataCnt == 0) {
        MISO_frame[DB6] = pgm_read_word(opdata + opdataNo);
        MISO_frame[DB9] = pgm_read_word(opdata + opdataNo) >> 8;
        opdataNo = (opdataNo + 1) % opdataCnt;
      }
    }
  } else {
    MISO_frame[DB6] = 0x80;
    MISO_frame[DB9] = 0xff;
  }

  if (doubleframe) {
    // default leegmaken
    MISO_frame[DB0] = 0x00;
    MISO_frame[DB1] = 0x00;
    MISO_frame[DB2] = 0x00;

    // Commands meesturen (legacy + 33B velden)
    MISO_frame[DB0] |= new_Power;     new_Power = 0;
    MISO_frame[DB0] |= new_Mode;      new_Mode = 0;
    MISO_frame[DB2]  = new_Tsetpoint; new_Tsetpoint = 0;
    MISO_frame[DB1] |= new_Fan;       new_Fan = 0;
    MISO_frame[DB0] |= new_Vanes0;    new_Vanes0 = 0;
    MISO_frame[DB1] |= new_Vanes1;    new_Vanes1 = 0;

    if (frameSize == 33) {
      // Extra command bytes voor WF-RAC
      MISO_frame[DB16] = new_VanesLR1; new_VanesLR1 = 0;
      MISO_frame[DB17] = (new_VanesLR0 | new_3Dauto);
      new_VanesLR0 = 0;
      new_3Dauto   = 0;
    }

    if (erropdataCnt > 0) {
      MISO_frame[DB6] = 0x80;
      MISO_frame[DB9] = 0xff;
      erropdataCnt--;
    }

    if (request_erropData) {
      MISO_frame[DB6] = 0x80;
      MISO_frame[DB9] = 0x45;
      request_erropData = false;
    }
  }

  // Externe/ interne kamertemperatuur byte
  MISO_frame[DB3] = new_Troom;

  // Checksums op OUT (MISO) correct zetten
  uint16_t chks = calc_checksum(MISO_frame);
  MISO_frame[CBH] = highByte(chks);
  MISO_frame[CBL] = lowByte(chks);

  if (frameSize == 33) {
    chks = calc_checksumFrame33(MISO_frame);
    MISO_frame[CBL2] = lowByte(chks);
  }

  // Bit-bang transfer van frameSize bytes
  for (uint8_t byte_cnt = 0; byte_cnt < frameSize; byte_cnt++) {
    MOSI_byte = 0;
    byte bit_mask = 1;
    for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
      unsigned long sckWait = millis();
      while (digitalRead(SCK_PIN)) {
        if (millis() - startMillis > max_time_ms) return err_msg_timeout_SCK_high;
      }
      digitalWrite(MISO_PIN, (MISO_frame[byte_cnt] & bit_mask) ? 1 : 0);
      while (!digitalRead(SCK_PIN)) {
        if (millis() - sckWait > max_time_ms) return err_msg_timeout_SCK_high;
      }
      if (digitalRead(MOSI_PIN)) MOSI_byte += bit_mask;
      bit_mask <<= 1;
    }
    if (MOSI_frame[byte_cnt] != MOSI_byte) {
      new_datapacket_received = true;
      MOSI_frame[byte_cnt] = MOSI_byte;
    }
  }

  // **Geen** harde fouten meer op MOSI signature/checksum – alleen loggen
  bool header_ok = ((MOSI_frame[SB0] & 0xfe) == 0x6c) && (MOSI_frame[SB1] == 0x80) && (MOSI_frame[SB2] == 0x04);
  if (!header_ok) {
    ESP_LOGW(MHI_RAW_TAG, "Header mismatch (MOSI): %02X %02X %02X", MOSI_frame[SB0], MOSI_frame[SB1], MOSI_frame[SB2]);
  } else {
    // (optioneel): checksum berekenen voor debug
    uint16_t cs_mosi = calc_checksum(MOSI_frame);
    uint16_t cs_mosi_frame = (MOSI_frame[CBH] << 8) | MOSI_frame[CBL];
    if (cs_mosi != cs_mosi_frame) {
      ESP_LOGW(MHI_RAW_TAG, "Checksum mismatch (MOSI): got=%04X calc=%04X", cs_mosi_frame, cs_mosi);
    }
  }

  if (new_datapacket_received) {
    // RAW dump voor analyse
    mhi_log_raw(MOSI_frame, frameSize);

    // ====== Spam-friendly publish: ALTIJD uitsturen, niet alleen bij verandering ======

    // 33B extra status
    if (frameSize == 33) {
      byte vanesLRtmp = (MOSI_frame[DB16] & 0x07) + ((MOSI_frame[DB17] & 0x01) << 4);
      m_cbiStatus->cbiStatusFunction(status_vanesLR, (vanesLRtmp & 0x10) ? vanesLR_swing : ((vanesLRtmp & 0x07) + 1));
      m_cbiStatus->cbiStatusFunction(status_3Dauto, MOSI_frame[DB17] & 0x04);
    }

    // Mode/Power/Fan/Vanes/Troom/Tsetpoint/Error altijd pushen
    m_cbiStatus->cbiStatusFunction(status_mode,      MOSI_frame[DB0] & 0x1c);
    m_cbiStatus->cbiStatusFunction(status_power,     MOSI_frame[DB0] & 0x01);
    m_cbiStatus->cbiStatusFunction(status_fan,       MOSI_frame[DB1] & 0x07);

    uint vanestmp = (MOSI_frame[DB0] & 0xc0) + ((MOSI_frame[DB1] & 0xB0) >> 4);
    m_cbiStatus->cbiStatusFunction(status_vanes, (vanestmp & 0x40) ? vanes_swing : ((vanestmp & 0x03) + 1));

    // Troom met anti-jitter zoals origineel
    if (MISO_frame[DB3] != 0xff) {
      m_cbiStatus->cbiStatusFunction(status_troom, MOSI_frame[DB3]);
      lastTroomInternalMillis = 0;
    } else if ((unsigned long)(millis() - lastTroomInternalMillis) > minTimeInternalTroom) {
      lastTroomInternalMillis = millis();
      m_cbiStatus->cbiStatusFunction(status_troom, MOSI_frame[DB3]);
    }

    m_cbiStatus->cbiStatusFunction(status_tsetpoint,  MOSI_frame[DB2]);
    m_cbiStatus->cbiStatusFunction(status_errorcode,  MOSI_frame[DB4]);

    // OpData – altijd pushen (niet alleen bij verandering)
    const bool mosi_is_op = ((MOSI_frame[DB10] & 0x30) == 0x10);
    switch (MOSI_frame[DB9]) {
      case 0x94: if ((MOSI_frame[DB6] & 0x80) != 0 && mosi_is_op)
                    m_cbiStatus->cbiStatusFunction(opdata_kwh, (MOSI_frame[DB12] << 8) + MOSI_frame[DB11]); break;
      case 0x02: m_cbiStatus->cbiStatusFunction(mosi_is_op ? opdata_mode : erropdata_mode, (MOSI_frame[DB10] & 0x0f) << 2); break;
      case 0x05: if ((MOSI_frame[DB6] & 0x80) != 0) {
                    if (MOSI_frame[DB10] == 0x13) m_cbiStatus->cbiStatusFunction(opdata_tsetpoint, MOSI_frame[DB11]);
                    else if (MOSI_frame[DB10] == 0x33) m_cbiStatus->cbiStatusFunction(erropdata_tsetpoint, MOSI_frame[DB11]);
                 } break;
      case 0x81: if ((MOSI_frame[DB6] & 0x80) != 0) {
                    if ((MOSI_frame[DB10] & 0x30) == 0x20) m_cbiStatus->cbiStatusFunction(opdata_thi_r1, MOSI_frame[DB11]);
                    else m_cbiStatus->cbiStatusFunction(erropdata_thi_r1, MOSI_frame[DB11]);
                 } else {
                    if (mosi_is_op) m_cbiStatus->cbiStatusFunction(opdata_thi_r2, MOSI_frame[DB11]);
                    else m_cbiStatus->cbiStatusFunction(erropdata_thi_r2, MOSI_frame[DB11]);
                 } break;
      case 0x87: if ((MOSI_frame[DB6] & 0x80) != 0)
                    (mosi_is_op ? m_cbiStatus->cbiStatusFunction(opdata_thi_r3, MOSI_frame[DB11])
                                : m_cbiStatus->cbiStatusFunction(erropdata_thi_r3, MOSI_frame[DB11])); break;
      case 0x80: if ((MOSI_frame[DB6] & 0x80) != 0) {
                    if ((MOSI_frame[DB10] & 0x30) == 0x20) m_cbiStatus->cbiStatusFunction(opdata_return_air, MOSI_frame[DB11]);
                    else m_cbiStatus->cbiStatusFunction(erropdata_return_air, MOSI_frame[DB11]);
                 } else {
                    if (mosi_is_op) m_cbiStatus->cbiStatusFunction(opdata_outdoor, MOSI_frame[DB11]);
                    else m_cbiStatus->cbiStatusFunction(erropdata_outdoor, MOSI_frame[DB11]);
                 } break;
      case 0x1f: if ((MOSI_frame[DB6] & 0x80) != 0)
                    (mosi_is_op ? m_cbiStatus->cbiStatusFunction(opdata_iu_fanspeed, MOSI_frame[DB10] & 0x0f)
                                : m_cbiStatus->cbiStatusFunction(erropdata_iu_fanspeed, MOSI_frame[DB10] & 0x0f));
                 else
                    (mosi_is_op ? m_cbiStatus->cbiStatusFunction(opdata_ou_fanspeed, MOSI_frame[DB10] & 0x0f)
                                : m_cbiStatus->cbiStatusFunction(erropdata_ou_fanspeed, MOSI_frame[DB10] & 0x0f)); break;
      case 0x1e: if ((MOSI_frame[DB6] & 0x80) != 0)
                    (mosi_is_op ? m_cbiStatus->cbiStatusFunction(opdata_total_iu_run, MOSI_frame[DB11])
                                : m_cbiStatus->cbiStatusFunction(erropdata_total_iu_run, MOSI_frame[DB11]));
                 else {
                    if (MOSI_frame[DB10] == 0x11) m_cbiStatus->cbiStatusFunction(opdata_total_comp_run, MOSI_frame[DB11]);
                    else m_cbiStatus->cbiStatusFunction(erropdata_total_comp_run, MOSI_frame[DB11]);
                 } break;
      case 0x82: if ((MOSI_frame[DB6] & 0x80) == 0)
                    (mosi_is_op ? m_cbiStatus->cbiStatusFunction(opdata_tho_r1, MOSI_frame[DB11])
                                : m_cbiStatus->cbiStatusFunction(erropdata_tho_r1, MOSI_frame[DB11])); break;
      case 0x11: if ((MOSI_frame[DB6] & 0x80) == 0)
                    (mosi_is_op ? m_cbiStatus->cbiStatusFunction(opdata_comp, ((MOSI_frame[DB10] << 8) | MOSI_frame[DB11]) & 0x0fff)
                                : m_cbiStatus->cbiStatusFunction(erropdata_comp, ((MOSI_frame[DB10] << 8) | MOSI_frame[DB11]) & 0x0fff)); break;
      case 0x85: if ((MOSI_frame[DB6] & 0x80) == 0)
                    (mosi_is_op ? m_cbiStatus->cbiStatusFunction(opdata_td, MOSI_frame[DB11])
                                : m_cbiStatus->cbiStatusFunction(erropdata_td, MOSI_frame[DB11])); break;
      case 0x90: if ((MOSI_frame[DB6] & 0x80) == 0)
                    (mosi_is_op ? m_cbiStatus->cbiStatusFunction(opdata_ct, MOSI_frame[DB11])
                                : m_cbiStatus->cbiStatusFunction(erropdata_ct, MOSI_frame[DB11])); break;
      case 0xb1: if ((MOSI_frame[DB6] & 0x80) == 0 && mosi_is_op)
                    m_cbiStatus->cbiStatusFunction(opdata_tdsh, MOSI_frame[DB11] / 2); break;
      case 0x7c: if ((MOSI_frame[DB6] & 0x80) == 0 && mosi_is_op)
                    m_cbiStatus->cbiStatusFunction(opdata_protection_no, MOSI_frame[DB11]); break;
      case 0x0c: if ((MOSI_frame[DB6] & 0x80) == 0 && mosi_is_op)
                    m_cbiStatus->cbiStatusFunction(opdata_defrost, MOSI_frame[DB10] & 0x01); break;
      case 0x13: if ((MOSI_frame[DB6] & 0x80) == 0)
                    (mosi_is_op ? m_cbiStatus->cbiStatusFunction(opdata_ou_eev1, (MOSI_frame[DB12] << 8) | MOSI_frame[DB11])
                                : m_cbiStatus->cbiStatusFunction(erropdata_ou_eev1, (MOSI_frame[DB12] << 8) | MOSI_frame[DB11])); break;
      case 0x45:
        if ((MOSI_frame[DB6] & 0x80) != 0) {
          if (MOSI_frame[DB10] == 0x11) m_cbiStatus->cbiStatusFunction(erropdata_errorcode, MOSI_frame[DB11]);
          else if (MOSI_frame[DB10] == 0x12) erropdataCnt = MOSI_frame[DB11] + 4;
        }
        break;
      default:
        m_cbiStatus->cbiStatusFunction(opdata_unknown, (MOSI_frame[DB10] << 8) | MOSI_frame[DB9]);
        break;
    }
  }

  return call_counter;
}
