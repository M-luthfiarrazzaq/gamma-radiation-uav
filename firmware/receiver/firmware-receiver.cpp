/*
 * =============================================================================
 *  RECEIVER FIRMWARE — Ground Station Dongle
 * =============================================================================
 *
 *  Board    : ESP32-S3-N16R8 (custom PCB, USB dongle form factor)
 *  Author   : Muhammad Luthfi Ar-Razzaq (NIM 022200025)
 *  Institute: Politeknik Teknologi Nuklir Indonesia — BRIN
 *  Version  : 5.0
 *
 *  -------------------------------------------------------------------------
 *  WHAT THIS DOES
 *  -------------------------------------------------------------------------
 *  Bridges the airborne detector to the Node-RED ground station. It receives
 *  binary LoRa frames from the transmitter, validates them, and re-emits them
 *  as one JSON object per line on USB serial. In the other direction it accepts
 *  JSON commands from Node-RED and relays them over LoRa, retrying until the
 *  transmitter acknowledges or the attempt budget is exhausted.
 *
 *  Deliberately a thin translation layer: no interpretation, no state beyond
 *  the pending command. All measurement logic lives on the transmitter, so a
 *  result is complete when it leaves the aircraft rather than being assembled
 *  on the ground from fragments.
 *
 *  -------------------------------------------------------------------------
 *  SERIAL PROTOCOL — 115200 baud, newline delimited
 *  -------------------------------------------------------------------------
 *  Out (to Node-RED), one JSON object per line:
 *    {"type":"cont",  "id":.., "cpm":.., "usv":..,   "lat":.., "lon":..,
 *                     "alt":.., "rssi":.., "snr":..}
 *    {"type":"timed", "id":.., "dur_s":.., "pulses":.., "cpm":.., "usv_h":..,
 *                     "usv_tot":.., "lat":.., "lon":.., "alt":.., "rssi":..,
 *                     "snr":..}
 *    {"status":"cmd_sending"|"cmd_ack"|"cmd_timeout", "cmd":"timed"|"stop", ...}
 *    {"error":"..."}
 *    {"status":"receiver_ready", ...}          on boot
 *
 *  In (from Node-RED):
 *    {"cmd":"timed","duration":30}
 *    {"cmd":"stop"}
 *
 *  -------------------------------------------------------------------------
 *  CHANGES IN v5.0
 *  -------------------------------------------------------------------------
 *    - altSrc, gpsFix and sats removed from the packet struct and JSON output.
 *    - StaticJsonDocument replaced with JsonDocument (ArduinoJson v7).
 *    - Packet structs made identical to transmitter v5.0.
 *
 *  -------------------------------------------------------------------------
 *  PIN MAPPING — LoRa SX1278 / RA-02 over SPI
 *  -------------------------------------------------------------------------
 *      IO10   NSS/CS        IO12   SCK
 *      IO11   MOSI          IO13   MISO
 *      IO16   RST           IO15   DIO0
 *      IO48   On-board LED
 *
 *  -------------------------------------------------------------------------
 *  DEPENDENCIES
 *  -------------------------------------------------------------------------
 *    LoRa        — Sandeep Mistry    v0.8.0+
 *    ArduinoJson — Benoit Blanchon   v7.4.3+
 *
 *  -------------------------------------------------------------------------
 *  KNOWN ISSUES — identified during review, deliberately left unfixed here so
 *  that this build matches the firmware used to produce the thesis results.
 *  Each is marked inline with a "KNOWN ISSUE" comment at the relevant code.
 *  -------------------------------------------------------------------------
 *    1. onLoRaReceive() runs in interrupt context but is not marked IRAM_ATTR,
 *       and the buffers it writes are not declared volatile.
 *
 *    2. The JSON key "dur_s" carries milliseconds, not seconds. The key name is
 *       retained here because the Node-RED flow depends on it; anything
 *       consuming that field must divide by 1000.
 *
 *    3. LORA_TX_POWER is defined but setTxPower() is never called, so outbound
 *       commands transmit at the library default rather than the configured
 *       value. The transmitter does set it.
 *
 *    4. containsKey() is deprecated in ArduinoJson v7.
 * =============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>


// =============================================================================
//  PIN ASSIGNMENTS
// =============================================================================
#define LORA_CS    10
#define LORA_RST   16
#define LORA_DIO0  15
#define LORA_SCK   12
#define LORA_MOSI  11
#define LORA_MISO  13
#define LED_PIN    48


// =============================================================================
//  LORA CONFIGURATION — must match the transmitter exactly
// =============================================================================
#define LORA_FREQ       433E6
#define LORA_SF         10
#define LORA_BW         125E3
#define LORA_CR         8
#define LORA_TX_POWER   20      // see known issue 3 — defined but never applied


// =============================================================================
//  PACKET TYPE IDENTIFIERS — must match the transmitter
// =============================================================================
#define PKT_CONT    0x01    // TX -> RX  continuous measurement
#define PKT_TIMED   0x02    // TX -> RX  timed count result
#define PKT_ACK     0x03    // TX -> RX  command acknowledgement
#define PKT_CMD     0x10    // RX -> TX  operator command

#define CMD_TIMED   0x01    // start a timed count; param = duration in seconds
#define CMD_STOP    0x02    // abort the running timed count


// =============================================================================
//  RADIO PACKET STRUCTURES
//
//  Byte-for-byte identical to the transmitter's definitions. Any change on one
//  side must be mirrored on the other, or every frame will fail its CRC check.
// =============================================================================
#pragma pack(push, 1)

struct RadioPacketCont {          // 27 bytes
  uint8_t  pktType;
  uint32_t id;
  uint32_t cpm;
  float    doseRate;              // uSv/h
  float    lat;
  float    lon;
  float    alt;                   // metres
  uint16_t crc;
};

struct RadioPacketTimed {         // 39 bytes
  uint8_t  pktType;
  uint32_t id;
  uint32_t durationMs;            // milliseconds — see known issue 2
  uint32_t pulses;
  float    cpm;
  float    doseRate;              // uSv/h
  float    totalDose;             // uSv accumulated over the count
  float    lat;
  float    lon;
  float    alt;
  uint16_t crc;
};

struct AckPacket {                // 8 bytes
  uint8_t  pktType;
  uint8_t  cmdAcked;
  uint32_t param;
  uint16_t crc;
};

struct CmdPacket {                // 8 bytes
  uint8_t  pktType;
  uint8_t  cmd;
  uint32_t param;
  uint16_t crc;
};

#pragma pack(pop)


// =============================================================================
//  CRC-16/CCITT  (polynomial 0x1021, initial value 0xFFFF)
//
//  A second, independent check on top of the SX1278's hardware CRC. It covers
//  the struct contents, so a frame of the right length but the wrong shape is
//  rejected rather than silently misread as valid measurement data.
// =============================================================================
uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc <<= 1;
    }
  }
  return crc;
}

// Recomputes the CRC over a received packet and compares it against the value
// stored in the struct's trailing crc field.
template<typename T>
bool verifyCRC(const T& pkt) {
  uint16_t expected = crc16((const uint8_t*)&pkt, sizeof(T) - sizeof(uint16_t));
  const uint16_t* storedCRC =
    (const uint16_t*)((const uint8_t*)&pkt + sizeof(T) - sizeof(uint16_t));
  return *storedCRC == expected;
}

// Computes the CRC for an outbound packet before transmission.
template<typename T>
uint16_t calcStructCRC(const T& pkt) {
  return crc16((const uint8_t*)&pkt, sizeof(T) - sizeof(uint16_t));
}


// =============================================================================
//  RECEIVE BUFFER
//
//  Filled by the LoRa interrupt callback, drained by the main loop.
//
//  KNOWN ISSUE 1: everything below except packetReady is written from interrupt
//  context without being declared volatile. The compiler may cache these in
//  registers. All of them should be volatile, and the buffer should ideally be
//  double-buffered so a second packet arriving mid-parse cannot corrupt the one
//  being read.
// =============================================================================
#define RX_BUF_SIZE 64

volatile bool packetReady = false;
uint8_t  rxBuf[RX_BUF_SIZE];
int      rxLen  = 0;
int      rxRSSI = 0;      // received signal strength, dBm
float    rxSNR  = 0.0f;   // signal-to-noise ratio, dB

// --- Non-blocking LED ---
bool     ledPending = false;
uint32_t ledOnTime  = 0;

// --- Inbound command line buffer from Node-RED ---
String   serialCmdBuf = "";


// =============================================================================
//  LORA INITIALISATION
// =============================================================================
bool setupLoRa() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) return false;

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  // KNOWN ISSUE 3: LoRa.setTxPower(LORA_TX_POWER) is absent here, so outbound
  // commands go out at the library default rather than 20 dBm.
  LoRa.enableCrc();
  return true;
}


// =============================================================================
//  HANDLE A CONTINUOUS MEASUREMENT PACKET
//
//  Converts the binary struct to JSON and writes one line to Node-RED. RSSI and
//  SNR are attached from the radio rather than the packet, so link quality is
//  logged alongside every measurement — which is what made the range
//  characterisation possible after the fact.
// =============================================================================
void processCont(const uint8_t* buf, int len) {
  if (len < (int)sizeof(RadioPacketCont)) {
    Serial.println("{\"error\":\"cont_size\"}");
    return;
  }

  RadioPacketCont pkt;
  memcpy(&pkt, buf, sizeof(pkt));

  if (!verifyCRC(pkt)) {
    Serial.printf("{\"error\":\"crc_fail\",\"pktType\":\"cont\",\"rssi\":%d}\n", rxRSSI);
    return;
  }

  JsonDocument doc;
  doc["type"] = "cont";
  doc["id"]   = pkt.id;
  doc["cpm"]  = pkt.cpm;
  doc["usv"]  = pkt.doseRate;
  doc["lat"]  = pkt.lat;
  doc["lon"]  = pkt.lon;
  doc["alt"]  = pkt.alt;
  doc["rssi"] = rxRSSI;
  doc["snr"]  = rxSNR;

  String out;
  serializeJson(doc, out);
  Serial.println(out);
}


// =============================================================================
//  HANDLE A TIMED COUNT RESULT PACKET
//
//  KNOWN ISSUE 2: "dur_s" carries pkt.durationMs, which is milliseconds. The
//  key name is kept as-is because the Node-RED flow depends on it. Anything
//  reading this field must divide by 1000.
// =============================================================================
void processTimed(const uint8_t* buf, int len) {
  if (len < (int)sizeof(RadioPacketTimed)) {
    Serial.println("{\"error\":\"timed_size\"}");
    return;
  }

  RadioPacketTimed pkt;
  memcpy(&pkt, buf, sizeof(pkt));

  if (!verifyCRC(pkt)) {
    Serial.printf("{\"error\":\"crc_fail\",\"pktType\":\"timed\",\"rssi\":%d}\n", rxRSSI);
    return;
  }

  JsonDocument doc;
  doc["type"]    = "timed";
  doc["id"]      = pkt.id;
  doc["dur_s"]   = pkt.durationMs;   // milliseconds — see known issue 2
  doc["pulses"]  = pkt.pulses;
  doc["cpm"]     = pkt.cpm;
  doc["usv_h"]   = pkt.doseRate;
  doc["usv_tot"] = pkt.totalDose;
  doc["lat"]     = pkt.lat;
  doc["lon"]     = pkt.lon;
  doc["alt"]     = pkt.alt;
  doc["rssi"]    = rxRSSI;
  doc["snr"]     = rxSNR;

  String out;
  serializeJson(doc, out);
  Serial.println(out);
}


// =============================================================================
//  OUTBOUND COMMAND STATE
//
//  LoRa gives no delivery guarantee, so a command is held pending and re-sent
//  on a timer until the transmitter acknowledges it. Without this an operator
//  could press a button and never learn whether anything happened.
// =============================================================================
#define RETRY_INTERVAL  1500UL    // milliseconds between attempts
#define RETRY_MAX       10        // ~15 s total before giving up

struct PendingCmd {
  bool     active       = false;
  uint8_t  cmd          = 0;
  uint32_t param        = 0;
  uint32_t lastSentTime = 0;
  uint8_t  retryCount   = 0;
};
PendingCmd pendingCmd;


// =============================================================================
//  HANDLE AN ACKNOWLEDGEMENT FROM THE TRANSMITTER
//
//  Clears the pending command so retries stop, and tells Node-RED the command
//  was acted on rather than merely sent.
// =============================================================================
void processAck(const uint8_t* buf, int len) {
  if (len < (int)sizeof(AckPacket)) {
    Serial.println("{\"error\":\"ack_size\"}");
    return;
  }

  AckPacket pkt;
  memcpy(&pkt, buf, sizeof(pkt));

  if (!verifyCRC(pkt)) {
    Serial.println("{\"error\":\"crc_fail\",\"pktType\":\"ack\"}");
    return;
  }

  // Only clear the pending command if this acknowledges the command we sent.
  if (pendingCmd.active && pendingCmd.cmd == pkt.cmdAcked) {
    pendingCmd.active = false;

    JsonDocument doc;
    doc["status"] = "cmd_ack";
    doc["cmd"]    = (pkt.cmdAcked == CMD_TIMED) ? "timed" : "stop";
    if (pkt.param > 0) doc["param"] = pkt.param;

    String out;
    serializeJson(doc, out);
    Serial.println(out);
  }
}


// =============================================================================
//  PACKET ROUTER — dispatch on the first byte
// =============================================================================
void routePacket() {
  if (rxLen < 1) return;

  uint8_t pktType = rxBuf[0];

  switch (pktType) {
    case PKT_CONT:
      processCont(rxBuf, rxLen);
      break;
    case PKT_TIMED:
      processTimed(rxBuf, rxLen);
      break;
    case PKT_ACK:
      processAck(rxBuf, rxLen);
      break;
    default:
      // Report rather than discard silently: an unexpected type usually means
      // a firmware version mismatch between the two ends.
      Serial.printf("{\"error\":\"unknown_pktType\",\"got\":\"0x%02X\"}\n", pktType);
      break;
  }
}


// =============================================================================
//  TRANSMIT A COMMAND TO THE AIRCRAFT
//
//  Returns the radio to receive mode immediately afterwards so telemetry is not
//  missed while waiting for the acknowledgement.
// =============================================================================
void sendCommandLoRa(uint8_t cmd, uint32_t param) {
  CmdPacket pkt;
  pkt.pktType = PKT_CMD;
  pkt.cmd     = cmd;
  pkt.param   = param;
  pkt.crc     = calcStructCRC(pkt);

  LoRa.beginPacket();
  LoRa.write((const uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
  LoRa.receive();
}

void startPendingCmd(uint8_t cmd, uint32_t param) {
  pendingCmd.active       = true;
  pendingCmd.cmd          = cmd;
  pendingCmd.param        = param;
  pendingCmd.retryCount   = 0;
  pendingCmd.lastSentTime = 0;   // zero forces an immediate first attempt
}


// =============================================================================
//  RETRY DRIVER
//
//  Called every loop. Re-sends the pending command on the retry interval, and
//  reports a timeout to Node-RED once the attempt budget is spent — so a failed
//  command is visible to the operator instead of disappearing.
// =============================================================================
void handleRetry() {
  if (!pendingCmd.active) return;

  uint32_t now = millis();
  if (now - pendingCmd.lastSentTime < RETRY_INTERVAL) return;

  if (pendingCmd.retryCount >= RETRY_MAX) {
    String cmdName = (pendingCmd.cmd == CMD_TIMED) ? "timed" : "stop";
    Serial.println("{\"status\":\"cmd_timeout\",\"cmd\":\"" + cmdName + "\"}");
    pendingCmd.active = false;
    return;
  }

  sendCommandLoRa(pendingCmd.cmd, pendingCmd.param);
  pendingCmd.lastSentTime = now;
  pendingCmd.retryCount++;
}


// =============================================================================
//  INBOUND COMMANDS FROM NODE-RED (USB serial)
//
//  Accepts one JSON object per line:
//    {"cmd":"timed","duration":30}   or   {"cmd":"stop"}
//
//  The line buffer is capped so a malformed stream with no newline cannot grow
//  without bound and exhaust memory.
//
//  KNOWN ISSUE 4: containsKey() is deprecated in ArduinoJson v7; the modern
//  form is doc["cmd"].is<const char*>().
// =============================================================================
void handleSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      serialCmdBuf.trim();

      if (serialCmdBuf.length() > 0) {
        JsonDocument doc;

        if (!deserializeJson(doc, serialCmdBuf) && doc.containsKey("cmd")) {
          const char* cmdStr = doc["cmd"];
          uint8_t  cmd   = 0;
          uint32_t param = 0;
          String   cmdName;

          if (strcmp(cmdStr, "timed") == 0) {
            cmd     = CMD_TIMED;
            param   = doc["duration"] | 10;   // default 10 s if omitted
            cmdName = "timed";
          } else if (strcmp(cmdStr, "stop") == 0) {
            cmd     = CMD_STOP;
            param   = 0;
            cmdName = "stop";
          } else {
            Serial.println("{\"error\":\"unknown_cmd\"}");
            serialCmdBuf = "";
            return;
          }

          startPendingCmd(cmd, param);

          // Echo back immediately so the operator sees the command was accepted
          // here, distinct from the later cmd_ack that confirms the aircraft
          // received it.
          JsonDocument echo;
          echo["status"] = "cmd_sending";
          echo["cmd"]    = cmdName;
          if (param > 0) echo["param"] = param;

          String echoStr;
          serializeJson(echo, echoStr);
          Serial.println(echoStr);
        }
      }
      serialCmdBuf = "";

    } else {
      serialCmdBuf += c;
      if (serialCmdBuf.length() > 200) serialCmdBuf = "";   // guard against runaway input
    }
  }
}


// =============================================================================
//  LORA RECEIVE CALLBACK
//
//  Invoked from the DIO0 interrupt. Copies the frame out of the radio FIFO and
//  raises a flag; all parsing and serial output happens in the main loop.
//
//  packetReady is checked on entry so a packet arriving before the previous one
//  has been consumed is dropped rather than overwriting a buffer mid-read.
//
//  KNOWN ISSUE 1: this is interrupt context but the function is not marked
//  IRAM_ATTR, so it will fault if invoked while the flash cache is disabled.
//  The buffers it writes are also not volatile.
// =============================================================================
void onLoRaReceive(int packetSize) {
  if (packetSize == 0 || packetReady) return;
  if (packetSize > RX_BUF_SIZE) packetSize = RX_BUF_SIZE;

  int idx = 0;
  while (LoRa.available() && idx < packetSize) {
    rxBuf[idx++] = (uint8_t)LoRa.read();
  }

  rxLen  = idx;
  rxRSSI = LoRa.packetRssi();
  rxSNR  = LoRa.packetSnr();

  packetReady = true;   // set last, so the buffer is complete before it is read

  digitalWrite(LED_PIN, HIGH);
  ledPending = true;
  ledOnTime  = 0;       // the main loop timestamps it
}


// =============================================================================
//  NON-BLOCKING RECEIVE INDICATOR — 80 ms flash per packet
// =============================================================================
void handleLED() {
  if (!ledPending) return;

  if (ledOnTime == 0) ledOnTime = millis();

  if (millis() - ledOnTime >= 80) {
    digitalWrite(LED_PIN, LOW);
    ledPending = false;
    ledOnTime  = 0;
  }
}


// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Block until the radio initialises. A dongle that enumerates but cannot
  // receive would look healthy to the operator while silently reporting nothing.
  while (!setupLoRa()) {
    Serial.println("{\"error\":\"lora_init_fail\"}");
    delay(2000);
  }

  LoRa.onReceive(onLoRaReceive);
  LoRa.receive();

  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);

  // Announce configuration and struct sizes on boot. The packet sizes let
  // Node-RED — or a human reading the log — confirm both ends agree on the
  // wire format before trusting any measurement.
  JsonDocument info;
  info["status"]          = "receiver_ready";
  info["version"]         = "5.0";
  info["freq_mhz"]        = 433;
  info["sf"]              = LORA_SF;
  info["cont_pkt_bytes"]  = sizeof(RadioPacketCont);
  info["timed_pkt_bytes"] = sizeof(RadioPacketTimed);
  info["ack_pkt_bytes"]   = sizeof(AckPacket);
  info["cmd_pkt_bytes"]   = sizeof(CmdPacket);

  String infoStr;
  serializeJson(info, infoStr);
  Serial.println(infoStr);
}


// =============================================================================
//  MAIN LOOP
// =============================================================================
void loop() {
  handleSerialCommand();
  handleLED();
  handleRetry();

  if (packetReady) {
    // Clear the flag with interrupts masked so a packet arriving at this exact
    // moment cannot be lost between the test and the clear.
    noInterrupts();
    packetReady = false;
    interrupts();

    routePacket();

    LoRa.receive();   // return to receive mode after handling
  }

  delay(5);
}
