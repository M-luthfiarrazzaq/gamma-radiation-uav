/*
 * =============================================================================
 *  TRANSMITTER FIRMWARE — Airborne Gamma Radiation Detection System
 * =============================================================================
 *
 *  Board    : ESP32-S3-N16R8 (custom PCB, hexacopter-mounted)
 *  Author   : Muhammad Luthfi Ar-Razzaq (NIM 022200025)
 *  Institute: Politeknik Teknologi Nuklir Indonesia — BRIN
 *  Version  : 5.1
 *
 *  -------------------------------------------------------------------------
 *  WHAT THIS DOES
 *  -------------------------------------------------------------------------
 *  Counts pulses from a Geiger-Muller tube, tags them with position, and
 *  transmits the result to a ground station over LoRa. Two acquisition modes:
 *
 *    Continuous  — transmits count rate and dose rate every 2 s for live
 *                  monitoring during flight.
 *    Timed Count — counts for a fixed operator-specified duration, then
 *                  transmits a single result with total accumulated dose.
 *
 *  Altitude is taken from the flight controller over MAVLink where available,
 *  falling back to the on-board GPS if the FC link goes quiet. The node also
 *  feeds its own GPS fix back to the flight controller as a GPS_INPUT source.
 *
 *  After every transmission the radio opens a 1 s listen window, which gives
 *  the operator a command channel on what is otherwise a one-way link.
 *
 *  -------------------------------------------------------------------------
 *  CHANGES IN v5.1 (from v5.0)
 *  -------------------------------------------------------------------------
 *    - Added requestGlobalPositionInt(): explicitly asks the FC for message 33
 *      via MAV_CMD_SET_MESSAGE_INTERVAL rather than relying on the SRx_POSITION
 *      parameter being configured correctly.
 *    - Added an FC link timeout: if no message arrives for FC_TIMEOUT_MS the
 *      node falls back to GPS altitude automatically.
 *    - The message request is re-sent automatically whenever the FC link drops,
 *      so recovery needs no operator action.
 *
 *  -------------------------------------------------------------------------
 *  PIN MAPPING
 *  -------------------------------------------------------------------------
 *    Geiger counter
 *      IO47   TTL pulse input, FALLING-edge interrupt
 *
 *    GPS — u-blox NEO-6M (UART1, 9600 baud)
 *      IO17   ESP32 RX  <- GPS TX
 *      IO18   ESP32 TX  -> GPS RX
 *
 *    Flight controller — ArduPilot (UART2, 115200 baud, MAVLink 2)
 *      IO2    ESP32 RX  <- FC TX
 *      IO38   ESP32 TX  -> FC RX
 *
 *    LoRa — SX1278 / RA-02 (SPI)
 *      IO10   NSS/CS        IO12   SCK
 *      IO11   MOSI          IO13   MISO
 *      IO16   RST           IO15   DIO0
 *
 *    Status
 *      IO48   On-board LED
 *
 *  -------------------------------------------------------------------------
 *  DEPENDENCIES
 *  -------------------------------------------------------------------------
 *    LoRa       — Sandeep Mistry   v0.8.0+
 *    TinyGPS++  — Mikal Hart       v1.0.3+
 *    MAVLink2   — gmelchett        (Arduino Library Manager)
 *
 *  Flight controller must be configured with GPS_TYPE = 14 (MAVLink GPS input)
 *  and GPS_TYPE2 = 0 for sendGPSToFC() to be accepted.
 *
 *  -------------------------------------------------------------------------
 *  KNOWN ISSUES — identified during review, deliberately left unfixed here so
 *  that this build matches the firmware used to produce the thesis results.
 *  Each is marked inline with a "KNOWN ISSUE" comment at the relevant code.
 *  -------------------------------------------------------------------------
 *    1. A STOP command cannot be received while a timed count is running.
 *       loop() returns early in MODE_TIMED_COUNT, so handleListenWindow() —
 *       the only path that reaches processCommand() — never executes. The
 *       CMD_STOP branch that handles an active count is therefore unreachable
 *       in practice. See loop() and handleListenWindow().
 *
 *    2. timedPulses and timedActive are read and written inside the ISR but
 *       are not declared volatile. See the sensor state block.
 *
 *    3. endPacket(true) starts an asynchronous transmit, and the radio is then
 *       switched to receive mode without waiting for it to finish. At SF10 /
 *       BW 125 kHz a 27-byte packet is >100 ms of airtime. See
 *       sendContinuousPacket() and sendAck().
 *
 *    4. Latitude and longitude travel as float32. Near 110 degrees longitude
 *       that quantises to roughly 0.85 m. MAVLink avoids this by sending
 *       int32 scaled by 1e7 — as sendGPSToFC() below does correctly.
 *
 *    5. This node packs MAVLink messages with system ID 1, which is also the
 *       flight controller's system ID. Two nodes must not share a system ID.
 *
 *    6. time_usec in sendGPSToFC() only has year resolution.
 *
 *    7. GM_PIN is configured as INPUT, not INPUT_PULLUP. Confirm the Geiger
 *       module drives the line actively before relying on this.
 * =============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <common/mavlink.h>


// =============================================================================
//  PIN ASSIGNMENTS
// =============================================================================
#define GM_PIN      47      // Geiger-Muller pulse input
#define GPS_RX      17
#define GPS_TX      18
#define FC_RX       2
#define FC_TX       38
#define LORA_CS     10
#define LORA_RST    16
#define LORA_DIO0   15
#define LORA_SCK    12
#define LORA_MOSI   11
#define LORA_MISO   13
#define LED_PIN     48


// =============================================================================
//  SERIAL BAUD RATES
// =============================================================================
#define GPS_BAUD    9600
#define FC_BAUD     115200


// =============================================================================
//  LORA CONFIGURATION
//  Must match the receiver exactly. SF10 was chosen to trade data rate for
//  link margin; ground testing established a validated range of 350 m at a
//  95%+ packet success rate with these settings.
// =============================================================================
#define LORA_FREQ       433E6
#define LORA_SF         10
#define LORA_BW         125E3
#define LORA_CR         8
#define LORA_TX_POWER   20


// =============================================================================
//  TIMING
// =============================================================================
#define SEND_INTERVAL   2000UL    // continuous-mode transmit period
#define LISTEN_WINDOW   1000UL    // command window opened after each transmit
#define CPM_WINDOW_MS   2000UL    // pulse integration window for CPM


// =============================================================================
//  FLIGHT CONTROLLER LINK HEALTH
// =============================================================================
#define FC_TIMEOUT_MS        3000UL   // treat FC as lost after this much silence
#define FC_REQUEST_RETRY_MS  2000UL   // re-request message 33 at this interval
#define FC_TARGET_SYSID      1        // ArduPilot default system ID
#define FC_TARGET_COMPID     1        // autopilot default component ID

// GPS is forwarded to the flight controller at 5 Hz, per ArduPilot guidance.
#define GPS_TO_FC_INTERVAL_MS  200UL
uint32_t lastGPStoFCTime = 0;


// =============================================================================
//  PACKET TYPE IDENTIFIERS
//  The first byte of every radio frame. Must match the receiver.
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
//  #pragma pack(push, 1) removes inter-field padding so the struct can be
//  written straight to the radio and reconstructed byte-for-byte at the far
//  end. These definitions are duplicated verbatim in the receiver firmware;
//  any change here must be mirrored there or the CRC check will fail.
// =============================================================================
#pragma pack(push, 1)

// Continuous measurement, TX -> RX.  1+4+4+4+4+4+4+2 = 27 bytes
struct RadioPacketCont {
  uint8_t  pktType;         // PKT_CONT
  uint32_t id;              // monotonic packet sequence number
  uint32_t cpm;             // counts per minute
  float    doseRate;        // dose rate, uSv/h
  float    lat;             // latitude, decimal degrees   (see known issue 4)
  float    lon;             // longitude, decimal degrees  (see known issue 4)
  float    alt;             // altitude, m — FC preferred, GPS fallback
  uint16_t crc;             // CRC-16/CCITT over all preceding bytes
};

// Timed count result, TX -> RX.  1+4+4+4+4+4+4+4+4+4+2 = 39 bytes
struct RadioPacketTimed {
  uint8_t  pktType;         // PKT_TIMED
  uint32_t id;
  uint32_t durationMs;      // actual elapsed count duration, milliseconds
  uint32_t pulses;          // raw pulses counted over that duration
  float    cpm;             // counts per minute, derived
  float    doseRate;        // uSv/h
  float    totalDose;       // accumulated dose over the count, uSv
  float    lat;
  float    lon;
  float    alt;
  uint16_t crc;
};

// Command acknowledgement, TX -> RX.  1+1+4+2 = 8 bytes
struct AckPacket {
  uint8_t  pktType;         // PKT_ACK
  uint8_t  cmdAcked;        // the command being acknowledged
  uint32_t param;           // echoed parameter, 0 if unused
  uint16_t crc;
};

// Operator command, RX -> TX.  1+1+4+2 = 8 bytes
struct CmdPacket {
  uint8_t  pktType;         // PKT_CMD
  uint8_t  cmd;             // CMD_TIMED or CMD_STOP
  uint32_t param;           // duration in seconds for CMD_TIMED
  uint16_t crc;
};

#pragma pack(pop)


// =============================================================================
//  CRC-16/CCITT  (polynomial 0x1021, initial value 0xFFFF)
//
//  The SX1278 has its own hardware CRC, enabled in setupLoRa(). This second
//  software CRC covers the struct contents specifically, so a frame that is
//  the right length but the wrong shape — a version mismatch between the two
//  ends, for instance — is rejected rather than silently misinterpreted.
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

// Computes the CRC over every byte of a packet struct except its trailing
// crc field, which is assumed to be the final uint16_t member.
template<typename T>
uint16_t calcStructCRC(const T& pkt) {
  return crc16((const uint8_t*)&pkt, sizeof(T) - sizeof(uint16_t));
}


// =============================================================================
//  NON-BLOCKING STATUS LED
//
//  Blink patterns are driven from the main loop rather than with delay(), so
//  pulse counting and radio timing are never held up by indication.
// =============================================================================
struct LedJob {
  bool     active     = false;
  uint8_t  step       = 0;
  uint8_t  totalSteps = 0;    // two steps per blink: on, then off
  uint32_t lastTime   = 0;
  uint32_t duration   = 0;    // milliseconds per step
};
LedJob ledJob;

void ledBlink(uint8_t times, uint32_t durationMs) {
  ledJob.active     = true;
  ledJob.step       = 0;
  ledJob.totalSteps = times * 2;
  ledJob.duration   = durationMs;
  ledJob.lastTime   = millis();
  digitalWrite(LED_PIN, HIGH);
}

void handleLED() {
  if (!ledJob.active) return;
  if (millis() - ledJob.lastTime < ledJob.duration) return;

  ledJob.step++;
  ledJob.lastTime = millis();

  if (ledJob.step >= ledJob.totalSteps) {
    digitalWrite(LED_PIN, LOW);
    ledJob.active = false;
    return;
  }
  digitalWrite(LED_PIN, ledJob.step % 2 == 0 ? HIGH : LOW);
}


// =============================================================================
//  STATE MACHINE
//
//  Two independent layers:
//    Mode    — what the instrument is measuring (continuous or timed count)
//    TxState — what the radio is doing (transmitting or listening)
// =============================================================================
enum Mode    { MODE_CONTINUOUS, MODE_TIMED_COUNT };
enum TxState { STATE_SENDING, STATE_LISTENING };

Mode    currentMode = MODE_CONTINUOUS;
TxState txState     = STATE_SENDING;


// =============================================================================
//  SENSOR AND LINK STATE
// =============================================================================
volatile uint32_t pulseCount = 0;   // incremented by the ISR, cleared each window

float    lastLat      = 0.0f;
float    lastLon      = 0.0f;
float    lastAltGPS   = 0.0f;       // altitude from the on-board GPS
float    lastAltFC    = 0.0f;       // relative altitude reported by the FC
bool     fcAltValid   = false;      // is the FC altitude currently trustworthy

uint32_t lastFcMsgTime     = 0;     // last time message 33 arrived from the FC
uint32_t lastFcRequestTime = 0;     // last time message 33 was requested

uint32_t lastCPM      = 0;
float    lastDoseRate = 0.0f;

uint32_t lastSendTime    = 0;
uint32_t listenStartTime = 0;
uint32_t cpmWindowStart  = 0;
uint32_t packetID        = 0;

// --- Timed count state ---
//
// KNOWN ISSUE 2: timedPulses and timedActive are both accessed inside
// onGeigerPulse(), which runs in interrupt context, but neither is declared
// volatile. The compiler is free to cache them in a register, so a timed count
// can under-report. Both should be volatile; timedActive should additionally be
// snapshotted before use in the ISR. Left as-is to match the tested build.
uint32_t timedDurationMs = 0;
uint32_t timedStartTime  = 0;
uint32_t timedPulses     = 0;
bool     timedActive     = false;

HardwareSerial gpsSerial(1);
HardwareSerial fcSerial(2);

mavlink_message_t mavMsg;
mavlink_status_t  mavStatus;

TinyGPSPlus gps;


// =============================================================================
//  INTERRUPT SERVICE ROUTINE — Geiger-Muller pulse
//
//  Kept to two increments and nothing else. IRAM_ATTR places it in internal RAM
//  so it remains callable when the flash cache is disabled.
//
//  Counting on an interrupt rather than polling is what allows every pulse to
//  be captured while the main loop is busy transmitting over LoRa or parsing
//  NMEA sentences — both of which take far longer than the interval between
//  pulses at elevated count rates.
// =============================================================================
void IRAM_ATTR onGeigerPulse() {
  pulseCount++;
  if (timedActive) timedPulses++;
}


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
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.enableCrc();
  return true;
}


// =============================================================================
//  GPS — drain the UART and cache the latest valid fix
// =============================================================================
void readGPS() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  if (gps.location.isValid()) {
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();
  }
  if (gps.altitude.isValid()) lastAltGPS = gps.altitude.meters();
}


// =============================================================================
//  REQUEST GLOBAL_POSITION_INT FROM THE FLIGHT CONTROLLER
//
//  Explicitly asks the FC to stream message 33 on this link at 5 Hz. Doing this
//  rather than relying on the SRx_POSITION parameter means the node works on an
//  FC that has not been pre-configured for it, and recovers by itself if the
//  link drops and the stream stops.
//
//  KNOWN ISSUE 5: this packs with system ID 1, which is also FC_TARGET_SYSID.
//  Two nodes on a MAVLink network must not share a system ID; a distinct value
//  (255 is the convention for a ground station) should be used instead.
// =============================================================================
void requestGlobalPositionInt() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  float intervalUs = 200000.0f;   // 200 ms -> 5 Hz

  mavlink_msg_command_long_pack(
    1,      // this node's system ID     (see known issue 5)
    200,    // this node's component ID
    &msg,
    FC_TARGET_SYSID,
    FC_TARGET_COMPID,
    MAV_CMD_SET_MESSAGE_INTERVAL,
    0,      // confirmation
    MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    intervalUs,
    0, 0, 0, 0, 0
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  fcSerial.write(buf, len);
}


// =============================================================================
//  FLIGHT CONTROLLER — parse inbound MAVLink and manage link health
//
//  Relative altitude from the FC is preferred over GPS altitude because it is
//  fused from the barometer and is both smoother and referenced to the launch
//  point, which is what matters for a survey flight.
// =============================================================================
void readFC() {
  while (fcSerial.available()) {
    uint8_t b = (uint8_t)fcSerial.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, b, &mavMsg, &mavStatus)) {
      if (mavMsg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t pos;
        mavlink_msg_global_position_int_decode(&mavMsg, &pos);

        lastAltFC     = pos.relative_alt / 1000.0f;   // mm -> m
        fcAltValid    = true;
        lastFcMsgTime = millis();
      }
    }
  }

  uint32_t now = millis();

  // Fall back to GPS altitude if the FC has gone quiet.
  if (fcAltValid && (now - lastFcMsgTime > FC_TIMEOUT_MS)) {
    fcAltValid = false;
    Serial.println("[FC] Timeout - falling back to GPS altitude");
  }

  // Keep re-requesting the stream for as long as the FC altitude is invalid.
  if (!fcAltValid && (now - lastFcRequestTime >= FC_REQUEST_RETRY_MS)) {
    lastFcRequestTime = now;
    requestGlobalPositionInt();
  }
}


// =============================================================================
//  SEND GPS TO THE FLIGHT CONTROLLER  (MAVLink GPS_INPUT, message 232)
//
//  Lets the detector node act as the aircraft's GPS source, so a single
//  receiver serves both navigation and measurement tagging.
//  Requires GPS_TYPE = 14 and GPS_TYPE2 = 0 on the flight controller.
//
//  Note the position is sent as int32 scaled by 1e7 here — full precision —
//  whereas the radio packet above carries float32. See known issue 4.
// =============================================================================
void sendGPSToFC() {
  if (!gps.location.isValid()) return;   // never send an unfixed position

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // KNOWN ISSUE 6: this expression only has year resolution, so it resolves to
  // 1 January of the current year rather than the current instant. Sending 0
  // and letting the FC use its own clock — which is the else branch — is more
  // honest than sending a timestamp that is wrong by months.
  uint64_t timeUsec = gps.time.isValid()
    ? (uint64_t)(gps.date.year() - 1970) * 31557600ULL * 1000000ULL
    : 0ULL;

  mavlink_gps_input_t gpsIn = {};

  gpsIn.time_usec    = timeUsec;
  gpsIn.gps_id       = 0;                 // GPS instance 0

  // Tell the FC which fields to disregard rather than trusting our zeros.
  gpsIn.ignore_flags =
    GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
    GPS_INPUT_IGNORE_FLAG_VEL_VERT  |
    GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY;

  gpsIn.time_week_ms = 0;
  gpsIn.time_week    = 0;
  gpsIn.fix_type     = gps.location.isValid() ? 3 : 0;   // 3 = 3D fix

  gpsIn.lat = (int32_t)(gps.location.lat() * 1e7);
  gpsIn.lon = (int32_t)(gps.location.lng() * 1e7);
  gpsIn.alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0f;

  gpsIn.hdop = gps.hdop.isValid() ? (float)gps.hdop.value() / 100.0f : 99.99f;
  gpsIn.vdop = 99.99f;                    // not exposed by TinyGPS++

  gpsIn.vn = 0.0f;                        // velocity ignored via ignore_flags
  gpsIn.ve = 0.0f;
  gpsIn.vd = 0.0f;
  gpsIn.speed_accuracy = 0.0f;

  gpsIn.horiz_accuracy = 2.5f;            // NEO-6M datasheet estimate, metres
  gpsIn.vert_accuracy  = 3.5f;

  gpsIn.satellites_visible =
    gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;

  mavlink_msg_gps_input_encode(
    1,      // this node's system ID     (see known issue 5)
    200,    // this node's component ID
    &msg,
    &gpsIn
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  fcSerial.write(buf, len);
}


// =============================================================================
//  COUNT RATE TO DOSE RATE
//
//  Empirical conversion derived by calibrating this detector against a
//  BAPETEN/KAN-certified reference survey meter using a Cs-137 source at six
//  measured distances:
//
//      dose rate (uSv/h) = 0.0028 * CPM + 0.1198        R2 = 0.9991
//
//  The slope is expressed directly in uSv/h per CPM so it can be applied here
//  with no unit inversion. The intercept is the fitted background offset, which
//  means this function never returns less than 0.1198 uSv/h.
//
//  The fit is empirical over the calibrated range. It does not include a
//  correction for Geiger tube dead time, so linearity is not guaranteed at
//  count rates well above those used for calibration.
// =============================================================================
float cpmToDoseRate(float cpm) {
  return cpm * 0.0028 + 0.1198;
}


// =============================================================================
//  COUNT RATE UPDATE
//
//  Pulses are accumulated by the ISR and converted to a rate once per window.
//  The read-and-clear of pulseCount is a read-modify-write and so is guarded
//  against the interrupt firing between the two operations.
//
//  Dividing by the measured elapsed time rather than the nominal window means
//  the result stays correct even if the loop is delayed.
// =============================================================================
void updateCPM() {
  uint32_t elapsed = millis() - cpmWindowStart;
  if (elapsed < CPM_WINDOW_MS) return;

  noInterrupts();
  uint32_t cnt = pulseCount;
  pulseCount   = 0;
  interrupts();

  cpmWindowStart = millis();
  lastCPM        = (uint32_t)((float)cnt * 60000.0f / (float)elapsed);
  lastDoseRate   = cpmToDoseRate(lastCPM);
}


// =============================================================================
//  ALTITUDE SELECTION — flight controller preferred, GPS as fallback
// =============================================================================
float getAlt() {
  return fcAltValid ? lastAltFC : lastAltGPS;
}


// =============================================================================
//  TRANSMIT — continuous measurement
//
//  KNOWN ISSUE 3: endPacket(true) is asynchronous and returns immediately. The
//  caller then invokes openListenWindow(), which puts the radio into receive
//  mode. At SF10 / BW 125 kHz this packet is over 100 ms of airtime, so the
//  mode change can abort the transmission. LoRa.isTransmitting() should be
//  polled before switching, or the transmit made synchronous.
// =============================================================================
void sendContinuousPacket() {
  RadioPacketCont pkt;
  pkt.pktType  = PKT_CONT;
  pkt.id       = packetID++;
  pkt.cpm      = lastCPM;
  pkt.doseRate = lastDoseRate;
  pkt.lat      = lastLat;
  pkt.lon      = lastLon;
  pkt.alt      = getAlt();
  pkt.crc      = calcStructCRC(pkt);

  LoRa.beginPacket();
  LoRa.write((const uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket(true);

  ledBlink(1, 50);

  Serial.printf("[TX-CONT] id=%lu cpm=%lu dose=%.4f lat=%.7f lon=%.7f alt=%.2f (%d bytes)\n",
    pkt.id, pkt.cpm, pkt.doseRate, pkt.lat, pkt.lon, pkt.alt, sizeof(pkt));
}


// =============================================================================
//  TRANSMIT — timed count result
//
//  Rate and dose are derived here rather than on the ground so the result is
//  self-contained and survives a partial telemetry loss.
// =============================================================================
void sendTimedResult(uint32_t durMs, uint32_t pulses) {
  float durS      = durMs / 1000.0f;
  float cpm       = (durS > 0) ? ((float)pulses / durS) * 60.0f : 0.0f;
  float doseRate  = cpmToDoseRate(cpm);
  float totalDose = doseRate * (durS / 3600.0f);   // uSv/h over the elapsed hours

  RadioPacketTimed pkt;
  pkt.pktType    = PKT_TIMED;
  pkt.id         = packetID++;
  pkt.durationMs = durMs;
  pkt.pulses     = pulses;
  pkt.cpm        = cpm;
  pkt.doseRate   = doseRate;
  pkt.totalDose  = totalDose;
  pkt.lat        = lastLat;
  pkt.lon        = lastLon;
  pkt.alt        = getAlt();
  pkt.crc        = calcStructCRC(pkt);

  LoRa.beginPacket();
  LoRa.write((const uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket(true);

  ledBlink(2, 100);

  Serial.printf("[TX-TIMED] id=%lu dur=%.1fs pulses=%lu cpm=%.2f dose=%.4f tot=%.6f (%d bytes)\n",
    pkt.id, durS, pkt.pulses, pkt.cpm, pkt.doseRate, pkt.totalDose, sizeof(pkt));
}


// =============================================================================
//  TRANSMIT — command acknowledgement
//
//  Closes the loop on the operator's command so the ground station can stop
//  retrying. Without this the receiver would have no way to distinguish a
//  command that was never heard from one that was heard and acted on.
//
//  KNOWN ISSUE 3 applies here too — see sendContinuousPacket().
// =============================================================================
void sendAck(uint8_t cmdAcked, uint32_t param = 0) {
  AckPacket pkt;
  pkt.pktType  = PKT_ACK;
  pkt.cmdAcked = cmdAcked;
  pkt.param    = param;
  pkt.crc      = calcStructCRC(pkt);

  LoRa.beginPacket();
  LoRa.write((const uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket(true);
  LoRa.receive();

  Serial.printf("[ACK] cmd=0x%02X param=%lu (%d bytes)\n", cmdAcked, param, sizeof(pkt));
}


// =============================================================================
//  PROCESS AN OPERATOR COMMAND
//
//  Validated in three stages before anything acts on it: length, packet type,
//  then CRC. A malformed frame is logged and discarded rather than guessed at.
// =============================================================================
void processCommand(const uint8_t* raw, int len) {
  if (len < (int)sizeof(CmdPacket)) {
    Serial.printf("[CMD] Invalid length: %d bytes\n", len);
    return;
  }

  CmdPacket cmd;
  memcpy(&cmd, raw, sizeof(cmd));

  if (cmd.pktType != PKT_CMD) {
    Serial.printf("[CMD] Not a command packet: pktType=0x%02X\n", cmd.pktType);
    return;
  }

  uint16_t expectedCRC = crc16(raw, sizeof(CmdPacket) - sizeof(uint16_t));
  if (cmd.crc != expectedCRC) {
    Serial.printf("[CMD] CRC mismatch: got=0x%04X expected=0x%04X\n", cmd.crc, expectedCRC);
    return;
  }

  Serial.printf("[CMD] Received cmd=0x%02X param=%lu\n", cmd.cmd, cmd.param);

  if (cmd.cmd == CMD_TIMED) {
    uint32_t durSec = cmd.param > 0 ? cmd.param : 10;   // default 10 s

    timedDurationMs = durSec * 1000UL;
    timedStartTime  = millis();
    timedPulses     = 0;
    timedActive     = true;
    currentMode     = MODE_TIMED_COUNT;

    sendAck(CMD_TIMED, durSec);
    ledBlink(3, 100);
    Serial.printf("[CMD] Timed count started: %lu s\n", durSec);

  } else if (cmd.cmd == CMD_STOP) {
    // KNOWN ISSUE 1: this branch is unreachable while a count is running,
    // because loop() returns early in MODE_TIMED_COUNT and never calls
    // handleListenWindow(). A STOP sent mid-count is therefore never received,
    // and the ground station's retry logic will report a timeout instead.
    if (timedActive) {
      uint32_t elapsed = millis() - timedStartTime;
      timedActive = false;
      currentMode = MODE_CONTINUOUS;

      sendAck(CMD_STOP, 0);
      sendTimedResult(elapsed, timedPulses);
      Serial.println("[CMD] Timed count aborted by operator");
    } else {
      sendAck(CMD_STOP, 0);
      Serial.println("[CMD] STOP received, no timed count active");
    }

  } else {
    Serial.printf("[CMD] Unknown command: 0x%02X\n", cmd.cmd);
  }
}


// =============================================================================
//  LISTEN WINDOW
//
//  LoRa is used here as a half-duplex point-to-point link with no MAC layer,
//  so a fixed window after each transmission is what makes the channel
//  bidirectional without risking a collision: the ground station only ever
//  transmits into a window it knows is open.
// =============================================================================
void openListenWindow() {
  LoRa.receive();
  listenStartTime = millis();
  txState = STATE_LISTENING;
}

void handleListenWindow() {
  int pktSize = LoRa.parsePacket();

  if (pktSize > 0) {
    uint8_t buf[64];
    int idx = 0;
    while (LoRa.available() && idx < (int)sizeof(buf)) {
      buf[idx++] = (uint8_t)LoRa.read();
    }
    processCommand(buf, idx);
  }

  if (millis() - listenStartTime >= LISTEN_WINDOW) {
    txState = STATE_SENDING;
  }
}


// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("=== GAMMA RADIATION TRANSMITTER v5.1 ===");
  Serial.printf("[INFO] RadioPacketCont  = %d bytes\n", sizeof(RadioPacketCont));
  Serial.printf("[INFO] RadioPacketTimed = %d bytes\n", sizeof(RadioPacketTimed));
  Serial.printf("[INFO] AckPacket        = %d bytes\n", sizeof(AckPacket));
  Serial.printf("[INFO] CmdPacket        = %d bytes\n", sizeof(CmdPacket));

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(100);
  Serial.println("[GPS] UART1 ready");

  fcSerial.begin(FC_BAUD, SERIAL_8N1, FC_RX, FC_TX);
  delay(100);
  Serial.println("[FC] UART2 ready - MAVLink 2");

  // KNOWN ISSUE 7: configured as INPUT, while the header documents
  // INPUT_PULLUP. Confirm the Geiger module drives this line actively.
  pinMode(GM_PIN, INPUT);
  delay(10);
  attachInterrupt(digitalPinToInterrupt(GM_PIN), onGeigerPulse, FALLING);
  Serial.println("[GM] Pulse interrupt attached");

  // Nothing this node does is meaningful without the radio, so block here
  // rather than starting up in a state that silently discards measurements.
  while (!setupLoRa()) {
    Serial.println("[LoRa] Init failed, retrying...");
    delay(2000);
  }
  Serial.println("[LoRa] Ready - 433 MHz SF10");

  cpmWindowStart = millis();
  lastSendTime   = millis();

  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);

  Serial.println("[READY] System active v5.1");
}


// =============================================================================
//  MAIN LOOP
//
//  Entirely non-blocking. Nothing here waits on anything, so pulse counting,
//  GPS parsing, FC telemetry and the radio schedule all proceed together.
// =============================================================================
void loop() {
  readGPS();
  readFC();
  handleLED();

  uint32_t now = millis();

  // Forward our GPS fix to the flight controller at a fixed rate.
  if (now - lastGPStoFCTime >= GPS_TO_FC_INTERVAL_MS) {
    lastGPStoFCTime = now;
    sendGPSToFC();
  }

  // ---- Timed count mode -------------------------------------------------
  //
  // KNOWN ISSUE 1: this branch returns before handleListenWindow() is reached,
  // so no inbound command — including STOP — can be received while a count is
  // running. Handling the listen window here as well would make the STOP
  // branch in processCommand() reachable.
  if (currentMode == MODE_TIMED_COUNT) {
    // Slow 1 Hz blink to indicate a count is in progress, but only when no
    // explicit blink pattern is already playing.
    if (!ledJob.active) {
      digitalWrite(LED_PIN, (now / 500) % 2);
    }

    if (timedActive && now - timedStartTime >= timedDurationMs) {
      timedActive  = false;
      currentMode  = MODE_CONTINUOUS;
      lastSendTime = now;
      sendTimedResult(now - timedStartTime, timedPulses);
    }
    return;
  }

  // ---- Continuous mode --------------------------------------------------
  updateCPM();

  if (txState == STATE_SENDING) {
    if (now - lastSendTime >= SEND_INTERVAL) {
      lastSendTime = now;
      sendContinuousPacket();
      openListenWindow();
    }
  } else {
    handleListenWindow();
  }
}
