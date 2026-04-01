#include "communication.h"

#include <cstdio>
#include <cstring>

namespace Communication {

const LoRaTransmitter::Pins kDefaultLoRaPins = {
  .sck  = 18,
  .miso = 19,
  .mosi = 23,
  .cs   = 27,
  .rst  = 14,
  .dio0 = 26,
  .led  = 2
};

/*
VCC -> 3V3
GND -> GND
SCK -> D18
MISO -> D19
MOSI -> D23
NSS/CS -> D27
RST -> D14
DIO0 -> D26
*/

const LoRaTransmitter::Config kDefaultLoRaConfig = {
  .frequencyMHz     = 434.0,
  .bandwidthkHz     = 125.0,
  .spreadingFactor  = 12,
  .codingRateDenom  = 8,
  .syncWord         = 0x12,
  .txPowerdBm       = 20,
  .preambleLength   = 12,
  .sendIntervalMs   = 5000
};

void Logger::begin(unsigned long baud) {
  Serial.begin(baud);

  const uint32_t start = millis();
  while (!Serial && (millis() - start < 3000)) {
    delay(10);
  }

  delay(300);
  Serial.println();
  Serial.println("==================================================");
  Serial.println("[BOOT] ESP32 LoRa transmitter booting...");
  Serial.println("==================================================");
  Serial.flush();
}

void Logger::info(const char* tag, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  log("INFO", tag, fmt, args);
  va_end(args);
}

void Logger::warn(const char* tag, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  log("WARN", tag, fmt, args);
  va_end(args);
}

void Logger::error(const char* tag, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  log("ERROR", tag, fmt, args);
  va_end(args);
}

void Logger::log(const char* level, const char* tag, const char* fmt, va_list args) {
  char message[256];
  vsnprintf(message, sizeof(message), fmt, args);
  Serial.printf("[%10lu ms] %-5s %-10s %s\r\n",
                static_cast<unsigned long>(millis()),
                level,
                tag,
                message);
  Serial.flush();
}

LoRaTransmitter::LoRaTransmitter(const Pins& pins, const Config& config)
  : pins_(pins),
    config_(config),
    spiBus_(VSPI),
    spiSettings_(2000000, MSBFIRST, SPI_MODE0),
    radio_(new Module(pins.cs, pins.dio0, pins.rst, RADIOLIB_NC, spiBus_, spiSettings_)) {}

bool LoRaTransmitter::begin() {
  Logger::info("APP", "LoRa transmitter begin()");
  printBoardInfo();
  printPins();
  printConfig();

  initLed();
  blinkLed(2, 120);

  Logger::info("GPIO", "Preparing radio control pins");
  pinMode(pins_.cs, OUTPUT);
  digitalWrite(pins_.cs, HIGH);

  pinMode(pins_.rst, OUTPUT);
  digitalWrite(pins_.rst, HIGH);

  pinMode(pins_.dio0, INPUT);

  Logger::info("GPIO", "Manual SX1278 reset on RST pin %d", pins_.rst);
  hardwareReset();

  Logger::info("SPI", "Starting VSPI: SCK=%d MISO=%d MOSI=%d CS=%d",
               pins_.sck, pins_.miso, pins_.mosi, pins_.cs);
  spiBus_.begin(pins_.sck, pins_.miso, pins_.mosi, pins_.cs);
  delay(50);

  Logger::info("RADIO", "Calling radio.begin(...)");
  const int16_t state = radio_.begin(
    config_.frequencyMHz,
    config_.bandwidthkHz,
    config_.spreadingFactor,
    config_.codingRateDenom,
    config_.syncWord,
    config_.txPowerdBm,
    config_.preambleLength
  );

  if (state != RADIOLIB_ERR_NONE) {
    logFailure("radio.begin() failed", state);
    return false;
  }

  Logger::info("RADIO", "radio.begin() OK");

  const int16_t chipVersion = radio_.getChipVersion();
  Logger::info("RADIO", "Chip version register = 0x%02X", chipVersion);

  if (chipVersion != 0x12 && chipVersion != 0x22) {
    Logger::error("RADIO", "Unexpected chip version. Check wiring/power/module type.");
    return false;
  }

  const int16_t crcState = radio_.setCRC(true);
  if (crcState != RADIOLIB_ERR_NONE) {
    logFailure("setCRC(true) failed", crcState);
    return false;
  }
  Logger::info("RADIO", "CRC enabled");

  const int16_t standbyState = radio_.standby();
  if (standbyState != RADIOLIB_ERR_NONE) {
    logFailure("standby() failed", standbyState);
    return false;
  }
  Logger::info("RADIO", "Radio set to standby");

  initialized_ = true;
  lastSendAtMs_ = millis();
  lastHeartbeatAtMs_ = millis();

  Logger::info("APP", "Initialization complete. Ready to transmit.");
  blinkLed(3, 80);
  return true;
}

void LoRaTransmitter::loop() {
  heartbeat();

  if (!initialized_) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastSendAtMs_ >= config_.sendIntervalMs) {
    sendPacket();
    lastSendAtMs_ = millis();
  }
}

void LoRaTransmitter::failureLoop() {
  Logger::error("APP", "Entering failure loop. Radio init failed.");
  while (true) {
    digitalWrite(pins_.led, HIGH);
    delay(150);
    digitalWrite(pins_.led, LOW);
    delay(850);
    Serial.println("[FAIL] Check SX1278 wiring, 3.3V power, antenna, and pins.");
    Serial.flush();
  }
}

void LoRaTransmitter::initLed() {
  pinMode(pins_.led, OUTPUT);
  digitalWrite(pins_.led, LOW);
}

void LoRaTransmitter::blinkLed(uint8_t count, uint16_t ms) {
  for (uint8_t i = 0; i < count; ++i) {
    digitalWrite(pins_.led, HIGH);
    delay(ms);
    digitalWrite(pins_.led, LOW);
    delay(ms);
  }
}

void LoRaTransmitter::hardwareReset() {
  digitalWrite(pins_.rst, HIGH);
  delay(10);
  digitalWrite(pins_.rst, LOW);
  delay(20);
  digitalWrite(pins_.rst, HIGH);
  delay(20);
}

void LoRaTransmitter::heartbeat() {
  const uint32_t now = millis();
  if (now - lastHeartbeatAtMs_ >= 5000) {
    uint32_t nextTxIn = 0;
    if (initialized_) {
      const uint32_t elapsed = now - lastSendAtMs_;
      nextTxIn = (elapsed < config_.sendIntervalMs) ? (config_.sendIntervalMs - elapsed) : 0;
    }

    Logger::info("HEARTBEAT", "alive=%s freeHeap=%lu nextTxIn=%lu ms",
                 initialized_ ? "yes" : "no",
                 static_cast<unsigned long>(ESP.getFreeHeap()),
                 static_cast<unsigned long>(nextTxIn));

    lastHeartbeatAtMs_ = now;
  }
}

void LoRaTransmitter::sendPacket() {
  char payload[kPayloadSize];
  snprintf(
    payload,
    sizeof(payload),
    "{\"seq\":%lu,\"uptime_ms\":%lu,\"msg\":\"Hello There\"}",
    static_cast<unsigned long>(sequence_++),
    static_cast<unsigned long>(millis())
  );

  const size_t payloadLen = strnlen(payload, sizeof(payload));
  const unsigned long toaUs = static_cast<unsigned long>(radio_.getTimeOnAir(payloadLen));

  Logger::info("TX", "Sending payload: %s", payload);
  Logger::info("TX", "Payload length=%u bytes, estimated ToA=%lu us (~%lu ms)",
               static_cast<unsigned>(payloadLen),
               toaUs,
               toaUs / 1000UL);

  digitalWrite(pins_.led, HIGH);
  const int16_t state = radio_.transmit(payload);
  digitalWrite(pins_.led, LOW);

  if (state == RADIOLIB_ERR_NONE) {
    Logger::info("TX", "Packet sent successfully");
  } else {
    logFailure("transmit() failed", state);
  }
}

void LoRaTransmitter::printBoardInfo() const {
  Logger::info("BOARD", "Chip model: %s", ESP.getChipModel());
  Logger::info("BOARD", "Chip revision: %d", ESP.getChipRevision());
  Logger::info("BOARD", "CPU freq: %d MHz", ESP.getCpuFreqMHz());
  Logger::info("BOARD", "Flash size: %u bytes", ESP.getFlashChipSize());
  Logger::info("BOARD", "Free heap: %lu bytes", static_cast<unsigned long>(ESP.getFreeHeap()));
}

void LoRaTransmitter::printPins() const {
  Logger::info("PINS", "SCK=%d MISO=%d MOSI=%d CS=%d RST=%d DIO0=%d LED=%d",
               pins_.sck, pins_.miso, pins_.mosi, pins_.cs, pins_.rst, pins_.dio0, pins_.led);
}

void LoRaTransmitter::printConfig() const {
  Logger::info("CFG", "Frequency: %.3f MHz", config_.frequencyMHz);
  Logger::info("CFG", "Bandwidth: %.1f kHz", config_.bandwidthkHz);
  Logger::info("CFG", "Spreading Factor: %u", config_.spreadingFactor);
  Logger::info("CFG", "Coding Rate: 4/%u", config_.codingRateDenom);
  Logger::info("CFG", "Sync Word: 0x%02X", config_.syncWord);
  Logger::info("CFG", "TX Power: %d dBm", config_.txPowerdBm);
  Logger::info("CFG", "Preamble Length: %u", config_.preambleLength);
  Logger::info("CFG", "Send Interval: %lu ms", static_cast<unsigned long>(config_.sendIntervalMs));
}

void LoRaTransmitter::logFailure(const char* what, int16_t code) const {
  Logger::error("RADIO", "%s | code=%d | %s", what, code, statusToString(code));
}

const char* LoRaTransmitter::statusToString(int16_t code) const {
  switch (code) {
    case RADIOLIB_ERR_NONE: return "No error";
    case RADIOLIB_ERR_CHIP_NOT_FOUND: return "Chip not found";
    case RADIOLIB_ERR_INVALID_FREQUENCY: return "Invalid frequency";
    case RADIOLIB_ERR_INVALID_BANDWIDTH: return "Invalid bandwidth";
    case RADIOLIB_ERR_INVALID_SPREADING_FACTOR: return "Invalid spreading factor";
    case RADIOLIB_ERR_INVALID_CODING_RATE: return "Invalid coding rate";
    case RADIOLIB_ERR_INVALID_OUTPUT_POWER: return "Invalid output power";
    case RADIOLIB_ERR_TX_TIMEOUT: return "TX timeout";
    case RADIOLIB_ERR_SPI_WRITE_FAILED: return "SPI write/read verification failed";
    default: return "Unknown RadioLib status";
  }
}

}  // namespace Communication
