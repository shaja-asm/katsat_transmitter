#pragma once

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <stdarg.h>

namespace Communication {

class Logger {
public:
  static void begin(unsigned long baud);
  static void info(const char* tag, const char* fmt, ...);
  static void warn(const char* tag, const char* fmt, ...);
  static void error(const char* tag, const char* fmt, ...);

private:
  static void log(const char* level, const char* tag, const char* fmt, va_list args);
};

class LoRaTransmitter {
public:
  struct Pins {
    int sck;
    int miso;
    int mosi;
    int cs;
    int rst;
    int dio0;
    int led;
  };

  struct Config {
    float frequencyMHz;
    float bandwidthkHz;
    uint8_t spreadingFactor;
    uint8_t codingRateDenom;
    uint8_t syncWord;
    int8_t txPowerdBm;
    uint16_t preambleLength;
    uint32_t sendIntervalMs;
  };

  LoRaTransmitter(const Pins& pins, const Config& config);

  bool begin();
  void loop();
  void failureLoop();

private:
  static constexpr size_t kPayloadSize = 128;

  Pins pins_;
  Config config_;
  SPIClass spiBus_;
  SPISettings spiSettings_;
  SX1278 radio_;

  bool initialized_ = false;
  uint32_t sequence_ = 0;
  uint32_t lastSendAtMs_ = 0;
  uint32_t lastHeartbeatAtMs_ = 0;

  void initLed();
  void blinkLed(uint8_t count, uint16_t ms);
  void hardwareReset();
  void heartbeat();
  void sendPacket();
  void printBoardInfo() const;
  void printPins() const;
  void printConfig() const;
  void logFailure(const char* what, int16_t code) const;
  const char* statusToString(int16_t code) const;
};

extern const LoRaTransmitter::Pins kDefaultLoRaPins;
extern const LoRaTransmitter::Config kDefaultLoRaConfig;


}  // namespace Communication
