#include <Arduino.h>

#include "communication.h"

using Communication::Logger;
using Communication::LoRaTransmitter;

LoRaTransmitter app(Communication::kDefaultLoRaPins, Communication::kDefaultLoRaConfig);

void setup() {
  Logger::begin(115200);
  Logger::info("SETUP", "setup() entered");

  if (!app.begin()) {
    Logger::error("SETUP", "Fatal initialization error");
    app.failureLoop();
  }

  Logger::info("SETUP", "setup() completed");
}

void loop() {
  app.loop();
}
