#include <Arduino.h>
#include "Settings.h"
#include "RelayController.h"
#include "Server.h"

Settings data;
Server server(data);


void setup() {
    Serial.begin(18200);
    delay(100);
    data.load();
    RelayController::setup(data);
    server.setup();
}

void loop() {
    RelayController::idle();
    server.idle();
}