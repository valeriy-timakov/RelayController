#include <Arduino.h>
#include "Settings.h"
#include "RelayController.h"
#include "Server.h"

Settings data;
Server communicator(data);


void setup() {
    Serial.begin(9600);
    delay(100);
    data.load();
    RelayController::setup(data);
    communicator.setup();
}

void loop() {
    RelayController::idle();
    communicator.idle();
}