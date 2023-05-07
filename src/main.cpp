#include <Arduino.h>
#include "Settings.h"
#include "RelayController.h"
#include "Communicator.h"

Settings data;
RelayController relayController(data);
Communicator communicator(data, relayController);


void setup() {
    data.load();
    relayController.setup();
    communicator.setup();
}

void loop() {
    relayController.idle();
    communicator.idle();
}