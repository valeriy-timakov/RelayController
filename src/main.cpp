#include <Arduino.h>
#include "Settings.h"
#include "RelayController.h"
#include "Server.h"

Settings data;
Server server(data);


void setup() {
    Serial.begin(9600);
    delay(100);
    data.load();
    RelayController::setup(data);
#ifdef MEM_32KB
    server.setup();
#else
    Server::setup();
#endif
}

void loop() {
    RelayController::idle();
    server.idle();
}