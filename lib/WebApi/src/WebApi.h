#pragma once
// Wi-Fi (punto de acceso o estación), servidor HTTP con la API REST y WebSocket /ws.
#include <Arduino.h>
#include "Mission.h"
#include "Motion.h"
#include "Safety.h"
#include "Imu.h"
#include "Pose.h"

namespace webapi {

struct Context {
  Mission *mission;
  Motion *motion;
  Safety *safety;
  Imu *bno;  // (no 'imu': choca con el namespace de Adafruit)
  Pose *pose;
};

void begin(const Context &ctx);       // Wi-Fi + mDNS + LittleFS + servidor
void sendTelemetry();                 // tarea de telemetría (cada 100 ms)
void broadcastPlan();                 // al terminar de planear
void sendEvent(const char *text);     // aviso para la interfaz
const char *networkInfo();            // "AP Roomba-UAMM-B 192.168.4.1" o "STA red 10.0.0.5"

}  // namespace webapi
