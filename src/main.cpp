// Roomba cuadrícula — Propuesta B
// ESP32 + 2× BTS7960 + encoders de un canal + BNO055 + interfaz web.
// Por ahora sin medición de corriente (IS), sin voltaje de batería y sin zumbador.
//
// Núcleo 1: control (100 Hz), seguridad (50 Hz), misión (20 Hz).
// Núcleo 0: Wi-Fi y servidor (AsyncTCP), planificador (bajo demanda), telemetría (10 Hz).
#include <Arduino.h>

#include "roomba_config.h"
#include "roomba_settings.h"
#include "Bts7960.h"
#include "SingleEncoder.h"
#include "Imu.h"
#include "Pose.h"
#include "Motion.h"
#include "Safety.h"
#include "Mission.h"
#include "WebApi.h"

Settings gSettings;

static Bts7960 motorL, motorR;
static SingleEncoder encL, encR;
static Imu bno;  // no se llama 'imu': choca con el namespace imu de Adafruit
static Pose pose;
static Motion motion;
static Safety safety;
static Mission mission;

static volatile bool planReady = false;

// ---------------------------------------------------------------- utilidades
static bool missionRunning() {
  Mission::State s = mission.state();
  Motion::State m = motion.state();
  return s == Mission::State::EJECUTANDO ||
         m == Motion::State::FORWARD || m == Motion::State::TURN || m == Motion::State::OPEN_LOOP;
}

// ---------------------------------------------------------------- tareas
static void controlTask(void *) {
  TickType_t last = xTaskGetTickCount();
  const float dt = CONTROL_PERIOD_MS / 1000.0f;
  for (;;) {
    motion.step(dt);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

static void safetyTask(void *) {
  TickType_t last = xTaskGetTickCount();
  Safety::Event prev = Safety::Event::NONE;
  for (;;) {
    Safety::Event ev = safety.step(missionRunning());
    if (ev != Safety::Event::NONE && ev != prev) {
      const char *msg = Safety::eventMessage(ev);
      Serial.printf("[seguridad] %s\n", msg);
      if (ev == Safety::Event::HEARTBEAT_LOST) {
        // Se frena (freno activo y EN en bajo); hay que volver a planear y ejecutar
        mission.emergencyStop();
      } else {
        motion.emergency();
        mission.fail(msg);
      }
      webapi::sendEvent(msg);
    }
    prev = ev;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SAFETY_PERIOD_MS));
  }
}

static void missionTask(void *) {
  TickType_t last = xTaskGetTickCount();
  bool btnPrev = true;
  uint32_t btnT = 0;
  Mission::State prevState = mission.state();
  for (;;) {
    mission.step();

    // Botón físico (activo en bajo): paro si está en marcha, ejecutar si hay plan, reanudar si está pausado
    bool btn = digitalRead(PIN_BUTTON);
    if (!btn && btnPrev && millis() - btnT > 300) {
      btnT = millis();
      const char *err = "";
      switch (mission.state()) {
        case Mission::State::EJECUTANDO: mission.emergencyStop(); break;
        case Mission::State::LISTO: mission.execute(true, &err); break;
        case Mission::State::PAUSADO: mission.togglePause(&err); break;
        default: motion.emergency(); break;
      }
    }
    btnPrev = btn;

    // LED: fijo = ejecutando, parpadeo lento = listo/pausado, rápido = falla
    Mission::State s = mission.state();
    uint32_t t = millis();
    bool led = false;
    switch (s) {
      case Mission::State::EJECUTANDO: led = true; break;
      case Mission::State::LISTO:
      case Mission::State::PAUSADO: led = (t / 500) & 1; break;
      case Mission::State::FALLA: led = (t / 100) & 1; break;
      default: led = (t % 2000) < 50; break;
    }
    digitalWrite(PIN_LED, led);

    if (s != prevState) {
      Serial.printf("[mision] %s -> %s (%s)\n", Mission::stateName(prevState), Mission::stateName(s),
                    mission.message());
      prevState = s;
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(MISSION_PERIOD_MS));
  }
}

static void plannerTask(void *) { mission.plannerLoop(); }

static void telemetryTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    if (planReady) {
      planReady = false;
      webapi::broadcastPlan();
    }
    webapi::sendTelemetry();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
  }
}

// ---------------------------------------------------------------- arranque
void setup() {
  // Primero, motores apagados (además de los pull-down de 10 kΩ en EN)
  pinMode(PIN_MOTORS_EN, OUTPUT);
  digitalWrite(PIN_MOTORS_EN, LOW);

  Serial.begin(115200);
  delay(100);
  Serial.println("\n[roomba-grid-b] arrancando");

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  gSettings.load();

  motorL.begin({PIN_L_RPWM, PIN_L_LPWM, LEDC_CH_L_R, LEDC_CH_L_L}, PWM_FREQ_HZ,
               PWM_BITS, LEFT_MOTOR_INVERT);
  motorR.begin({PIN_R_RPWM, PIN_R_LPWM, LEDC_CH_R_R, LEDC_CH_R_L}, PWM_FREQ_HZ,
               PWM_BITS, RIGHT_MOTOR_INVERT);

  if (!encL.begin(PIN_ENC_L, PCNT_UNIT_0, ENC_BOTH_EDGES, ENC_GLITCH_FILTER_NS, ENC_STOP_TIMEOUT_US))
    Serial.println("[enc] fallo PCNT izquierdo");
  if (!encR.begin(PIN_ENC_R, PCNT_UNIT_1, ENC_BOTH_EDGES, ENC_GLITCH_FILTER_NS, ENC_STOP_TIMEOUT_US))
    Serial.println("[enc] fallo PCNT derecho");

  Imu::Config ic{PIN_SDA, PIN_SCL, PIN_BNO_RST, I2C_FREQ, I2C_TIMEOUT_MS, IMU_I2C_ADDRESS,
                 IMU_USE_EXT_CRYSTAL, IMU_BEGIN_RETRIES, IMU_BOOT_WAIT_MS, IMU_YAW_SIGN};
  if (bno.begin(ic)) {
    Serial.printf("[imu] BNO055 en 0x%02X, IMUPLUS, offsets %s\n", IMU_I2C_ADDRESS,
                  bno.offsetsRestored() ? "restaurados" : "sin guardar (calibra desde la web)");
  } else {
    Serial.printf("[imu] BNO055 NO responde en 0x%02X (se reintenta cada 2 s)\n", IMU_I2C_ADDRESS);
  }
  bno.update();
  bno.setYaw(0);

  motion.begin({&motorL, &motorR, &encL, &encR, &bno, &pose, PIN_MOTORS_EN});
  safety.begin(&motorL, &motorR, &encL, &encR);
  mission.begin(&motion, &pose, &bno);
  mission.onPlanReady([] { planReady = true; });

  // Tareas de tiempo real en el núcleo 1
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(safetyTask, "safety", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(missionTask, "mission", 4096, nullptr, 3, nullptr, 1);
  // Red y planificación en el núcleo 0
  xTaskCreatePinnedToCore(plannerTask, "planner", 8192, nullptr, 3, nullptr, 0);
  delay(10);  // que la tarea planner registre su handle antes de recibir pedidos

  webapi::begin({&mission, &motion, &safety, &bno, &pose});
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 6144, nullptr, 1, nullptr, 0);

  Serial.printf("[roomba-grid-b] listo: %s\n", webapi::networkInfo());
}

void loop() {
  // Todo corre en tareas; loop() solo cede el procesador.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
