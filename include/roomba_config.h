#pragma once
// ============================================================================
//  Roomba cuadrícula — Propuesta B
//  Pines, constantes físicas y valores por defecto.
//  Lo marcado con (NVS) se puede cambiar desde la interfaz web sin recompilar.
// ============================================================================
#include <stdint.h>

// ---------------------------------------------------------------- Motores (BTS7960 / IBT-2)
constexpr int PIN_L_RPWM = 25;  // motor izquierdo, adelante
constexpr int PIN_L_LPWM = 26;  // motor izquierdo, atrás
constexpr int PIN_L_EN = 27;    // R_EN + L_EN unidos, pull-down de 10 kΩ
constexpr int PIN_R_RPWM = 16;  // motor derecho, adelante
constexpr int PIN_R_LPWM = 17;  // motor derecho, atrás
constexpr int PIN_R_EN = 13;    // R_EN + L_EN unidos, pull-down de 10 kΩ

constexpr uint32_t PWM_FREQ_HZ = 20000;  // 20 kHz, inaudible
constexpr uint8_t PWM_BITS = 10;         // 0..1023
constexpr uint8_t LEDC_CH_L_R = 0, LEDC_CH_L_L = 1, LEDC_CH_R_R = 2, LEDC_CH_R_L = 3;

// Si un motor avanza al revés, cambia su INVERT en vez de recablear.
constexpr bool LEFT_MOTOR_INVERT = false;
constexpr bool RIGHT_MOTOR_INVERT = false;

// ---------------------------------------------------------------- Encoders de un canal
constexpr int PIN_ENC_L = 34;  // solo entrada: pull-up externo de 4.7 kΩ si es colector abierto
constexpr int PIN_ENC_R = 35;
// Medido en tu robot: 254 cuentas por vuelta de rueda contando UN flanco.
// Si activas ENC_BOTH_EDGES, pon 508 en CUENTAS_POR_VUELTA (o ajústalo en la web).
constexpr bool ENC_BOTH_EDGES = false;
constexpr float CUENTAS_POR_VUELTA = 254.0f;  // (NVS)
constexpr uint16_t ENC_GLITCH_FILTER_NS = 10000;  // 10 µs (PCNT admite hasta ~12.7 µs)
constexpr uint32_t ENC_STOP_TIMEOUT_US = 100000;  // sin pulsos en 100 ms → velocidad 0
constexpr uint32_t ENC_STILL_BEFORE_REVERSE_MS = 100;  // quieto antes de invertir sentido

// ---------------------------------------------------------------- Geometría
// Valores medidos en tu robot (la propuesta decía r = 35 mm y B = 234 mm).
constexpr float WHEEL_RADIUS_M = 0.0357f;  // (NVS)
constexpr float WHEEL_BASE_M = 0.235f;     // (NVS) distancia entre ruedas
constexpr float CELL_SIZE_M = 0.50f;

// ---------------------------------------------------------------- BNO055
constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;
constexpr int PIN_BNO_RST = 23;           // reinicio por hardware si el bus se bloquea (activo en bajo)
constexpr uint32_t I2C_FREQ = 100000;     // 100 kHz por el clock stretching del BNO055
constexpr uint16_t I2C_TIMEOUT_MS = 50;
// Tu módulo responde en 0x29 (ADR en alto). La propuesta decía 0x28.
constexpr uint8_t IMU_I2C_ADDRESS = 0x29;
constexpr bool IMU_USE_EXT_CRYSTAL = true;
constexpr uint8_t IMU_BEGIN_RETRIES = 5;      // begin() falla al primer intento: reintentar
constexpr uint16_t IMU_BOOT_WAIT_MS = 800;    // el BNO055 tarda ~650 ms en arrancar
// El BNO055 da el rumbo en sentido horario; el robot usa antihorario positivo.
constexpr float IMU_YAW_SIGN = -1.0f;

// ---------------------------------------------------------------- Interfaz física
constexpr int PIN_BUTTON = 18;  // paro / inicio, INPUT_PULLUP
constexpr int PIN_LED = 2;

// ---------------------------------------------------------------- Valores por defecto (NVS)
struct DefaultGains {
  // Velocidad por rueda: duty = signo·(PWM_MIN + KFF·|v|) + Kp·e + Ki·∫e
  static constexpr float WHEEL_KP = 1.2f;    // duty por (m/s)
  static constexpr float WHEEL_KI = 6.0f;    // duty por (m/s·s)
  static constexpr float WHEEL_KFF = 2.6f;   // duty por (m/s); 2.6 funcionó en tu otro robot
  static constexpr float WHEEL_PWM_MIN = 0.12f;  // duty para vencer la fricción estática
  // Avance: ω_ref = Kθ·eθ + Ky·ey
  static constexpr float HEADING_K = 4.0f;   // (rad/s)/rad
  static constexpr float LATERAL_K = 6.0f;   // (rad/s)/m
  // Giro en sitio: PD sobre el rumbo
  static constexpr float TURN_KP = 5.0f;     // (rad/s)/rad
  static constexpr float TURN_KD = 0.25f;
};

constexpr float V_MAX = 0.30f;            // m/s (NVS)
constexpr float ACCEL = 0.50f;            // m/s² (NVS)
constexpr float V_APPROACH = 0.08f;       // m/s, últimos 10 cm (NVS)
constexpr float APPROACH_DIST = 0.10f;    // m
constexpr float TURN_W_MAX_DEG = 90.0f;   // °/s (NVS)
constexpr float TURN_ALPHA_DEG = 180.0f;  // °/s² (NVS)
constexpr float TURN_W_MIN_DEG = 15.0f;   // °/s mínimos mientras falte giro
constexpr float TURN_TOL_DEG = 1.0f;      // |eθ| < 1°
constexpr float TURN_STILL_DEG_S = 3.0f;  // |ω| < 3 °/s
constexpr uint16_t TURN_SETTLE_MS = 100;  // ...durante 100 ms
constexpr uint8_t TURN_MAX_CORRECTIONS = 3;
constexpr float FORWARD_STOP_TICKS = 2.0f;  // distancia restante < 2 cuentas
constexpr uint16_t PRIMITIVE_PAUSE_MS = 150;  // pausa con freno entre primitivas
constexpr float SLIP_LIMIT_DEG = 3.0f;    // encoder vs BNO055 durante un avance
constexpr float TURN_COST = 0.5f;         // c_g del planificador (NVS)

// Atasco (sin sensor de corriente): PWM alto y sin pulsos del encoder durante STALL_TIME_MS
constexpr float STALL_MIN_DUTY = 0.25f;
constexpr uint16_t STALL_TIME_MS = 500;
constexpr uint16_t HEARTBEAT_TIMEOUT_MS = 2000;  // latido del navegador (NVS: activar/desactivar)

// ---------------------------------------------------------------- Wi-Fi
constexpr const char *AP_SSID = "Roomba-UAMM-B";
constexpr const char *AP_PASSWORD = "roomba-grid";  // WPA2: mínimo 8 caracteres. ¡Cámbiala!
constexpr const char *MDNS_NAME = "roomba-b";       // roomba-b.local en modo estación
constexpr uint32_t STA_CONNECT_TIMEOUT_MS = 15000;

// ---------------------------------------------------------------- Tareas (FreeRTOS)
constexpr uint32_t CONTROL_PERIOD_MS = 10;    // 100 Hz, núcleo 1
constexpr uint32_t SAFETY_PERIOD_MS = 20;     // núcleo 1
constexpr uint32_t MISSION_PERIOD_MS = 50;    // núcleo 1
constexpr uint32_t TELEMETRY_PERIOD_MS = 100; // núcleo 0
