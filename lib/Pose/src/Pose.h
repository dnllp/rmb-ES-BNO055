#pragma once
// Pose (x, y, θ): distancia de los encoders + rumbo absoluto del BNO055.
// El rumbo de los encoders solo se usa para detectar deslizamiento.
#include <math.h>
#include <stdint.h>

class Pose {
 public:
  float x = 0, y = 0, theta = 0;  // m, m, rad (continuo)

  void reset(float x0, float y0, float th0) {
    x = x0;
    y = y0;
    theta = th0;
    resetSlip();
  }

  // dnL, dnR: cuentas con signo de este ciclo. thetaImu: rumbo del BNO055 (rad, continuo).
  void update(int32_t dnL, int32_t dnR, float thetaImu, float metersPerTick, float wheelBase) {
    float dsL = dnL * metersPerTick;
    float dsR = dnR * metersPerTick;
    float ds = 0.5f * (dsL + dsR);
    // Integración con el rumbo medio del ciclo
    float thMid = 0.5f * (theta + thetaImu);
    x += ds * cosf(thMid);
    y += ds * sinf(thMid);
    float dThImu = thetaImu - theta;
    theta = thetaImu;
    distance += fabsf(ds);

    // Verificación de deslizamiento: Δθ de encoders vs Δθ del BNO055
    if (slipWatch) {
      slipEnc += (dsR - dsL) / wheelBase;
      slipImu += dThImu;
    }
  }

  // Diferencia acumulada encoder − IMU desde resetSlip() [rad]
  float slipError() const { return slipEnc - slipImu; }
  void resetSlip() {
    slipEnc = slipImu = 0;
  }

  bool slipWatch = false;
  uint32_t slipEvents = 0;
  float distance = 0;  // odómetro total [m]

 private:
  float slipEnc = 0, slipImu = 0;
};

// Utilidades de ángulos
inline float wrapPi(float a) {
  while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}
