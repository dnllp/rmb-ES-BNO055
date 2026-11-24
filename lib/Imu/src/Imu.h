#pragma once
// BNO055 en modo IMUPLUS (acelerómetro + giroscopio, sin magnetómetro).
// - Espera el arranque del sensor y reintenta begin() (en tu robot falla el primer intento).
// - Reinicio por el pin RST si el bus se bloquea o hay lecturas inválidas seguidas.
// - Offsets de calibración guardados en NVS y restaurados en cada arranque.
// - Rumbo continuo ("desenvuelto"), en radianes, antihorario positivo.
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class Imu {
 public:
  struct Config {
    int sda, scl, rst;
    uint32_t i2cFreq;
    uint16_t i2cTimeoutMs;
    uint8_t address;
    bool extCrystal;
    uint8_t retries;
    uint16_t bootWaitMs;
    float yawSign;
  };

  bool begin(const Config &c) {
    cfg_ = c;
    if (cfg_.rst >= 0) {
      pinMode(cfg_.rst, OUTPUT);
      digitalWrite(cfg_.rst, HIGH);
    }
    Wire.begin(cfg_.sda, cfg_.scl, cfg_.i2cFreq);
    Wire.setTimeOut(cfg_.i2cTimeoutMs);
    if (!bno_) bno_ = new Adafruit_BNO055(55, cfg_.address, &Wire);
    return start();
  }

  // Reinicio completo: pulso en RST, vuelve a configurar y restaura offsets.
  bool reset() {
    resets_++;
    if (cfg_.rst >= 0) {
      digitalWrite(cfg_.rst, LOW);
      delay(5);
      digitalWrite(cfg_.rst, HIGH);
    }
    bootT0_ = millis();
    return start();
  }

  // Lectura a 100 Hz desde la tarea de control. Devuelve false si la lectura no sirve.
  // Ojo: si el sensor se perdió, el reintento bloquea hasta ~2 s. Quien llama debe
  // frenar los motores en cuanto update() devuelva false (Motion lo hace).
  bool update() {
    if (!ok_) {
      // Reintento lento si el sensor se perdió (cable flojo): uno cada 2 s
      if (millis() - lastRetryMs_ > 2000) {
        lastRetryMs_ = millis();
        reset();
      }
      return false;
    }
    imu::Vector<3> e = bno_->getVector(Adafruit_BNO055::VECTOR_EULER);  // x = rumbo, 0..360° horario
    imu::Vector<3> gy = bno_->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);  // °/s
    float hdg = e.x();
    bool valid = !(isnan(hdg) || hdg < 0.0f || hdg > 360.0f);
    // La biblioteca no avisa si la lectura I2C falló (devuelve ceros): cada 0.5 s se
    // comprueba que el sensor responda (falsos contactos del cable).
    if (++pingCount_ >= 50) {
      pingCount_ = 0;
      Wire.beginTransmission(cfg_.address);
      if (Wire.endTransmission() != 0) {
        if (++pingFails_ >= 3) {
          ok_ = false;
          lastRetryMs_ = millis();
          return false;
        }
      } else {
        pingFails_ = 0;
        // Estado de calibración (en caché: todo el I2C ocurre en esta tarea)
        uint8_t cs = 0, cg = 0, ca = 0, cm = 0;
        bno_->getCalibration(&cs, &cg, &ca, &cm);
        calSys_ = cs;
        calGyro_ = cg;
        calAccel_ = ca;
        if (calibrating_ && calGyro_ == 3 && calAccel_ == 3) {
          calibrating_ = false;
          calSaved_ = saveOffsets();
        }
      }
    }
    if (!valid) {
      if (++failures_ > 100) {  // 1 s seguido de error: se reinicia en el siguiente ciclo
        failures_ = 0;
        ok_ = false;
        lastRetryMs_ = 0;
      }
      return false;
    }
    failures_ = 0;
    float yawDeg = cfg_.yawSign * hdg;
    if (!haveYaw_) {
      lastRawDeg_ = yawDeg;
      unwrappedDeg_ = yawDeg;
      haveYaw_ = true;
    } else {
      float d = yawDeg - lastRawDeg_;
      while (d > 180.0f) d -= 360.0f;
      while (d < -180.0f) d += 360.0f;
      unwrappedDeg_ += d;
      lastRawDeg_ = yawDeg;
    }
    // ω alrededor de z, antihorario positivo (z del sensor hacia arriba)
    yawRateDegS_ = gy.z();
    lastGoodMs_ = millis();
    return true;
  }

  // Rumbo continuo en radianes desde el arranque, más el desplazamiento de zero()
  float yaw() const { return (unwrappedDeg_ * DEG_TO_RAD) + offsetRad_; }
  float yawRate() const { return yawRateDegS_ * DEG_TO_RAD; }  // rad/s
  // Hace que el rumbo actual valga 'yawRad' (por ejemplo, el de la orientación de inicio)
  void setYaw(float yawRad) { offsetRad_ = yawRad - unwrappedDeg_ * DEG_TO_RAD; }

  bool ok() const { return ok_; }
  bool fresh(uint32_t maxAgeMs = 100) const { return ok_ && (millis() - lastGoodMs_) < maxAgeMs; }
  uint32_t resets() const { return resets_; }

  // Último estado de calibración leído (0–3). En IMUPLUS el giroscopio se calibra solo
  // con el robot quieto unos segundos; el acelerómetro, poniéndolo en varias posiciones.
  void calibration(uint8_t &sys, uint8_t &gyro, uint8_t &accel) const {
    sys = calSys_;
    gyro = calGyro_;
    accel = calAccel_;
  }

  // Calibración: cuando giroscopio y acelerómetro llegan a 3 se guardan los offsets en NVS.
  // Se revisa dentro de update(), en la tarea de control.
  void startCalibration() {
    calSaved_ = false;
    calibrating_ = ok_;
  }
  bool calibrating() const { return calibrating_; }
  bool calibrationSaved() const { return calSaved_; }

  bool saveOffsets() {
    if (!ok_) return false;
    adafruit_bno055_offsets_t off;
    if (!bno_->getSensorOffsets(off)) return false;
    Preferences p;
    if (!p.begin("imu", false)) return false;
    p.putBytes("off", &off, sizeof(off));
    p.end();
    return true;
  }

  bool offsetsRestored() const { return restored_; }

 private:
  bool start() {
    // El BNO055 tarda ~650 ms en arrancar tras encender o tras RST
    while (millis() - bootT0_ < cfg_.bootWaitMs) delay(10);
    ok_ = false;
    for (uint8_t i = 0; i < cfg_.retries && !ok_; i++) {
      ok_ = bno_->begin(OPERATION_MODE_IMUPLUS);
      if (!ok_) delay(200);
    }
    if (!ok_) return false;
    restored_ = restoreOffsets();
    bno_->setExtCrystalUse(cfg_.extCrystal);
    haveYaw_ = false;  // el rumbo del sensor vuelve a 0: se re-ancla sin salto
    failures_ = 0;
    pingFails_ = 0;
    lastGoodMs_ = millis();
    return true;
  }

  bool restoreOffsets() {
    Preferences p;
    if (!p.begin("imu", true)) return false;
    adafruit_bno055_offsets_t off;
    size_t n = p.getBytes("off", &off, sizeof(off));
    p.end();
    if (n != sizeof(off)) return false;
    bno_->setSensorOffsets(off);
    return true;
  }

  Config cfg_{};
  Adafruit_BNO055 *bno_ = nullptr;
  bool ok_ = false;
  bool restored_ = false;
  volatile bool calibrating_ = false;
  volatile bool calSaved_ = false;
  volatile uint8_t calSys_ = 0, calGyro_ = 0, calAccel_ = 0;
  bool haveYaw_ = false;
  uint32_t bootT0_ = 0;
  uint32_t lastRetryMs_ = 0;
  uint32_t lastGoodMs_ = 0;
  uint32_t resets_ = 0;
  uint16_t failures_ = 0;
  uint8_t pingCount_ = 0;
  uint8_t pingFails_ = 0;
  float lastRawDeg_ = 0, unwrappedDeg_ = 0, offsetRad_ = 0, yawRateDegS_ = 0;
};
